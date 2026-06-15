#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/nav.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

// Hybrid navigation server: exposes a simple (target_x, target_y) Nav action.
// If the target lies in known/free map space -> hand off to Nav2 (planned,
// obstacle-aware). If the target is unknown/unmapped -> drive there "blindly"
// with a simple /cmd_vel controller (no obstacle avoidance).
class NavigationServer : public rclcpp::Node
{
    public:
        using Navigation = action_interfaces::action::Nav;
        using GoalHandle = rclcpp_action::ServerGoalHandle<Navigation>;

        using NavToPose = nav2_msgs::action::NavigateToPose;
        using NavToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavToPose>;

        NavigationServer() : Node("navigation_server")
        {
            RCLCPP_INFO(this->get_logger(), "Navigation Server has been started.");

            nav_server_ = rclcpp_action::create_server<Navigation>(
                this, "navigate",
                std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
            );

            nav2_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            // /map is latched (transient local), so match that QoS
            rclcpp::QoS map_qos(rclcpp::KeepLast(1));
            map_qos.transient_local().reliable();
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", map_qos,
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    latest_map_ = msg;
                });

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                std::shared_ptr<const Navigation::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal request with target X:%.2f  Y:%.2f",
                        goal->target_x, goal->target_y);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received cancel request");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
            std::thread{[this, goal_handle](){ execute(goal_handle); }}.detach();
        }

        // Robot's real pose from TF (map -> robot_base). Returns false if unavailable.
        bool getRobotPose(double & x, double & y, double & yaw){
            try {
                auto tf = tf_buffer_->lookupTransform("map", "robot_base", tf2::TimePointZero);
                x = tf.transform.translation.x;
                y = tf.transform.translation.y;
                const auto & q = tf.transform.rotation;
                yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                return true;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", e.what());
                return false;
            }
        }

        // Is the target a known, free cell in the current map?
        // Returns false if there's no map, the cell is unknown (-1), out of
        // bounds, or occupied -> treated as "unexplored" => blind move.
        bool isExplored(double x, double y){
            std::lock_guard<std::mutex> lock(map_mutex_);
            if(!latest_map_){
                return false;
            }
            const auto & m = *latest_map_;
            double res = m.info.resolution;
            int col = static_cast<int>((x - m.info.origin.position.x) / res);
            int row = static_cast<int>((y - m.info.origin.position.y) / res);
            if(col < 0 || row < 0 ||
               col >= static_cast<int>(m.info.width) ||
               row >= static_cast<int>(m.info.height)){
                return false;  // outside the mapped area
            }
            int8_t v = m.data[row * m.info.width + col];
            return v >= 0 && v < 50;  // known and free-ish
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle){
            double tx = goal_handle->get_goal()->target_x;
            double ty = goal_handle->get_goal()->target_y;

            if(isExplored(tx, ty)){
                RCLCPP_INFO(this->get_logger(), "Target (%.2f, %.2f) is mapped -> using Nav2.", tx, ty);
                navigateWithNav2(goal_handle, tx, ty);
            } else {
                RCLCPP_INFO(this->get_logger(), "Target (%.2f, %.2f) is unmapped -> blind move.", tx, ty);
                blindMove(goal_handle, tx, ty);
            }
        }

        // ---- Mode 1: planned navigation via Nav2 ----
        void navigateWithNav2(const std::shared_ptr<GoalHandle> goal_handle, double tx, double ty){
            auto result = std::make_shared<Navigation::Result>();

            if(!nav2_client_->wait_for_action_server(5s)){
                RCLCPP_ERROR(this->get_logger(), "Nav2 (navigate_to_pose) not available, aborting.");
                result->status = "Nav2 not available";
                goal_handle->abort(result);
                return;
            }

            NavToPose::Goal nav2_goal;
            nav2_goal.pose.header.frame_id = "map";
            nav2_goal.pose.header.stamp = this->get_clock()->now();
            nav2_goal.pose.pose.position.x = tx;
            nav2_goal.pose.pose.position.y = ty;
            nav2_goal.pose.pose.orientation.w = 1.0;

            auto send_options = rclcpp_action::Client<NavToPose>::SendGoalOptions();
            send_options.feedback_callback =
                [this, goal_handle](NavToPoseGoalHandle::SharedPtr,
                                    const std::shared_ptr<const NavToPose::Feedback> nav2_fb){
                    auto fb = std::make_shared<Navigation::Feedback>();
                    fb->current_x = nav2_fb->current_pose.pose.position.x;
                    fb->current_y = nav2_fb->current_pose.pose.position.y;
                    fb->distance_remaining = nav2_fb->distance_remaining;
                    goal_handle->publish_feedback(fb);
                };

            auto gh_future = nav2_client_->async_send_goal(nav2_goal, send_options);
            if(gh_future.wait_for(5s) != std::future_status::ready){
                result->status = "send goal timeout";
                goal_handle->abort(result);
                return;
            }
            auto nav2_goal_handle = gh_future.get();
            if(!nav2_goal_handle){
                result->status = "rejected by Nav2";
                goal_handle->abort(result);
                return;
            }

            auto result_future = nav2_client_->async_get_result(nav2_goal_handle);
            while(rclcpp::ok()){
                if(goal_handle->is_canceling()){
                    nav2_client_->async_cancel_goal(nav2_goal_handle);
                    finish(goal_handle, result, "canceled", false, true);
                    return;
                }
                if(result_future.wait_for(100ms) == std::future_status::ready){
                    break;
                }
            }

            auto nav2_result = result_future.get();
            bool ok = (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED);
            finish(goal_handle, result, ok ? "Goal reached" : "Navigation failed", ok, false);
        }

        // ---- Mode 2: blind drive toward the point (no map, no avoidance) ----
        void blindMove(const std::shared_ptr<GoalHandle> goal_handle, double tx, double ty){
            auto result = std::make_shared<Navigation::Result>();

            const double goal_tol = 0.15;   // m
            const double yaw_tol  = 0.30;    // rad: turn in place above this
            const double max_lin  = 0.22;    // m/s
            const double max_ang  = 1.0;     // rad/s
            const double k_lin    = 0.5;
            const double k_ang    = 1.5;

            geometry_msgs::msg::Twist cmd;

            while(rclcpp::ok()){
                if(goal_handle->is_canceling()){
                    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop
                    finish(goal_handle, result, "canceled", false, true);
                    return;
                }

                double cx, cy, cyaw;
                if(!getRobotPose(cx, cy, cyaw)){
                    std::this_thread::sleep_for(100ms);
                    continue;
                }

                double dx = tx - cx;
                double dy = ty - cy;
                double dist = std::hypot(dx, dy);
                if(dist < goal_tol){
                    break;  // arrived
                }

                double yaw_err = normalizeAngle(std::atan2(dy, dx) - cyaw);
                cmd.angular.z = std::clamp(k_ang * yaw_err, -max_ang, max_ang);
                // only drive forward once roughly facing the target
                cmd.linear.x = (std::abs(yaw_err) > yaw_tol) ? 0.0
                                                             : std::min(k_lin * dist, max_lin);
                cmd_vel_pub_->publish(cmd);

                auto fb = std::make_shared<Navigation::Feedback>();
                fb->current_x = cx;
                fb->current_y = cy;
                fb->distance_remaining = dist;
                goal_handle->publish_feedback(fb);

                std::this_thread::sleep_for(50ms);
            }

            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop
            finish(goal_handle, result, "Goal reached (blind)", true, false);
        }

        // Fill final pose + status and terminate the action in the right state.
        void finish(const std::shared_ptr<GoalHandle> & goal_handle,
                    const std::shared_ptr<Navigation::Result> & result,
                    const std::string & status, bool succeeded, bool canceled){
            double x = 0.0, y = 0.0, yaw = 0.0;
            getRobotPose(x, y, yaw);
            result->status = status;
            result->final_x = x;
            result->final_y = y;
            if(canceled){
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                goal_handle->canceled(result);
            } else if(succeeded){
                RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", status.c_str());
                goal_handle->succeed(result);
            } else {
                RCLCPP_WARN(this->get_logger(), "Goal failed: %s", status.c_str());
                goal_handle->abort(result);
            }
        }

        static double normalizeAngle(double a){
            while(a > M_PI)  a -= 2.0 * M_PI;
            while(a < -M_PI) a += 2.0 * M_PI;
            return a;
        }

        rclcpp_action::Server<Navigation>::SharedPtr nav_server_;
        rclcpp_action::Client<NavToPose>::SharedPtr nav2_client_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::mutex map_mutex_;
        nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
