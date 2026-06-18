#include <thread>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/nav.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/float64.hpp"


using namespace std::chrono_literals;

// Navigation server: exposes a simple (target_x, target_y) Nav action and
// forwards every goal to Nav2 (NavigateToPose). Nav2's lidar-fed costmaps and
// allow_unknown let it plan into unexplored space and replan as it goes.
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

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Latched so a late-joining CameraRotator still gets the value.
            // Keep the camera pointing straight ahead (0.0) for now.
            rclcpp::QoS cam_qos(rclcpp::KeepLast(1));
            cam_qos.transient_local().reliable();
            camera_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("camera_angle_target", cam_qos);
            camera_angle_pub_->publish(std_msgs::msg::Float64().set__data(0.0));
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

        void execute(const std::shared_ptr<GoalHandle> goal_handle){
            double tx = goal_handle->get_goal()->target_x;
            double ty = goal_handle->get_goal()->target_y;

            // Forward every goal to Nav2. Its lidar-fed costmaps + allow_unknown
            // let it plan into unexplored space and replan as the scan reveals
            // obstacles.
            RCLCPP_INFO(this->get_logger(), "Navigating to target (%.2f, %.2f) via Nav2.", tx, ty);
            navigateWithNav2(goal_handle, tx, ty);
        }

        // Navigate to the target via Nav2.
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

        rclcpp_action::Server<Navigation>::SharedPtr nav_server_;
        rclcpp_action::Client<NavToPose>::SharedPtr nav2_client_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_angle_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
