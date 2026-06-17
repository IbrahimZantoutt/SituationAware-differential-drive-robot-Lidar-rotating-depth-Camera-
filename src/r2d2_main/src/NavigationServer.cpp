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
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav_msgs/msg/path.hpp"
#include <limits>


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

            // Latched so a late-joining CameraRotator still gets the last target.
            rclcpp::QoS cam_qos(rclcpp::KeepLast(1));
            cam_qos.transient_local().reliable();
            camera_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("camera_angle_target", cam_qos);

            plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10,
            [this](nav_msgs::msg::Path::SharedPtr msg){
                std::lock_guard<std::mutex> lock(plan_mutex_);
                latest_plan_ = msg;
            });

            laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10,
                [this](sensor_msgs::msg::LaserScan::SharedPtr msg){
                    std::lock_guard lk(scan_mtx_); last_scan_ = msg;
                });
           
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
                double cx, cy, cyaw;
                if(getRobotPose(cx, cy, cyaw)){
                    double angle;
                    if(cameraAngleFromPath(cx, cy, cyaw, angle))
                        aimCamera(angle);                                                  // path direction
                    else
                        aimCamera(normalizeAngle(std::atan2(ty - cy, tx - cx) - cyaw));     // fallback: goal
                }
                if(result_future.wait_for(100ms) == std::future_status::ready){
                    break;
                }
            }

            auto nav2_result = result_future.get();
            bool ok = (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED);
            finish(goal_handle, result, ok ? "Goal reached" : "Navigation failed", ok, false);
        }

        // ---- Mode 2: blind drive toward the point, avoiding obstacles (Bug2) ----
        void blindMove(const std::shared_ptr<GoalHandle> goal_handle, double tx, double ty){
            auto result = std::make_shared<Navigation::Result>();

            // go-to-goal gains
            const double goal_tol = 0.15;   // m
            const double yaw_tol  = 0.30;   // rad: turn in place above this
            const double max_lin  = 0.22;   // m/s
            const double max_ang  = 1.0;    // rad/s
            const double k_lin    = 0.5;
            const double k_ang    = 1.5;

            // obstacle / wall-follow params
            const double safe_front   = 0.5;   // m: closer than this in front => obstacle
            const double safe_clear   = 0.7;   // m: front must beat this to leave (hysteresis)
            const double desired_wall = 0.6;   // m: distance to hold from the wall
            const double wall_speed   = 0.15;  // m/s while following
            const double turn_speed   = 0.8;   // rad/s when turning out of a corner
            const double kp_wall      = 1.5;   // P gain on wall-distance error
            const double leave_eps    = 0.10;  // m: must be this much closer than hit_dist
            const double hit_far      = 0.6;   // m: counts as "left the hit point"
            const double hit_near     = 0.3;   // m: returning this close => looped => fail

            // sector arcs (radians, 0 = forward)
            auto front_of = [](const sensor_msgs::msg::LaserScan::SharedPtr & s){ return sectorMin(s, -0.35, 0.35); };
            auto left_of  = [](const sensor_msgs::msg::LaserScan::SharedPtr & s){ return sectorMin(s,  1.22, 1.92); };
            auto right_of = [](const sensor_msgs::msg::LaserScan::SharedPtr & s){ return sectorMin(s, -1.92, -1.22); };

            enum class BugState { GO_TO_GOAL, WALL_FOLLOW };
            enum class FollowSide { LEFT, RIGHT };

            // m-line tail = start point; wait for the first pose to anchor it
            double sx, sy, syaw;
            while(rclcpp::ok() && !getRobotPose(sx, sy, syaw))
                std::this_thread::sleep_for(50ms);

            BugState state      = BugState::GO_TO_GOAL;
            FollowSide side     = FollowSide::LEFT;
            double hit_dist     = 0.0;     // distance-to-goal when we hit the obstacle
            double hit_x = 0.0, hit_y = 0.0;
            bool   left_hit     = false;   // have we moved away from the hit point yet?
            double prev_side_of = sideOfMLine(sx, sy, sx, sy, tx, ty);  // = 0 at start

            geometry_msgs::msg::Twist cmd;

            while(rclcpp::ok()){
                if(goal_handle->is_canceling()){
                    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop
                    camera_angle_pub_->publish(std_msgs::msg::Float64().set__data(0.0));  // look forward
                    finish(goal_handle, result, "canceled", false, true);
                    return;
                }

                sensor_msgs::msg::LaserScan::SharedPtr scan;
                { std::lock_guard<std::mutex> lk(scan_mtx_); scan = last_scan_; }
                if(!scan){
                    std::this_thread::sleep_for(100ms);
                    continue;
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

                double front = front_of(scan);
                double left  = left_of(scan);
                double right = right_of(scan);
                double m_side = sideOfMLine(cx, cy, sx, sy, tx, ty);

                cmd = geometry_msgs::msg::Twist();  // default: stop, fill per state

                if(state == BugState::GO_TO_GOAL){
                    if(front < safe_front){
                        // hit an obstacle -> switch to wall-following
                        state    = BugState::WALL_FOLLOW;
                        hit_dist = dist;
                        hit_x = cx; hit_y = cy;
                        left_hit = false;
                        prev_side_of = m_side;
                        // hug whichever side is more open so we turn into free space
                        side = (left > right) ? FollowSide::RIGHT : FollowSide::LEFT;
                        RCLCPP_INFO(this->get_logger(),
                            "HIT front=%.2f at dist=%.2f -> WALL_FOLLOW (%s)",
                            front, dist, side == FollowSide::LEFT ? "left" : "right");
                    } else {
                        // ---- go straight at the goal (original blind controller) ----
                        double yaw_err = normalizeAngle(std::atan2(dy, dx) - cyaw);
                        aimCamera(yaw_err);
                        cmd.angular.z = std::clamp(k_ang * yaw_err, -max_ang, max_ang);
                        cmd.linear.x  = (std::abs(yaw_err) > yaw_tol) ? 0.0
                                                                     : std::min(k_lin * dist, max_lin);
                    }
                } else {  // WALL_FOLLOW
                    // track whether we've moved off the hit point (for loop detection)
                    double from_hit = std::hypot(cx - hit_x, cy - hit_y);
                    if(from_hit > hit_far) left_hit = true;

                    bool crossed = (prev_side_of * m_side < 0.0);   // sign flip = re-crossed m-line
                    if(crossed && dist < hit_dist - leave_eps && front > safe_clear){
                        // leave point found: head for the goal again
                        state = BugState::GO_TO_GOAL;
                        RCLCPP_INFO(this->get_logger(),
                            "LEAVE: re-crossed m-line at dist=%.2f (hit was %.2f) -> GO_TO_GOAL",
                            dist, hit_dist);
                        prev_side_of = m_side;
                        std::this_thread::sleep_for(50ms);
                        continue;   // next tick drives toward goal
                    }

                    if(left_hit && from_hit < hit_near){
                        // looped the whole obstacle without leaving => unreachable
                        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
                        finish(goal_handle, result, "Goal unreachable (blind)", false, false);
                        return;
                    }

                    // ---- wall-follow control ----
                    // side_sign: +1 hug wall on LEFT, -1 hug wall on RIGHT
                    double side_sign = (side == FollowSide::LEFT) ? 1.0 : -1.0;
                    double measured  = (side == FollowSide::LEFT) ? left : right;
                    aimCamera(0.0);  // look where we're going

                    if(front < safe_front){
                        // inside corner / dead end -> turn in place away from the wall
                        cmd.linear.x  = 0.0;
                        cmd.angular.z = -side_sign * turn_speed;
                    } else {
                        // hold desired_wall on the followed side; +error (too far) steers toward wall
                        double err = measured - desired_wall;
                        cmd.linear.x  = wall_speed;
                        cmd.angular.z = side_sign * std::clamp(kp_wall * err, -max_ang, max_ang);
                    }
                    prev_side_of = m_side;
                }

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

        // Smallest valid range (m) over the arc [lo, hi] in radians (robot frame,
        // 0 = straight ahead). Ignores inf/nan and out-of-range returns. Returns
        // range_max if the arc has no valid reading (i.e. "clear"). Handles arcs
        // that wrap past +/-pi (e.g. the rear sector).
        static double sectorMin(const sensor_msgs::msg::LaserScan::SharedPtr & scan,
                                double lo, double hi){
            const int n = static_cast<int>(scan->ranges.size());
            if(n == 0) return 0.0;

            // angle -> index:  index = round((angle - angle_min) / angle_increment)
            int i_lo = static_cast<int>(std::lround((lo - scan->angle_min) / scan->angle_increment));
            int i_hi = static_cast<int>(std::lround((hi - scan->angle_min) / scan->angle_increment));
            if(i_hi < i_lo) i_hi += n;   // arc wraps past the +/-pi seam

            double best = scan->range_max;
            for(int i = i_lo; i <= i_hi; ++i){
                int idx = ((i % n) + n) % n;   // wrap into [0, n)
                double r = scan->ranges[idx];
                if(std::isfinite(r) && r >= scan->range_min && r <= scan->range_max)
                    best = std::min(best, r);
            }
            return best;
        }

        // Signed position of (cx,cy) relative to the m-line (start -> goal).
        // >0 on one side, <0 on the other, ~0 on the line. The sign flipping
        // between two ticks means the robot crossed the m-line.
        static double sideOfMLine(double cx, double cy,
                                  double sx, double sy, double tx, double ty){
            return (cx - sx) * (ty - sy) - (cy - sy) * (tx - sx);
        }

        void aimCamera(double body_angle){
            if(std::abs(normalizeAngle(body_angle - last_cam_angle_)) < 0.05)  // ~3 deg gate
                return;
            last_cam_angle_ = body_angle;
            camera_angle_pub_->publish(std_msgs::msg::Float64().set__data(body_angle));
        }

        bool cameraAngleFromPath(double cx, double cy, double cyaw, double & out_angle){
            nav_msgs::msg::Path::SharedPtr path;
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                path = latest_plan_;
            }
            if(!path || path->poses.size() < 2) return false;

            const double look_ahead = 0.6;  // m

            // closest path point to the robot
            size_t closest = 0;
            double best = std::numeric_limits<double>::max();
            for(size_t i = 0; i < path->poses.size(); ++i){
                double d = std::hypot(path->poses[i].pose.position.x - cx,
                                    path->poses[i].pose.position.y - cy);
                if(d < best){ best = d; closest = i; }
            }
            // walk forward until ~look_ahead away (or path end)
            size_t idx = closest;
            for(size_t i = closest; i < path->poses.size(); ++i){
                idx = i;
                double d = std::hypot(path->poses[i].pose.position.x - cx,
                                    path->poses[i].pose.position.y - cy);
                if(d >= look_ahead) break;
            }
            out_angle = normalizeAngle(std::atan2(path->poses[idx].pose.position.y - cy,
                                                path->poses[idx].pose.position.x - cx) - cyaw);
            return true;
        }

        rclcpp_action::Server<Navigation>::SharedPtr nav_server_;
        rclcpp_action::Client<NavToPose>::SharedPtr nav2_client_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::mutex map_mutex_;
        nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_angle_pub_;

        double last_cam_angle_ = 100.0;  // last angle sent to the camera; 100 forces the first publish

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
        std::mutex plan_mutex_;
        nav_msgs::msg::Path::SharedPtr latest_plan_;


        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
        sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
        std::mutex scan_mtx_;



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
