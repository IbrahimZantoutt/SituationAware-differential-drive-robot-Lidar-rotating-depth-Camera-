#include <thread>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>
#include <atomic>
#include <limits>
#include <utility>
#include <algorithm>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/nav.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;

// ===========================================================================
// Frontier search (port of m-explore-ros2 explore_lite frontier_search.cpp).
//
// A frontier is a cluster of UNKNOWN cells that border FREE space: driving to
// one is guaranteed to reveal new map. We BFS over free space from the robot,
// and whenever we touch an unknown-bordering-free cell we grow the whole
// frontier cluster (8-connected) and record its size, centroid and the closest
// point to the robot. Frontiers are then scored and returned sorted by cost.
//
// Adapted from m-explore's costmap_2d to a plain nav_msgs/OccupancyGrid:
//   value  < 0  -> unknown (NO_INFORMATION)
//   value == 0  -> free space (FREE_SPACE)
//   value  > 0  -> cost / lethal (inflation .. obstacle)
// ===========================================================================

struct Frontier {
    std::uint32_t size = 1;
    double min_distance = std::numeric_limits<double>::infinity();
    double cost = 0.0;
    geometry_msgs::msg::Point initial;   // first contact cell
    geometry_msgs::msg::Point centroid;  // average of frontier cells (goal point)
    geometry_msgs::msg::Point middle;    // frontier cell closest to the robot
    std::vector<geometry_msgs::msg::Point> points;
};

class FrontierSearch {
    public:
        FrontierSearch(const nav_msgs::msg::OccupancyGrid & grid,
                       double potential_scale, double gain_scale,
                       double min_frontier_size, int occupied_threshold)
            : data_(grid.data),
              size_x_(grid.info.width),
              size_y_(grid.info.height),
              resolution_(grid.info.resolution),
              origin_x_(grid.info.origin.position.x),
              origin_y_(grid.info.origin.position.y),
              potential_scale_(potential_scale),
              gain_scale_(gain_scale),
              min_frontier_size_(min_frontier_size),
              occupied_threshold_(occupied_threshold) {}

        // Search for frontiers reachable from `position`, sorted by ascending cost.
        std::vector<Frontier> searchFrom(const geometry_msgs::msg::Point & position){
            std::vector<Frontier> frontier_list;
            if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0) return frontier_list;

            unsigned int mx, my;
            if (!worldToMap(position.x, position.y, mx, my)) return frontier_list;

            const size_t n = static_cast<size_t>(size_x_) * size_y_;
            std::vector<bool> frontier_flag(n, false);
            std::vector<bool> visited_flag(n, false);

            std::queue<unsigned int> bfs;
            unsigned int pos = getIndex(mx, my);

            // Start the search at the nearest free cell (the robot may sit on an
            // inflated / unknown cell).
            unsigned int clear;
            if (nearestFreeCell(clear, pos)) {
                bfs.push(clear);
            } else {
                bfs.push(pos);
            }
            visited_flag[bfs.front()] = true;

            while (!bfs.empty()){
                unsigned int idx = bfs.front();
                bfs.pop();

                // Iterate over the 4-connected neighbourhood.
                for (unsigned int nbr : nhood4(idx)){
                    // Add navigable, unvisited cells to the queue. We traverse any
                    // known cell below the occupied threshold (not just value 0), so
                    // the search can cross inflated corridors/doorways and still find
                    // unexplored pockets behind them.
                    if (isNavigable(nbr) && !visited_flag[nbr]){
                        visited_flag[nbr] = true;
                        bfs.push(nbr);
                    } else if (isNewFrontierCell(nbr, frontier_flag)){
                        frontier_flag[nbr] = true;
                        Frontier nf = buildNewFrontier(nbr, pos, frontier_flag);
                        if (nf.size * resolution_ >= min_frontier_size_){
                            frontier_list.push_back(nf);
                        }
                    }
                }
            }

            for (auto & f : frontier_list) f.cost = frontierCost(f);
            std::sort(frontier_list.begin(), frontier_list.end(),
                      [](const Frontier & a, const Frontier & b){ return a.cost < b.cost; });
            return frontier_list;
        }

    private:
        // Grow a frontier cluster from `initial_cell` using an 8-connected BFS.
        Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                  std::vector<bool> & frontier_flag){
            Frontier output;
            output.centroid.x = 0.0;
            output.centroid.y = 0.0;
            output.size = 1;
            output.min_distance = std::numeric_limits<double>::infinity();

            unsigned int ix, iy;
            indexToCells(initial_cell, ix, iy);
            mapToWorld(ix, iy, output.initial.x, output.initial.y);

            std::queue<unsigned int> bfs;
            bfs.push(initial_cell);

            unsigned int rx, ry;
            double ref_x, ref_y;
            indexToCells(reference, rx, ry);
            mapToWorld(rx, ry, ref_x, ref_y);

            while (!bfs.empty()){
                unsigned int idx = bfs.front();
                bfs.pop();

                for (unsigned int nbr : nhood8(idx)){
                    if (isNewFrontierCell(nbr, frontier_flag)){
                        frontier_flag[nbr] = true;

                        unsigned int mx, my;
                        double wx, wy;
                        indexToCells(nbr, mx, my);
                        mapToWorld(mx, my, wx, wy);

                        geometry_msgs::msg::Point point;
                        point.x = wx;
                        point.y = wy;
                        output.points.push_back(point);

                        output.size++;
                        output.centroid.x += wx;
                        output.centroid.y += wy;

                        double distance = std::hypot(ref_x - wx, ref_y - wy);
                        if (distance < output.min_distance){
                            output.min_distance = distance;
                            output.middle.x = wx;
                            output.middle.y = wy;
                        }

                        bfs.push(nbr);
                    }
                }
            }

            output.centroid.x /= output.size;
            output.centroid.y /= output.size;
            return output;
        }

        // A frontier cell is unknown, not yet claimed, and borders navigable space.
        bool isNewFrontierCell(unsigned int idx, const std::vector<bool> & frontier_flag) const {
            if (data_[idx] >= 0 || frontier_flag[idx]) return false;  // must be unknown
            for (unsigned int nbr : nhood4(idx)){
                if (isNavigable(nbr)) return true;                    // borders navigable space
            }
            return false;
        }

        // A known cell the robot could plausibly travel through (below the occupied
        // threshold). Unknown (<0) and lethal/inflated-above-threshold are excluded.
        bool isNavigable(unsigned int idx) const {
            return data_[idx] >= 0 && data_[idx] < occupied_threshold_;
        }

        // cost = potential_scale * distance - gain_scale * size  (lower is better).
        double frontierCost(const Frontier & f) const {
            return (potential_scale_ * f.min_distance * resolution_) -
                   (gain_scale_ * f.size * resolution_);
        }

        // BFS outward from `start` for the nearest navigable cell.
        bool nearestFreeCell(unsigned int & result, unsigned int start) const {
            if (isNavigable(start)){ result = start; return true; }
            const size_t n = static_cast<size_t>(size_x_) * size_y_;
            std::vector<bool> visited(n, false);
            std::queue<unsigned int> bfs;
            bfs.push(start);
            visited[start] = true;
            while (!bfs.empty()){
                unsigned int idx = bfs.front();
                bfs.pop();
                if (isNavigable(idx)){ result = idx; return true; }
                for (unsigned int nbr : nhood8(idx)){
                    if (!visited[nbr]){ visited[nbr] = true; bfs.push(nbr); }
                }
            }
            return false;
        }

        // ---- index / coordinate helpers -----------------------------------
        unsigned int getIndex(unsigned int mx, unsigned int my) const { return my * size_x_ + mx; }
        void indexToCells(unsigned int idx, unsigned int & mx, unsigned int & my) const {
            my = idx / size_x_;
            mx = idx % size_x_;
        }
        bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const {
            if (wx < origin_x_ || wy < origin_y_) return false;
            int ix = static_cast<int>((wx - origin_x_) / resolution_);
            int iy = static_cast<int>((wy - origin_y_) / resolution_);
            if (ix < 0 || iy < 0 || ix >= static_cast<int>(size_x_) || iy >= static_cast<int>(size_y_))
                return false;
            mx = static_cast<unsigned int>(ix);
            my = static_cast<unsigned int>(iy);
            return true;
        }
        void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const {
            wx = origin_x_ + (mx + 0.5) * resolution_;
            wy = origin_y_ + (my + 0.5) * resolution_;
        }
        std::vector<unsigned int> nhood4(unsigned int idx) const {
            std::vector<unsigned int> out;
            unsigned int x = idx % size_x_, y = idx / size_x_;
            if (x > 0)            out.push_back(idx - 1);
            if (x < size_x_ - 1)  out.push_back(idx + 1);
            if (y > 0)            out.push_back(idx - size_x_);
            if (y < size_y_ - 1)  out.push_back(idx + size_x_);
            return out;
        }
        std::vector<unsigned int> nhood8(unsigned int idx) const {
            std::vector<unsigned int> out = nhood4(idx);
            unsigned int x = idx % size_x_, y = idx / size_x_;
            if (x > 0 && y > 0)                     out.push_back(idx - 1 - size_x_);
            if (x < size_x_ - 1 && y > 0)           out.push_back(idx + 1 - size_x_);
            if (x > 0 && y < size_y_ - 1)           out.push_back(idx - 1 + size_x_);
            if (x < size_x_ - 1 && y < size_y_ - 1) out.push_back(idx + 1 + size_x_);
            return out;
        }

        const std::vector<int8_t> & data_;
        unsigned int size_x_;
        unsigned int size_y_;
        double resolution_;
        double origin_x_;
        double origin_y_;
        double potential_scale_;
        double gain_scale_;
        double min_frontier_size_;
        int occupied_threshold_;
};

// ===========================================================================
// Navigation server: exposes a simple (target_x, target_y) Nav action and
// forwards every goal to Nav2 (NavigateToPose). Nav2's lidar-fed costmaps and
// allow_unknown let it plan into unexplored space and replan as it goes.
//
// It also runs an explore_lite-style autonomous exploration loop: a timer
// repeatedly searches for frontiers, drives to the lowest-cost reachable one,
// and blacklists frontiers it can't make progress toward.
// ===========================================================================
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

            // explore_lite-style parameters (same names/defaults as m-explore).
            potential_scale_   = this->declare_parameter("potential_scale", 3.0);
            gain_scale_        = this->declare_parameter("gain_scale", 1.0);
            min_frontier_size_ = this->declare_parameter("min_frontier_size", 0.5);
            planner_frequency_ = this->declare_parameter("planner_frequency", 0.33);
            progress_timeout_  = this->declare_parameter("progress_timeout", 30.0);
            // Costmap cells at/above this cost (0..100) are treated as blocked by the
            // frontier search. Below it (incl. inflation) is traversable, so the
            // search can reach unexplored space through inflated corridors.
            occupied_threshold_ = static_cast<int>(
                this->declare_parameter("frontier_occupied_threshold", 90));

            nav_server_ = rclcpp_action::create_server<Navigation>(
                this, "navigate",
                std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
            );

            nav2_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Use Nav2's global costmap (not the raw SLAM map) for exploration: its
            // inflation gives frontier targets clearance from walls, and walls block
            // connectivity so we never try to reach unknown space sealed behind them.
            // Requires global_costmap track_unknown_space: true (so unknown stays -1).
            // Published with transient-local durability.
            rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
            costmap_qos.transient_local().reliable();
            costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap/costmap", costmap_qos,
                [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){
                    std::lock_guard<std::mutex> lock(costmap_mutex_);
                    costmap_ = msg;
                });

            // Explore-everything toggle. Publish true to start autonomously
            // exploring all reachable unknown space, false to stop:
            //   ros2 topic pub /explore_all std_msgs/msg/Bool "{data: true}"
            explore_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "explore_all", 10,
                std::bind(&NavigationServer::onExploreToggle, this, std::placeholders::_1));

            // Frontier visualization for RViz (explore_lite calls this explore/frontiers).
            frontier_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "explore/frontiers", rclcpp::QoS(10));

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

        nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap(){
            std::lock_guard<std::mutex> lock(costmap_mutex_);
            return costmap_;
        }

        // ---- explore_lite-style autonomous exploration --------------------

        // Start/stop the exploration timer (mirrors explore_lite's resume topic).
        void onExploreToggle(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data){
                if (explore_on_.exchange(true)) return;  // already running
                RCLCPP_INFO(this->get_logger(), "Explore-everything mode requested.");

                if (!nav2_client_->wait_for_action_server(5s)){
                    RCLCPP_ERROR(this->get_logger(), "Nav2 not available; cannot explore.");
                    explore_on_.store(false);
                    return;
                }

                frontier_blacklist_.clear();
                goal_active_ = false;
                last_progress_ = this->now();

                auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / planner_frequency_));
                explore_timer_ = this->create_wall_timer(
                    period, std::bind(&NavigationServer::makePlan, this));
                RCLCPP_INFO(this->get_logger(), "Explore-everything mode started.");
            } else {
                if (!explore_on_.exchange(false)) return;
                stopExploration();
                RCLCPP_INFO(this->get_logger(), "Explore-everything mode stopped.");
            }
        }

        void stopExploration(){
            explore_on_.store(false);
            if (explore_timer_){ explore_timer_->cancel(); explore_timer_.reset(); }
            nav2_client_->async_cancel_all_goals();
        }

        // Has this goal (or one very close to it) already been blacklisted?
        // Tolerance is 5 costmap cells, exactly like explore_lite.
        bool goalOnBlacklist(const geometry_msgs::msg::Point & goal) const {
            constexpr double tolerance_cells = 5.0;
            const double tol = tolerance_cells * last_resolution_;
            for (const auto & g : frontier_blacklist_){
                if (std::fabs(goal.x - g.x) < tol && std::fabs(goal.y - g.y) < tol) return true;
            }
            return false;
        }

        // Core loop, fired by the timer at planner_frequency. Find frontiers,
        // pick the lowest-cost reachable one, drive to it, and blacklist goals
        // we stop making progress toward. (Port of explore_lite makePlan.)
        void makePlan(){
            if (!explore_on_.load()) return;

            double rx, ry, ryaw;
            auto costmap = getCostmap();
            if (!costmap || !getRobotPose(rx, ry, ryaw)) return;
            last_resolution_ = costmap->info.resolution;

            geometry_msgs::msg::Point pose;
            pose.x = rx;
            pose.y = ry;

            FrontierSearch search(*costmap, potential_scale_, gain_scale_,
                                  min_frontier_size_, occupied_threshold_);
            auto frontiers = search.searchFrom(pose);

            if (frontiers.empty()){
                RCLCPP_INFO(this->get_logger(),
                            "Exploration complete: no frontiers left.");
                stopExploration();
                return;
            }

            publishFrontiers(frontiers, costmap->header);

            // If we're already driving to a frontier, keep driving to it. Re-picking
            // (and preempting) the goal every cycle makes the robot thrash between
            // near-equal frontiers, and each Nav2 preemption-abort would wrongly
            // blacklist a good frontier -- which is what ends exploration early. We
            // only check that we're still making progress.
            if (goal_active_){
                // Progress = the robot is actually moving. Measuring straight-line
                // distance to the goal would falsely flag "stuck" whenever the path
                // winds around walls (distance to goal barely shrinks), so we instead
                // reset the timer whenever the robot moves a meaningful amount.
                if (std::hypot(rx - last_moved_x_, ry - last_moved_y_) > kProgressMoveDist){
                    last_moved_x_ = rx;
                    last_moved_y_ = ry;
                    last_progress_ = this->now();
                }
                if ((this->now() - last_progress_) >
                    rclcpp::Duration::from_seconds(progress_timeout_)){
                    RCLCPP_WARN(this->get_logger(),
                                "Robot stuck en route to (%.2f, %.2f); blacklisting it.",
                                active_goal_.x, active_goal_.y);
                    frontier_blacklist_.push_back(active_goal_);
                    goal_active_ = false;                  // CANCELED callback replans
                    nav2_client_->async_cancel_all_goals();
                }
                return;
            }

            // No active goal: pick the lowest-cost frontier that isn't blacklisted.
            auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                [this](const Frontier & f){ return goalOnBlacklist(f.centroid); });
            if (frontier == frontiers.end()){
                RCLCPP_INFO(this->get_logger(),
                            "Exploration complete: all frontiers blacklisted.");
                stopExploration();
                return;
            }

            sendExploreGoal(frontier->centroid, costmap->header.frame_id, rx, ry);
        }

        // Send one exploration goal to Nav2 and start tracking progress toward it.
        void sendExploreGoal(const geometry_msgs::msg::Point & target,
                             const std::string & frame, double rx, double ry){
            RCLCPP_INFO(this->get_logger(), "Exploring frontier at (%.2f, %.2f).",
                        target.x, target.y);

            goal_active_ = true;
            active_goal_ = target;
            last_moved_x_ = rx;
            last_moved_y_ = ry;
            last_progress_ = this->now();

            NavToPose::Goal nav2_goal;
            nav2_goal.pose.header.frame_id = frame;
            nav2_goal.pose.header.stamp = this->get_clock()->now();
            nav2_goal.pose.pose.position = target;
            nav2_goal.pose.pose.orientation.w = 1.0;

            auto opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();
            opts.result_callback =
                [this, target](const NavToPoseGoalHandle::WrappedResult & result){
                    reachedGoal(result, target);
                };
            nav2_client_->async_send_goal(nav2_goal, opts);
        }

        // Result callback for an exploration goal (port of explore_lite reachedGoal).
        void reachedGoal(const NavToPoseGoalHandle::WrappedResult & result,
                         const geometry_msgs::msg::Point & frontier_goal){
            goal_active_ = false;
            switch (result.code){
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Reached frontier (%.2f, %.2f).",
                                frontier_goal.x, frontier_goal.y);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    // Nav2 genuinely gave up on this goal (e.g. controller couldn't
                    // make progress). Since we never preempt our own goals anymore,
                    // an abort means it's really hard to reach -> blacklist it.
                    RCLCPP_WARN(this->get_logger(),
                                "Nav2 aborted (%.2f, %.2f); blacklisting it.",
                                frontier_goal.x, frontier_goal.y);
                    frontier_blacklist_.push_back(frontier_goal);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    // We canceled it (progress timeout, or exploration stopped).
                    break;
                default:
                    break;
            }
            if (!explore_on_.load()) return;
            // Goal reached: replan immediately instead of waiting for the timer.
            // Run it from a one-shot timer to avoid replanning inside this callback.
            oneshot_timer_ = this->create_wall_timer(std::chrono::nanoseconds(0), [this](){
                oneshot_timer_->cancel();
                makePlan();
            });
        }

        // Publish frontiers for RViz: blue points for the cells, green spheres for
        // the (reachable) centroids we may target.
        void publishFrontiers(const std::vector<Frontier> & frontiers,
                              const std_msgs::msg::Header & header){
            visualization_msgs::msg::MarkerArray markers;

            visualization_msgs::msg::Marker clear;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            markers.markers.push_back(clear);

            int id = 0;
            for (const auto & f : frontiers){
                visualization_msgs::msg::Marker pts;
                pts.header = header;
                pts.ns = "frontier_points";
                pts.id = id++;
                pts.type = visualization_msgs::msg::Marker::POINTS;
                pts.action = visualization_msgs::msg::Marker::ADD;
                pts.scale.x = pts.scale.y = std::max(last_resolution_, 0.03);
                pts.color.b = 1.0;
                pts.color.a = 1.0;
                pts.points = f.points;
                markers.markers.push_back(pts);

                visualization_msgs::msg::Marker centroid;
                centroid.header = header;
                centroid.ns = "frontier_centroids";
                centroid.id = id++;
                centroid.type = visualization_msgs::msg::Marker::SPHERE;
                centroid.action = visualization_msgs::msg::Marker::ADD;
                centroid.pose.position = f.centroid;
                centroid.pose.orientation.w = 1.0;
                centroid.scale.x = centroid.scale.y = centroid.scale.z = 0.2;
                bool blacklisted = goalOnBlacklist(f.centroid);
                centroid.color.r = blacklisted ? 1.0 : 0.0;
                centroid.color.g = blacklisted ? 0.0 : 1.0;
                centroid.color.a = 1.0;
                markers.markers.push_back(centroid);
            }
            frontier_pub_->publish(markers);
        }

        // -------------------------------------------------------------------

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
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
        std::mutex costmap_mutex_;

        // Exploration state.
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;
        rclcpp::TimerBase::SharedPtr explore_timer_;
        rclcpp::TimerBase::SharedPtr oneshot_timer_;
        std::atomic<bool> explore_on_{false};
        std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
        bool goal_active_ = false;                  // are we currently driving to a frontier?
        geometry_msgs::msg::Point active_goal_;     // the frontier we're driving to
        double last_moved_x_ = 0.0;                 // robot pose when we last saw motion
        double last_moved_y_ = 0.0;
        static constexpr double kProgressMoveDist = 0.10;  // m of motion that counts as progress
        rclcpp::Time last_progress_;
        double last_resolution_ = 0.05;

        // Exploration parameters (explore_lite names).
        double potential_scale_;
        double gain_scale_;
        double min_frontier_size_;
        double planner_frequency_;
        double progress_timeout_;
        int occupied_threshold_;

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
