#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/nav.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"


class NavigationServer : public rclcpp::Node
{
    public:
        using Navigation = action_interfaces::action::Nav;
        using GoalHandle = rclcpp_action::ServerGoalHandle<Nav>;
        NavigationServer() : Node("navigation_server")
        {
            RCLCPP_INFO(this->get_logger(), "Navigation Server has been started.");

            nav_server_ = create_server<Navigation>(
                this, "navigate",
                std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
            );

        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,std::shared_ptr<const Navigation::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal request with target X:%.2f  Y:%.2f", goal->target_x, goal->target_y);
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


    private:


}