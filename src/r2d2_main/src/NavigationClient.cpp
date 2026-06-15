#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/nav.hpp"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"

using Navigation = action_interfaces::action::Nav;
using GoalHandle = rclcpp_action::ClientGoalHandle<Navigation>;

class NavigationClient : public rclcpp::Node{
    public:
    NavigationClient(): Node("action_client_node"){
        nav_client_ = rclcpp_action::create_client<Navigation>(this, "navigate");

        while(!nav_client_->wait_for_action_server(std::chrono::seconds(1))){
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }

        input_thread_ = std::thread([this](){
            while(true){
                float x, y;
                std::cout << "Enter target X: ";
                std::cin >> x;
                std::cout << "Enter target Y: ";
                std::cin >> y;
                sendGoal(x, y);
            }
        });
        input_thread_.detach();

        //tf2 part:
        nav_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        nav_listener_ = std::make_shared<tf2_ros::TransformListener>(*nav_buffer_, this);
        
    }

    void sendGoal(float x, float y){
        auto options = rclcpp_action::Client<Navigation>::SendGoalOptions();
        options.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const Navigation::Feedback> feedback){
            RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", feedback->current_x, feedback->current_y);
        };
        options.result_callback = [this](const GoalHandle::WrappedResult & result){
            switch(result.code){
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded with new coordinates (%.2f, %.2f)", result.result->final_x, result.result->final_y);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "Unknown result code");
            }
        };


        auto goal = Navigation::Goal();
        goal.target_x = x;
        goal.target_y = y;
        nav_client_->async_send_goal(goal, options);

        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this](){
            try {
                geometry_msgs::msg::TransformStamped t = nav_buffer_->lookupTransform(
                    "map",
                    "robot_base",
                    tf2::TimePointZero
                );
                RCLCPP_INFO(this->get_logger(), "TF2 coordinates: x: %f  y: %f",
                    t.transform.translation.x, t.transform.translation.y);
            }
            catch (const tf2::LookupException & e) {
                RCLCPP_WARN(this->get_logger(), "Transform not available yet: %s", e.what());
            }
        });
    }

    private:
    rclcpp_action::Client<Navigation>::SharedPtr nav_client_;
    std::thread input_thread_;


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> nav_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> nav_listener_;

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}