#include <iostream>
#include <thread>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/nav.hpp"

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
            float x, y;
            while(rclcpp::ok()){
                std::cout << "Enter target X: ";
                if(!(std::cin >> x)){
                    RCLCPP_WARN(this->get_logger(),
                        "stdin closed/invalid -> stopping goal input loop. "
                        "Run NavigationClient in an interactive terminal.");
                    break;   // EOF / bad input: stop instead of spinning forever
                }
                std::cout << "Enter target Y: ";
                if(!(std::cin >> y)){
                    RCLCPP_WARN(this->get_logger(),
                        "stdin closed/invalid -> stopping goal input loop.");
                    break;
                }
                sendGoal(x, y);
            }
        });
        input_thread_.detach();
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
    }

    private:
    rclcpp_action::Client<Navigation>::SharedPtr nav_client_;
    std::thread input_thread_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}