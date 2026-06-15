#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <cmath>
#include <algorithm>
#include <memory>


class CameraRotator : public rclcpp::Node
{
    public:
        CameraRotator() : rclcpp::Node("camera_rotator")
        {
            cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rotator_position_controller/commands", 10);

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&CameraRotator::jointStateCallback, this, std::placeholders::_1));

            angle_tar_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                "/camera_angle_target", 10, std::bind(&CameraRotator::angleTargetCallback, this, std::placeholders::_1));
        }

        void angleTargetCallback(const std_msgs::msg::Float64 ::SharedPtr msg)
        {
            float desired_angle = msg->data;
            setAngle(desired_angle);
        }

        void setAngle(float angle)
        {
            if (!have_angle_) return; 

            double diff = std::atan2(std::sin(angle - current_angle_),
                                std::cos(angle - current_angle_));

            double target = current_angle_ + diff;

            std_msgs::msg::Float64MultiArray msg;
            msg.data = {target};
            cmd_pub_->publish(msg);
        }

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), "rotator_joint");
            if (it == msg->name.end()) return;
            size_t idx = std::distance(msg->name.begin(), it);
            if (idx >= msg->position.size()) return;
            current_angle_ = msg->position[idx]; 
            have_angle_ = true;
        }


        private:
        float current_angle_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_tar_sub_;
        bool have_angle_ = false;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraRotator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}