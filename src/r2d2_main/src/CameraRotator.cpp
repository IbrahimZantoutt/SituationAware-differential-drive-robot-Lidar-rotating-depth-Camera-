#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <cmath>
#include <algorithm>
#include <memory>


class CameraRotator : public rclcpp::Node
{
    public:
        CameraRotator() : rclcpp::Node("camera_rotator")
        {
            // Commands the rotator joint via the gazebo_ros_joint_pose_trajectory
            // plugin (default topic: /set_joint_trajectory).
            cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/set_joint_trajectory", 10);

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&CameraRotator::jointStateCallback, this, std::placeholders::_1));

            // Match the latched (transient_local) QoS the server publishes with,
            // otherwise the subscription won't connect to it.
            rclcpp::QoS tar_qos(rclcpp::KeepLast(1));
            tar_qos.transient_local().reliable();
            angle_tar_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                "/camera_angle_target", tar_qos, std::bind(&CameraRotator::angleTargetCallback, this, std::placeholders::_1));
        }

        void angleTargetCallback(const std_msgs::msg::Float64 ::SharedPtr msg)
        {
            float desired_angle = msg->data;
            setAngle(desired_angle);
        }

        void setAngle(float angle)
        {
            if (!have_angle_) return; 

            RCLCPP_INFO(this->get_logger(), "Rotating camera to %.2f radians", angle);

            double diff = std::atan2(std::sin(angle - current_angle_),
                                std::cos(angle - current_angle_));

            double target = current_angle_ + diff;

            // Single-point trajectory commanding the rotator joint.
            trajectory_msgs::msg::JointTrajectory traj;
            // Leave header.stamp at 0 (do NOT use this->now()): the plugin
            // schedules points against SIM time, but this node runs on wall time,
            // so a wall-clock stamp would park the point ~1.7e9 s into the sim
            // future and it would never execute. stamp 0 => apply immediately.
            // The plugin also needs a reference frame; "world" is required (an
            // empty frame_id makes it silently do nothing).
            traj.header.frame_id = "world";
            traj.joint_names = {"rotator_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {target};
            traj.points = {point};

            cmd_pub_->publish(traj);
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
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
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