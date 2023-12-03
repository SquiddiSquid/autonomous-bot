#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

const double HERTZ = 100;
const double FREQUENCY = HERTZ / 1000;

struct RaidoBotParams
{
    double base_length = 0.5;
    double base_width = 0.35;
    double base_height = 0.11;

    double wheel_radius = 0.06;
    double wheel_thickness = 0.05;

    std::vector<std::string> joint_names = {
        "wheel_link_fr_joint",
        "wheel_link_fl_joint",
        "wheel_link_br_joint",
        "wheel_link_bl_joint"};
};

class RaidoDrive : public rclcpp::Node
{
public:
    RaidoDrive() : Node("raido_drive_node")
    {
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_combined", 10);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&RaidoDrive::get_velocity, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void get_velocity(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;  // Forward velocity
        double w = msg->angular.z; // Angular velocity

        // Velocities of the wheels
        double vl = v - (w * params_.base_width / 2);
        double vr = v + (w * params_.base_width / 2);

        // Rotation angles of the wheels
        double wl = vl * FREQUENCY / params_.wheel_radius;
        double wr = vr * FREQUENCY / params_.wheel_radius;

        // Orientation difference robot (along z axis)
        double delta_theta = ((vr - vl) / params_.base_width);

        // Addition of the rotation angles
        rotation_l_ += wl;
        rotation_r_ += wr;

        // Calc new robot position in room
        double x_pos = (x_) + (v * 0.1 * std::cos(theta_ + (delta_theta / 2) * FREQUENCY));
        double y_pos = (y_) + (v * 0.1 * std::sin(theta_ + (delta_theta / 2) * FREQUENCY));
        double theta = theta_ + delta_theta * FREQUENCY;

        publish_odometry(x_pos, y_pos, theta, v, w); // Publish new robot position

        publish_rotations(rotation_r_, rotation_l_); // Publish new joint rotation of the wheels

        // Save current position of the robot
        x_ = x_pos;
        y_ = y_pos;
        theta_ = theta;
    }

    void publish_odometry(double x, double y, double theta, double vel, double omega)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "odom_combined";
        odom.child_frame_id = "base_link";
        odom.header.stamp = now();

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion rot;
        rot.setRPY(0, 0, theta);

        odom.pose.pose.orientation.x = rot.x();
        odom.pose.pose.orientation.y = rot.y();
        odom.pose.pose.orientation.z = rot.z();
        odom.pose.pose.orientation.w = rot.w();

        odom.twist.twist.linear.x = vel;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = omega;

        odometry_publisher_->publish(odom);

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.frame_id = "odom_combined";
        odom_trans.child_frame_id = "base_link";

        odom_trans.header.stamp = now();
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        rot.setRPY(0, 0, theta);

        odom_trans.transform.rotation.x = rot.x();
        odom_trans.transform.rotation.y = rot.y();
        odom_trans.transform.rotation.z = rot.z();
        odom_trans.transform.rotation.w = rot.w();

        tf_broadcaster_->sendTransform(odom_trans);
    }

    void publish_rotations(double rotation_r, double rotation_l)
    {
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name = params_.joint_names;
        joint_state.position[0] = joint_state.position[1] = rotation_l;
        joint_state.position[2] = joint_state.position[3] = rotation_r;

        joint_state_publisher_->publish(joint_state);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_;
    double y_;
    double theta_;
    double rotation_l_;
    double rotation_r_;

    RaidoBotParams params_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RaidoDrive>());
    rclcpp::shutdown();
    return 0;
}
