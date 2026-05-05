#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

class Node_1 : public rclcpp ::Node
{
public:
    Node_1() : Node("Node_1")
    {
        subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std ::bind(&Node_1::onOdom, this, std::placeholders ::_1));
        subscriber_imu = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&Node_1::onImu, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/cylinder_pose", 10);
    }

private:
    void onOdom(const nav_msgs::msg::Odometry &msg)
    {
        double dx = msg.pose.pose.position.x - 3.0;
        double dy = msg.pose.pose.position.y - 2.0;

        double Q = atan2(dy, dx);
        double z = msg.pose.pose.position.z;
        curr_z = z;

        curr_theta = 0.9 * old_theta +  0.1 * Q;
        old_theta = curr_theta;




        geometry_msgs::msg::Point out_msg;

        out_msg.y = curr_z;
        out_msg.x = curr_theta;
        out_msg.z = pitch_angle;
        publisher_->publish(out_msg);
    }
    void onImu(const sensor_msgs::msg::Imu &msg)
    {

        double ang_x = msg.angular_velocity.x;
        double ang_y = msg.angular_velocity.y;
        double ang_z = msg.angular_velocity.z;
        double acc_x = msg.linear_acceleration.x;
        double acc_y = msg.linear_acceleration.y;
        double acc_z = msg.linear_acceleration.z;
        double current_time = this->now().seconds();
        if (previous_time == 0)
        {
            previous_time = current_time;
            return;
        }
        double dt = current_time - previous_time;
        previous_time = current_time;

        gyro_pitch = gyro_pitch + ang_x * dt;
        gyro_roll = gyro_roll + ang_y * dt;
        gyro_yaw = gyro_yaw + ang_z * dt;

        double acc_pitch = atan2(acc_y, acc_z);
        double acc_roll = atan2(acc_x, acc_z);

        pitch_angle = 0.98 * (pitch_angle + ang_x*dt) + 0.02 * acc_pitch; // complementry filter

    }

    double old_theta = 0.0;
    double gyro_pitch = 0.0;
    double gyro_roll = 0.0;
    double gyro_yaw = 0.0;

    double pitch_angle = 0.0;
    double previous_time = 0.0;
    double curr_z = 0.0;
    double curr_theta  = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                 // start ROS2
    rclcpp::spin(std::make_shared<Node_1>()); // run node
    rclcpp::shutdown();                       // cleanup
    return 0;
}
