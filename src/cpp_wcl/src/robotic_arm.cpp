// src/arm_teleop_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class ArmTeleop : public rclcpp::Node
{
public:
  ArmTeleop() : Node("arm_teleop_node")
  {
    // ── Publisher ─────────────────────────────────────────────
    // Publishes servo angles to ESP32 via micro-ROS
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/arm_angles", 10);

    // ── Subscriber ────────────────────────────────────────────
    // Listens to PS4 controller data from joy driver
    subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&ArmTeleop::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Arm Teleop Node Started!");
    RCLCPP_INFO(this->get_logger(), "Left stick vertical  → Shoulder");
    RCLCPP_INFO(this->get_logger(), "Right stick vertical → Elbow");
  }

private:

  // ── Converts joystick axis value to servo angle ────────────
  // joystick gives -1.0 to +1.0
  // servo needs  0 to 180
  // formula: angle = (axis + 1.0) / 2.0 * 180
  int axis_to_angle(float axis_val)
  {
    int angle = static_cast<int>((axis_val + 1.0f) / 2.0f * 180.0f);
    // clamp to safe range
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    return angle;
  }

  // ── Callback: runs every time PS4 publishes ────────────────
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Safety: make sure enough axes exist
    if (msg->axes.size() < 5) {
      RCLCPP_WARN(this->get_logger(), "Not enough axes in Joy message!");
      return;
    }

    // Read axes
    // Axis 1 = Left stick vertical  → shoulder
    // Axis 4 = Right stick vertical → elbow
    // NOTE: PS4 vertical axes are inverted (push up = -1.0)
    //       so we negate them
    float shoulder_raw = -(msg->axes[1]);
    float elbow_raw    = -(msg->axes[2]);

    int shoulder_angle = axis_to_angle(shoulder_raw);
    int elbow_angle    = axis_to_angle(elbow_raw);

    // Build the message
    std_msgs::msg::Int32MultiArray out_msg;
    out_msg.data = {shoulder_angle, elbow_angle};

    // Publish
    publisher_->publish(out_msg);

    RCLCPP_INFO(this->get_logger(),
      "Shoulder: %d deg | Elbow: %d deg",
      shoulder_angle, elbow_angle);
  }

  // ── Member variables ───────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
};

// ── Main ──────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTeleop>());
  rclcpp::shutdown();
  return 0;
}