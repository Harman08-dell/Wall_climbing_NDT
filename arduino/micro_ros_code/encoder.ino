#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#define ENCODER_A 22
#define ENCODER_B  21
#define TICKS_PER_REV 10000.0
#define CIRCUMFERENCE 0.14

volatile long tick_count = 0;

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transform;

// ── ISR (you already wrote this!) ──
void IRAM_ATTR onEncoderTick()
 {
  int a_state = digitalRead(ENCODER_A);
  int b_state = digitalRead(ENCODER_B);

  if(a_state == HIGH) {
    // A is rising
    if(b_state == LOW)  tick_count++;
    else                tick_count--;
  } else {
    // A is falling
    if(b_state == HIGH) tick_count++;
    else                tick_count--;
  }
}

// ── Timer Callback ──
// This fires every 100ms and publishes distance
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  // Step 1: calculate distance (you already know this formula)
  
  float distance_m = (tick_count / 10000.0) *  0.14;
  msg.data = distance_m;
  // New for TF2
  transform.header.frame_id.data = "odom";
  transform.child_frame_id.data = "base_link";

  // Postion -- robot moved this far along X axis 
  transform.transform.translation.x = distance_m;
   transform.transform.translation.y = 0;
    transform.transform.translation.z = 0;

    // rotation - no rotation 
    transform.transform.rotation.x = 0;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = 0;
    transform.transform.rotation.w = 1;

    tf_msg.transforms.data    = &transform;
  tf_msg.transforms.size    = 1;
  tf_msg.transforms.capacity = 1;


  // Step 2: publish the message
  rcl_publish(&publisher, &msg, NULL);
  rcl_publish(&tf_publisher, &tf_msg, NULL);

}

void setup() {
  // Encoder setup (same as before)
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), onEncoderTick, CHANGE);

  // micro-ROS transport (Serial USB)
  set_microros_transports();
  delay(100);  // wait for agent to connect

  // micro-ROS init chain (boilerplate — just understand the order)
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node — fill in your node name
  rclc_node_init_default(&node, "dead_wheel_node", "", &support);

  // Create publisher — fill in your topic name
  rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/dead_wheel_distance"
  );
  // ── NEW: TF publisher ──
  rclc_publisher_init_default(
    &tf_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"
  );

  // Create timer — fires every 100ms (100,000,000 nanoseconds)
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

  // Add timer to executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  // This replaces your delay loop
  // It checks: is it time to fire the timer? if yes → calls timer_callback
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}