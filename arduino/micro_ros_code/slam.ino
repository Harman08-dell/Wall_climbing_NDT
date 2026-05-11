// ============================================================
// STAGE 1 — Teleop Only (cmd_vel → motors)
// ESP32 Single Sketch | DRISHTI Ground SLAM
// Motor drivers: MD20A (Left) + RMCS-2305 (Right)
// ============================================================

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/time_sync.h>
#include <Wire.h>

// ── BLOCK 1: Pin Definitions ──────────────────────────────
// Left side  → MD20A
const int DIR_LEFT  = 26;
const int PWM_LEFT  = 25;

// Right side → RMCS-2305
const int DIR_RIGHT = 32;
const int PWM_RIGHT = 33;

// Tuning constants — adjust these during testing
const float DEADBAND  = 0.05f;
const float MAX_SPEED = 0.6f;

#define ENCODER_A  14
#define ENCODER_B   27
#define ENCODER_C  13
#define ENCODER_D   12
#define TICKS_PER_REV 10000.0
#define CIRCUMFERENCE 0.14


#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68 // imu addresss unique home address

volatile long tick_count_x = 0;
volatile long tick_count_y = 0;


// ── DIR POLARITY ──
// If a side goes wrong direction, flip its value (HIGH → LOW or LOW → HIGH)
const int LEFT_FORWARD_DIR  = HIGH;
const int RIGHT_FORWARD_DIR = HIGH;  // RMCS often needs opposite polarity

// ── BLOCK 2: micro-ROS Entities ───────────────────────────
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rcl_publisher_t publisher;   // for odom 
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer; // for odom

rcl_timer_t timer_imu;        //for imu
rcl_publisher_t imu_pub; //for imu

rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transform;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

// ── BLOCK 3: Connection State Machine ─────────────────────
// The ESP32 lives in one of these three states at all times
enum AgentState { WAITING, AGENT_OK, RECONNECTING };
AgentState agent_state = WAITING;

// ── BLOCK 4: Error Handling Macros ────────────────────────
// RCCHECK  → if this fails, return false (used in create_entities)
// RCSOFTCHECK → if this fails, just continue (used in loop)
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ============================================================
// MOTOR HELPER
// speed: -1.0 (full backward) to +1.0 (full forward)
// Positive → DIR HIGH, PWM = speed * 255
// Negative → DIR LOW,  PWM = abs(speed) * 255
// Near zero → stop
// ============================================================

void IRAM_ATTR onEncoderTick_x()
 {
  int a_state = digitalRead(ENCODER_A);
  int b_state = digitalRead(ENCODER_B);

  if(a_state == HIGH) {
    // A is rising
    if(b_state == LOW)  tick_count_x++;
    else                tick_count_x--;
  } else {
    // A is falling
    if(b_state == HIGH) tick_count_x++;
    else                tick_count_x--;
  }
}

void IRAM_ATTR onEncoderTick_y()
 {
  int a_state = digitalRead(ENCODER_C);
  int b_state = digitalRead(ENCODER_D);

  if(a_state == HIGH) {
    // A is rising
    if(b_state == LOW)  tick_count_y++;
    else                tick_count_y--;
  } else {
    // A is falling
    if(b_state == HIGH) tick_count_y++;
    else                tick_count_y--;
  }
}



void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz)
{
  Wire.beginTransmission(MPU_ADDR);  // Hey i want to talk to you 
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // gives me 14 bytes from there 

  ax = (Wire.read() << 8) | Wire.read();   // thsi is a conversion which shifts 8 bytes 
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp (ignore)
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ---------- TIMER CALLBACK ----------
void timer_callback_mpu(rcl_timer_t *, int64_t)
{
  int16_t ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  // Convert to SI units
  imu_msg.linear_acceleration.x = (ax / 16384.0) * 9.81;
  imu_msg.linear_acceleration.y = (ay / 16384.0) * 9.81;
  imu_msg.linear_acceleration.z = (az / 16384.0) * 9.81;

  imu_msg.angular_velocity.x = (gx / 131.0) * DEG_TO_RAD;
  imu_msg.angular_velocity.y = (gy / 131.0) * DEG_TO_RAD;
  imu_msg.angular_velocity.z = (gz / 131.0) * DEG_TO_RAD;

  // Orientation UNKNOWN (set to zero, filter will fill it)
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;

  // Frame ID
  imu_msg.header.frame_id.data = (char*)"imu_link";
  imu_msg.header.frame_id.size = strlen("imu_link");
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  int64_t now_ns = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec = now_ns / 1000000000;
  imu_msg.header.stamp.nanosec = now_ns % 1000000000;


  rcl_publish(&imu_pub, &imu_msg, NULL);
}


void timer_callback_odom(rcl_timer_t * timer, int64_t last_call_time) {
  // Step 1: calculate distance (you already know this formula)

  noInterrupts();
  long snap_x = tick_count_x;
  long snap_y = tick_count_y;
  interrupts();

  int64_t now_ns = rmw_uros_epoch_nanos();
  
  float distance_x_m = (snap_x / 10000.0) * 0.14;
  float distance_y_m = (snap_y / 10000.0) * 0.14;
  
  odom_msg.header.stamp.sec     = now_ns / 1000000000;
odom_msg.header.stamp.nanosec = now_ns % 1000000000;

odom_msg.header.frame_id.data     = (char*)"odom";
odom_msg.header.frame_id.size     = strlen("odom");
odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;

odom_msg.child_frame_id.data     = (char*)"base_link";
odom_msg.child_frame_id.size     = strlen("base_link");
odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;

  odom_msg.pose.pose.position.x = distance_x_m;
  odom_msg.pose.pose.position.y = distance_y_m;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = 0;
  odom_msg.pose.pose.orientation.w = 1;

   // TF header — same timestamp
  transform.header.stamp.sec     = now_ns / 1000000000;
  transform.header.stamp.nanosec = now_ns % 1000000000;

 


  // New for TF2
  transform.header.frame_id.data     = (char*)"odom";
  transform.header.frame_id.size     = strlen("odom");
  transform.header.frame_id.capacity = transform.header.frame_id.size + 1;

  transform.child_frame_id.data     = (char*)"base_link";
  transform.child_frame_id.size     = strlen("base_link");
  transform.child_frame_id.capacity = transform.child_frame_id.size + 1;



  // Postion -- robot moved this far along X axis 
    transform.transform.translation.x = distance_x_m;
    transform.transform.translation.y = distance_y_m;
    transform.transform.translation.z = 0.0;

    // rotation - no rotation 
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_msg.transforms.data    = &transform;
    tf_msg.transforms.size    = 1;
    tf_msg.transforms.capacity = 1;


  // Step 2: publish the message
  rcl_publish(&publisher, &odom_msg, NULL);
  rcl_publish(&tf_publisher, &tf_msg, NULL);

}

void setMotor(int dirPin, int pwmPin, float speed, int forwardDir) {
  if (fabsf(speed) < DEADBAND) {
    analogWrite(pwmPin, 0);
    return;
  }

  speed = constrain(speed, -1.0f, 1.0f);

  // forwardDir tells us which electrical signal means "forward" for this driver
  int dir = (speed > 0) ? forwardDir : (1 - forwardDir);
  digitalWrite(dirPin, dir);
  analogWrite(pwmPin, (int)(fabsf(speed) * 255));
}
// ============================================================
// TWIST CALLBACK
// Fires every time a new /cmd_vel message arrives
// This is the differential drive mixer — no if-else chain
// ============================================================
void twist_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg =
    (const geometry_msgs__msg__Twist*)msgin;

  // Read joystick values
  float lin = msg->linear.x;   // forward/backward: -1.0 to +1.0
  float ang = msg->angular.z;  // rotation: -1.0 to +1.0

  // Apply MAX_SPEED cap
  lin = constrain(lin * MAX_SPEED, -1.0f, 1.0f);
  ang = constrain(ang * MAX_SPEED, -1.0f, 1.0f);

  // ── The Differential Drive Mixer ──
  // Intuition:
  //   Both wheels get the same linear component (go forward together)
  //   Angular component is subtracted from left, added to right
  //   → turning left: right faster than left
  //   → pure rotation: they go in opposite directions automatically
  float left_speed  = lin - ang;
  float right_speed = lin + ang;

  // Clamp after mixing (mixing can push values past ±1.0)
  left_speed  = constrain(left_speed,  -1.0f, 1.0f);
  right_speed = constrain(right_speed, -1.0f, 1.0f);

  // Send to motors
 setMotor(DIR_LEFT,  PWM_LEFT,  left_speed,  LEFT_FORWARD_DIR);
setMotor(DIR_RIGHT, PWM_RIGHT, right_speed, RIGHT_FORWARD_DIR);
}

// ============================================================
// CREATE / DESTROY micro-ROS ENTITIES
// Called by state machine — not directly by you
// ============================================================
bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "drishti_motor_driver", "", &support));

  // Sync ESP32 clock with RPi — MUST be before any publishing
RCCHECK(rmw_uros_sync_session(1000));

    //publisher
    RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg,Odometry ),
    "/odom"));

    RCCHECK(rclc_publisher_init_default(
    &tf_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg,TFMessage),
    "/tf"));

    RCCHECK(rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg,Imu),
    "/imu/data_raw"));

  // Subscribe to /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));



  // Executor handles 1 entity (the subscriber)
  // We'll increase this number when we add IMU + odom in Stage 2
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback_odom));
    RCCHECK(rclc_timer_init_default(&timer_imu, &support, RCL_MS_TO_NS(20), timer_callback_mpu));

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

   
    RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_imu));
    
    return true;
}

void destroy_entities() {
  // Clean shutdown — important so reconnection works properly
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
  RCSOFTCHECK(rcl_publisher_fini(&tf_publisher, &node));
   RCSOFTCHECK(rcl_publisher_fini(&imu_pub, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rcl_timer_fini(&timer_imu));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rclc_support_fini(&support));
                

}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Configure motor pins
  pinMode(DIR_LEFT,  OUTPUT);
  pinMode(PWM_LEFT,  OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  // Start with motors stopped — safety
  analogWrite(PWM_LEFT,  0);
  analogWrite(PWM_RIGHT, 0);

  // Encoder setup (same as before)
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

   pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), onEncoderTick_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), onEncoderTick_y, CHANGE);

Wire.begin(SDA_PIN, SCL_PIN);
  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);



  


  // Configure micro-ROS transport (Serial USB to RPi)
  set_microros_transports();

  agent_state = WAITING;
}

// ============================================================
// LOOP — Connection State Machine
// ============================================================
void loop() {
  switch (agent_state) {

    case WAITING:
      // Ping the agent every 500ms
      // ping_agent(timeout_ms, attempts) → true if agent responds
      if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK) {
        if (create_entities()) {
          Serial.println("[DRISHTI] Agent connected. Motors ready.");
          agent_state = AGENT_OK;
        } else {
          destroy_entities();
          Serial.println("[DRISHTI] Entity creation failed. Retrying...");
        }
      } else {
        Serial.println("[DRISHTI] Waiting for micro-ROS agent...");
        delay(500);
      }
      break;

    case AGENT_OK:
      // Normal operation — spin executor at 10ms
      // This is why your old code had lag: you used 100ms here
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
      } else {
        // Agent disappeared — go to reconnecting
        Serial.println("[DRISHTI] Agent lost. Reconnecting...");
        agent_state = RECONNECTING;
      }
      break;

    case RECONNECTING:
      // Stop motors immediately for safety
      analogWrite(PWM_LEFT,  0);
      analogWrite(PWM_RIGHT, 0);
      destroy_entities();
      agent_state = WAITING;
      break;
  }
}
