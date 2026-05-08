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

// ── BLOCK 1: Pin Definitions ──────────────────────────────
// Left side  → MD20A
const int DIR_LEFT  = 12;
const int PWM_LEFT  = 13;

// Right side → RMCS-2305
const int DIR_RIGHT = 27;
const int PWM_RIGHT = 14;

// Tuning constants — adjust these during testing
const float DEADBAND  = 0.05f;
const float MAX_SPEED = 0.6f;

// ── DIR POLARITY ──
// If a side goes wrong direction, flip its value (HIGH → LOW or LOW → HIGH)
const int LEFT_FORWARD_DIR  = HIGH;
const int RIGHT_FORWARD_DIR = LOW;  // RMCS often needs opposite polarity

// ── BLOCK 2: micro-ROS Entities ───────────────────────────
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

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

  // Subscribe to /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Executor handles 1 entity (the subscriber)
  // We'll increase this number when we add IMU + odom in Stage 2
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities() {
  // Clean shutdown — important so reconnection works properly
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
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
