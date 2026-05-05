#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Motor pins
int dira = 21;  // Left motor direction
int pwma = 19;  // Left motor PWM
int dirb =  5; // Right motor direction
int pwmb = 4;  // Right motor PWM

// micro-ROS entities
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return false; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Subscriber callback
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
  float x = twist_msg->linear.x;
  float y = twist_msg->angular.z;

  if(x == 0 && y == 0)
    stopMotor();
  else if(x > 0 && y > 0)
    turn_left(x, y);
  else if(x < 0 && y > 0)
    reverseturn_left(x, y);
  else if(x > 0 && y < 0)
    turn_right(x, y);
  else if(x < 0 && y < 0)
    reverseturn_right(x, y);
  else if(x > 0 && y == 0)
    forward(x);
  else if(x < 0 && y == 0)
    backward(x);
  else if(x == 0 && y > 0)
    left(y);
  else if(x == 0 && y < 0)
    right(-y);
}

// Create micro-ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "front_motor_driver", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));

  return true;
}

// Destroy micro-ROS entities
void destroy_entities() {
  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rclc_support_fini(&support));
}

// Arduino setup
void setup() {
  Serial.begin(115200);
  set_microros_transports(); // Configure micro-ROS transport
  

  pinMode(dira, OUTPUT);
  pinMode(dirb, OUTPUT);
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);

  if (!create_entities()) {
    while(1) {
      Serial.println("Error setting up micro-ROS!");
      delay(1000);
    }
  }
}

// Arduino loop
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// Motor control functions
void forward(float speed) {
  speed = abs(speed * 255);
  digitalWrite(dira, HIGH);  
  analogWrite(pwma, (int)speed);  
  digitalWrite(dirb, HIGH);  
  analogWrite(pwmb, (int)speed);
  Serial.println("forward");
}

void backward(float speed) {
  speed = abs(speed * 255);
  digitalWrite(dira, LOW);  
  analogWrite(pwma, (int)speed);  
  digitalWrite(dirb, LOW);  
  analogWrite(pwmb, (int)speed);
  Serial.println("backward");
}

void stopMotor() {
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
  Serial.println("stop");
}

// Smooth turning functions for front motors only
void turn_left(float a, float b) {
  a = abs(a);
  int base = constrain((int)(a * 255), 0, 255);
  int slow = base / 2;

  digitalWrite(dira, HIGH);
  analogWrite(pwma, slow);   // Left slower
  digitalWrite(dirb, HIGH);
  analogWrite(pwmb, base);   // Right faster
  Serial.println("smooth turn_left");
}

void turn_right(float a, float b) {
  a = abs(a);
  int base = constrain((int)(a * 255), 0, 255);
  int slow = base / 2;

  digitalWrite(dira, HIGH);
  analogWrite(pwma, base);   // Left faster
  digitalWrite(dirb, HIGH);
  analogWrite(pwmb, slow);   // Right slower
  Serial.println("smooth turn_right");
}

void reverseturn_left(float a, float b) {
  a = abs(a);
  int base = constrain((int)(a * 255), 0, 255);
  int slow = base / 2;

  digitalWrite(dira, LOW);
  analogWrite(pwma, slow);
  digitalWrite(dirb, LOW);
  analogWrite(pwmb, base);
  Serial.println("smooth reverseturn_left");
}

void reverseturn_right(float a, float b) {
  a = abs(a);
  int base = constrain((int)(a * 255), 0, 255);
  int slow = base / 2;

  digitalWrite(dira, LOW);
  analogWrite(pwma, base);
  digitalWrite(dirb, LOW);
  analogWrite(pwmb, slow);
  Serial.println("smooth reverseturn_right");
}


// Simple left/right rotation (x==0)
void left(float speed) {
  speed = abs(speed * 255);
  digitalWrite(dira, HIGH);  
  analogWrite(pwma, (int)(speed * 0.5));  // Slow arc
  digitalWrite(dirb, HIGH);  
  analogWrite(pwmb, (int)speed);
  Serial.println("arc left");
}

void right(float speed) {
  speed = abs(speed * 255);
  digitalWrite(dira, HIGH);  
  analogWrite(pwma, (int)speed);
  digitalWrite(dirb, HIGH);  
  analogWrite(pwmb, (int)(speed * 0.5));  // Slow arc
  Serial.println("arc right");
}