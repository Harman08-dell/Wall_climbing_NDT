#include <micro_ros_arduino.h>
#include <rmw_microros/time_sync.h>


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

// ---------- ROS OBJECTS ----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t imu_pub;
rcl_timer_t timer;
rclc_executor_t executor;

sensor_msgs__msg__Imu imu_msg;

// ---------- TIME ----------
unsigned long prevTime = 0;

// ---------- LOW-LEVEL MPU READ ----------
void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp (ignore)
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ---------- TIMER CALLBACK ----------
void timer_callback(rcl_timer_t *, int64_t)
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

// ---------- SETUP ----------
void setup()
{
  set_microros_transports();
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(500);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_imu_node", "", &support);

  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw");

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(20),   // 50 Hz
    timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

// ---------- LOOP ----------
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
