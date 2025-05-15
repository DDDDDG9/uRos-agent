#include <micro_ros_arduino.h>
#include <CytronMotorDriver.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Configure the motor driver.
CytronMD motorL(PWM_PWM, 26, 27);   // PWM 1A = Pin 26, PWM 1B = Pin 27.
CytronMD motorR(PWM_PWM, 14, 12);  // PWM 2A = Pin 14, PWM 2B = Pin 12.

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void setMotorSpeed(int speedLeft, int speedRight);

//twist message cb
void cmd_vel_callback(const void *msgin) {
   const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
   
  // Calculate motor speeds based on twist message
  float linear_raw = msg->linear.x;
  float angular_raw = msg->angular.z;

  float wheel_base = 0.25;
  int max_pwm = 255;
  int min_pwm = 100;

  // 1) Apply minimum input threshold for raw joystick values
  float linear = (fabs(linear_raw) < 0.3) ? 0.0 : linear_raw;
  float angular = (fabs(angular_raw) < 0.3) ? 0.0 : angular_raw;

  // 2) Calculate raw motor PWM values
  int leftPWM = (int)((linear - angular * wheel_base / 2.0) * max_pwm);
  int rightPWM = (int)((linear + angular * wheel_base / 2.0) * max_pwm);

  // 3) Prevent motor stall by enforcing minimum PWM magnitude (except zero)
  if (leftPWM != 0 && abs(leftPWM) < min_pwm) {
    leftPWM = (leftPWM > 0) ? min_pwm : -min_pwm;
  }
  if (rightPWM != 0 && abs(rightPWM) < min_pwm) {
    rightPWM = (rightPWM > 0) ? min_pwm : -min_pwm;
  }

  // 4) Handle special case for in-place rotation when linear=0 but angular != 0
  if (linear == 0 && angular != 0) {
    leftPWM = (angular > 0) ? -min_pwm : min_pwm;
    rightPWM = (angular > 0) ? min_pwm : -min_pwm;
  }

  setMotorSpeed(leftPWM, rightPWM);
  
}

void setMotorSpeed(int speedLeft, int speedRight) {

  if (speedLeft != 0) {
      motorL.setSpeed(speedLeft);
  } else {
    motorL.setSpeed(0);
    } 
  // Set right motor direction and speed
  if (speedRight != 0) {
    motorR.setSpeed(speedRight);
  }else {
    motorR.setSpeed(0);
    } 
}

void setup() {
  delay(1000);
  set_microros_transports();
  

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "diff_drive_esp32", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  delay(1000);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  delay(1);
}
