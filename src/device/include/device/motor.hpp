#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "pid_controller.h"
#include "std_msgs/Int16MultiArray.h"
#include <array>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <vector>

#define LOOP_RATE 20
#define QUEUE_SIZE 1 // Subscriber buffer size
#define DEG2RAD (M_PI / 180.0) // Degrees to radians conversion factor
#define ENC_CPR 12.0 // Encoder counts/rev.
#define GEAR_REDUC 64.0 // Gears reduction ratio
#define TS (1.0 / 20.0) // Loop period in seconds
#define CPP2RADPS (2.0 * M_PI / (TS * ENC_CPR * GEAR_REDUC)) // Counts per loop period to rad/s conversion factor
#define DEADBAND 10 // Stops actuating motors when: -DEADBAND < actuation < DEADBAND

//--Publisher
ros::Publisher pub_cmd_motor;

//--Subscriber
ros::Subscriber sub_cmd_vel;
ros::Subscriber sub_enc_vel;

// PIDControl* myPID_wheel_0;
// PIDControl* myPID_wheel_1;
// PIDControl* myPID_wheel_2;
// PIDControl* myPID_wheel_3;

PIDControl* my_PID;

float wheel_speed[4] = { 0 };
float wheel_deg[4] = { 315, 45, 135, 225 };
float wheel_cos[4] = { sinf(wheel_deg[0] * DEG2RAD), sinf(wheel_deg[1] * DEG2RAD), sinf(wheel_deg[2] * DEG2RAD), sinf(wheel_deg[3] * DEG2RAD) };
float wheel_sin[4] = { cosf(wheel_deg[0] * DEG2RAD), cosf(wheel_deg[1] * DEG2RAD), cosf(wheel_deg[2] * DEG2RAD), cosf(wheel_deg[3] * DEG2RAD) };

int16_t vel_act[4] = { 0 };
int16_t vel_fb[4] = { 0 };
int16_t prev_vel_fb[4] = { 0 };

//--Parameter
float WHEEL_RADIUS = 0;
float DISTANCE_W2MID = 0;
float Kp, Ki, Kd, minOutput, maxOutput;

//--Callback
void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
void callback_enc_vel(const std_msgs::Int16MultiArray::ConstPtr& msg);

#endif // MOTOR_HPP