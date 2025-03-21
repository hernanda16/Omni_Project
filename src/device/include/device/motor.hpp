#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

// Global variables
pose_t robot_pose = { 0.0, 0.0, 0.0 }; // x, y, theta
pose_t robot_vel = { 0.0, 0.0, 0.0 }; // vx, vy, omega

// ROS objects
ros::Timer timer_main;
ros::Subscriber sub_cmd_vel;
ros::Publisher pub_pose_robot;

// Function prototypes
void timer_callback(const ros::TimerEvent&);
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr&);
void inverse_kinematics(float vx, float vy, float vtheta);
void pose_estimation();
void publish_pose();

#endif