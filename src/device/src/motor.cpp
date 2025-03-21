#include "device/motor.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(1);

    sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>("/robot/cmd_vel", 1, cmd_vel_callback);
    pub_pose_robot = nh.advertise<geometry_msgs::Pose>("/device/pose_robot", 1);
    timer_main = nh.createTimer(ros::Duration(0.1), timer_callback);

    spinner.spin();

    return 0;
}

void timer_callback(const ros::TimerEvent&)
{
    pose_estimation();
    publish_pose();
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    robot_vel.x = msg->linear.x;
    robot_vel.y = msg->linear.y;
    robot_vel.theta = msg->angular.z;

    inverse_kinematics(robot_vel.x, robot_vel.y, robot_vel.theta);
}

void inverse_kinematics(float vx, float vy, float vtheta)
{
    // ==== INVERSE KINEMATICS ==== //
    //@note: Data dari format m/s dan rad/s
    //@note: Data dikirim ke format pwm
}

void pose_estimation()
{
    // ==== POSE ESTIMATION ==== //
    //@note: Data dari format counter
    //@note: Data dikirim ke format m
}

void publish_pose()
{
    // ==== DATA DARI ENCODER ==== //
    geometry_msgs::Pose2D pose_robot;
    pose_robot.x = 0.0;
    pose_robot.y = 0.0;
    pose_robot.theta = 0.0;
    pub_pose_robot.publish(pose_robot);
}