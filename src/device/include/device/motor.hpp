#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <array>
#include <vector>
#include "std_msgs/Int16MultiArray.h"
#include "pid_controller.h"

#define LOOP_RATE 20
#define QUEUE_SIZE 1 // Subscriber buffer size
#define DEG2RAD (M_PI / 180.0) // Degrees to radians conversion factor
#define ENC_CPR 12.0 // Encoder counts/rev.
#define GEAR_REDUC 64.0 // Gears reduction ratio
#define TS (1.0 / 20.0) // Loop period in seconds
#define CPP2RADPS (2.0 * M_PI / (TS * ENC_CPR * GEAR_REDUC)) // Counts per loop period to rad/s conversion factor
#define DEADBAND 10 // Stops actuating motors when: -DEADBAND < actuation < DEADBAND

class NexusMotorController
{
public:
    NexusMotorController();

private:
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& twist_aux);
    void encoderCallback(const std_msgs::Int16MultiArray::ConstPtr& enc_aux);

    ros::NodeHandle nh;
    ros::Publisher cmd_motor_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber enc_sub;

    ros::Time last_time;

    float Kp, Ki, Kd, minOutput, maxOutput;

    std::array<double, 4> wheelSpeed;
    std::array<double, 4> wheelDeg;
    std::array<double, 4> wheelSin;
    std::array<double, 4> wheelCos;

    std::array<double, 4> vel_fb;
    std::array<double, 4> prev_vel_fb;

    double lin_x;
    double lin_y;
    double ang_z;

    double dt;

    std::vector<PIDControl> myPID_wheel;

    double WHEEL_RADIUS;
    double DISTANCE_W2MID;
};

#endif // MOTOR_HPP