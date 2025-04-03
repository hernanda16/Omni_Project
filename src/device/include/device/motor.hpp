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
#define TS (1.0 / 100.0) // Loop period in seconds
#define CPP2RADPS (2.0 * M_PI / (TS * ENC_CPR * GEAR_REDUC)) // Counts per loop period to rad/s conversion factor
#define DEADBAND 10 // Stops actuating motors when: -DEADBAND < ac tuation < DEADBAND

class NexusMotorController {
public:
    NexusMotorController();

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_motor_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber enc_sub;

    ros::Timer tim_motor;

    void velocityCallback(const geometry_msgs::Twist::ConstPtr& twist_aux);
    void encoderCallback(const std_msgs::Int16MultiArray::ConstPtr& enc_aux);

    void motorTimer(const ros::TimerEvent&);

    int16_t buffer_motor[4] = { 0 };

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

    float WHEEL_RADIUS;
    float DISTANCE_W2MID;
};

#endif // MOTOR_HPP