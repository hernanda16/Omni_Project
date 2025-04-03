#include "device/motor.hpp"

NexusMotorController::NexusMotorController()
    : wheelSpeed({ 0, 0, 0, 0 })
    , wheelDeg({ 315, 45, 135, 225 })
    , wheelSin({ sin(wheelDeg[0] * DEG2RAD), sin(wheelDeg[1] * DEG2RAD), sin(wheelDeg[2] * DEG2RAD), sin(wheelDeg[3] * DEG2RAD) })
    , wheelCos({ cos(wheelDeg[0] * DEG2RAD), cos(wheelDeg[1] * DEG2RAD), cos(wheelDeg[2] * DEG2RAD), cos(wheelDeg[3] * DEG2RAD) })
    , vel_fb({ 0, 0, 0, 0 })
    , prev_vel_fb({ 0, 0, 0, 0 })
    , lin_x(0)
    , lin_y(0)
    , ang_z(0)
    , dt(0)
{
    WHEEL_RADIUS = nh.param<float>("R", 0.05f);
    DISTANCE_W2MID = nh.param<float>("D", 0.175f);
    Kp = nh.param<float>("motor_kp", 1.0f);
    Ki = nh.param<float>("motor_ki", 0.0f);
    Kd = nh.param<float>("motor_kd", 0.0f);
    minOutput = nh.param<float>("pid_min_out", -150.0f);
    maxOutput = nh.param<float>("pid_max_out", 150.0f);

    printf("======================================\n");
    printf("        MOTOR NODE PARAMETERS         \n");
    printf("======================================\n");
    printf("Wheel Radius\t: %.2f\n", WHEEL_RADIUS);
    printf("Distance W2MID\t: %.2f\n", DISTANCE_W2MID);
    printf("Motor Kp\t: %.2f\n", Kp);
    printf("Motor Ki\t: %.2f\n", Ki);
    printf("Motor Kd\t: %.2f\n", Kd);
    printf("PID Min Out\t: %.2f\n", minOutput);
    printf("PID Max Out\t: %.2f\n", maxOutput);
    printf("======================================\n");

    for (int i = 0; i < 4; i++) {
        myPID_wheel.emplace_back(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
    }

    cmd_motor_pub = nh.advertise<std_msgs::Int16MultiArray>("/device/cmd_motor", QUEUE_SIZE);
    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/robot/cmd_vel", QUEUE_SIZE, &NexusMotorController::velocityCallback, this);
    enc_sub = nh.subscribe<std_msgs::Int16MultiArray>("/device/raw_enc", QUEUE_SIZE, &NexusMotorController::encoderCallback, this);

    tim_motor = nh.createTimer(ros::Duration(0.01), &NexusMotorController::motorTimer, this);

    last_time = ros::Time::now();
}

void NexusMotorController::velocityCallback(const geometry_msgs::Twist::ConstPtr& twist_aux)
{
    lin_x = -twist_aux->linear.x * 10;
    lin_y = -twist_aux->linear.y * 10;
    ang_z = -twist_aux->angular.z * 10;

    // [ INFO] [1742819912.256170617]: lin_x: 0.000000, lin_y: 0.000000, ang_z: 0.000000, WHEEL_RADIUS: 0.050000, DISTANCE_W2MID: 0.175000, wheelSin: [0.707107, 0.707107, -0.707107, -0.707107], wheelCos: [0.707107, -0.707107, -0.707107, 0.707107]

    wheelSpeed[0] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[0] + lin_y * wheelSin[0] + DISTANCE_W2MID * ang_z);
    wheelSpeed[1] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[1] + lin_y * wheelSin[1] + DISTANCE_W2MID * ang_z);
    wheelSpeed[2] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[2] + lin_y * wheelSin[2] + DISTANCE_W2MID * ang_z);
    wheelSpeed[3] = 1 / WHEEL_RADIUS * (lin_x * wheelCos[3] + lin_y * wheelSin[3] + DISTANCE_W2MID * ang_z);
}

void NexusMotorController::encoderCallback(const std_msgs::Int16MultiArray::ConstPtr& enc_aux)
{
    ros::Time current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

    for (int i = 0; i < 4; i++) {
        vel_fb[i] = enc_aux->data[i] * CPP2RADPS;
    }
}

void NexusMotorController::motorTimer(const ros::TimerEvent&)
{
    std_msgs::Int16MultiArray cmd_motor_msg;
    for (int8_t i = 0; i < 4; i++) {
        myPID_wheel[i].PIDSetpointSet(wheelSpeed[i]);
        myPID_wheel[i].PIDInputSet(vel_fb[i]);
        myPID_wheel[i].PIDCompute();
        short motor_command = (short)round(myPID_wheel[i].PIDOutputGet());
        cmd_motor_msg.data.push_back(motor_command);
    }

    ROS_INFO("Publishing motor commands: [%d, %d, %d, %d]",
        cmd_motor_msg.data[0], cmd_motor_msg.data[1], cmd_motor_msg.data[2], cmd_motor_msg.data[3]);
    cmd_motor_pub.publish(cmd_motor_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmc");
    NexusMotorController nmc;

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();
    // ros::spin();
    return 0;
}