#include "motor.hpp"

NexusMotorController::NexusMotorController()
    : wheelSpeed({0, 0, 0, 0}),
      wheelDeg({45, 135, 225, 315}),
      wheelSin({sin(wheelDeg[0] * DEG2RAD), sin(wheelDeg[1] * DEG2RAD), sin(wheelDeg[2] * DEG2RAD), sin(wheelDeg[3] * DEG2RAD)}),
      wheelCos({cos(wheelDeg[0] * DEG2RAD), cos(wheelDeg[1] * DEG2RAD), cos(wheelDeg[2] * DEG2RAD), cos(wheelDeg[3] * DEG2RAD)}),
      vel_fb({0, 0, 0, 0}),
      prev_vel_fb({0, 0, 0, 0}),
      lin_Vx(0),
      lin_Vy(0),
      ang_Vz(0),
      dt(0)
{
    nh.param("R", WHEEL_RADIUS, 0.05);
    nh.param("D", DISTANCE_W2MID, 0.175);
    nh.param("motor_kp", Kp, 1.0f);
    nh.param("motor_ki", Ki, 0.0f);
    nh.param("motor_kd", Kd, 0.0f);
    nh.param("pid_min_out", minOutput, -150.0f);
    nh.param("pid_max_out", maxOutput, 150.0f);

    // Tambahkan elemen-elemen PID controllers ke vector
    for (int i = 0; i < 4; i++)
    {
        myPID_wheel.emplace_back(Kp, Ki, Kd, TS, minOutput, maxOutput, AUTOMATIC, DIRECT);
    }

    cmd_motor_pub = nh.advertise<std_msgs::Int16MultiArray>("/device/cmd_motor", QUEUE_SIZE);
    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/robot/cmd_vel", QUEUE_SIZE, &NexusMotorController::velocityCallback, this);
    enc_sub = nh.subscribe<std_msgs::Int16MultiArray>("/device/raw_enc", QUEUE_SIZE, &NexusMotorController::encoderCallback, this);

    last_time = ros::Time::now();
}

void NexusMotorController::velocityCallback(const geometry_msgs::Twist::ConstPtr& twist_aux)
{
    lin_Vx = twist_aux->linear.x;
    lin_Vy = twist_aux->linear.y;
    ang_Vz = twist_aux->angular.z;

    wheelSpeed[0] = 1 / WHEEL_RADIUS * (-wheelSin[0] * lin_Vx + wheelCos[0] * lin_Vy + DISTANCE_W2MID * ang_Vz);
    wheelSpeed[1] = 1 / WHEEL_RADIUS * (-wheelSin[1] * lin_Vx - wheelCos[1] * lin_Vy + DISTANCE_W2MID * ang_Vz);
    wheelSpeed[2] = 1 / WHEEL_RADIUS * (wheelSin[2] * lin_Vx - wheelCos[2] * lin_Vy + DISTANCE_W2MID * ang_Vz);
    wheelSpeed[3] = 1 / WHEEL_RADIUS * (wheelSin[3] * lin_Vx + wheelCos[3] * lin_Vy + DISTANCE_W2MID * ang_Vz);

    ROS_INFO("Wheel speeds: [%f, %f, %f, %f]", wheelSpeed[0], wheelSpeed[1], wheelSpeed[2], wheelSpeed[3]);
}

void NexusMotorController::encoderCallback(const std_msgs::Int16MultiArray::ConstPtr& enc_aux)
{
    std_msgs::Int16MultiArray cmd_motor;

    ros::Time current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    last_time = current_time;

    for (int i = 0; i < 4; i++)
    {
        vel_fb[i] = enc_aux->data[i] * CPP2RADPS;
    }

    bool should_publish = false; // Flag to check if publishing is needed
    for (int i = 0; i < 4; i++)
    {
        myPID_wheel[i].PIDSetpointSet(wheelSpeed[i]);
        myPID_wheel[i].PIDInputSet(vel_fb[i]);
        myPID_wheel[i].PIDCompute();

        short motor_command = (short)round(myPID_wheel[i].PIDOutputGet());
        cmd_motor.data.push_back(motor_command);

        if (motor_command <= -DEADBAND || motor_command >= DEADBAND)
        {
            should_publish = true;
        }
    }

    if (should_publish)
    {
        ROS_INFO("Publishing motor commands: [%d, %d, %d, %d]",
                 cmd_motor.data[0], cmd_motor.data[1], cmd_motor.data[2], cmd_motor.data[3]);
        cmd_motor_pub.publish(cmd_motor);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmc");
    NexusMotorController nmc;
    ros::spin();
    return 0;
}