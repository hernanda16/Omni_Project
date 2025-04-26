// Copyright 2020 amsl

#include "robot/dwa_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dwa_planner");
    DWAPlanner planner;
    planner.process();
    return 0;
}
