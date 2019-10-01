#include <ros/ros.h>
#include "../include/ros_pf/pf_second_order_water_depth.h"
#include <iostream>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_water_depth");
    ros::NodeHandle n;
    //DiffDrive dd(n);
    MyParticleFilter pf(n);
    // dd.cmd_vel(0.1, 0.2);
    ros::spin();
    return 0;
}
