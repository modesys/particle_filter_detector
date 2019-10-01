#include <ros/ros.h>
#include "../include/ros_pf/particle_filetr_param.h"
#include <iostream>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle n;
    //DiffDrive dd(n);
    MyParticleFilter pf(n);
    // dd.cmd_vel(0.1, 0.2);
    ros::spin();
    return 0;
}
