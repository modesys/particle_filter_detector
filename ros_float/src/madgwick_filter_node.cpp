#include "../include/imu_filter_madgwick/madgwick_filter.h"
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "madgwick_filter_node");
    ros::NodeHandle n;

    std::string filename("hdckadbc");
    madgwick::MADGWICK_reader reader(filename, n);

    // port publisher2 inside constructor exactly as publisher 1
    // or to be precise port it inside the class
    ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3Stamped>("ypr", 1);

    tf::TransformBroadcaster q_brodecaster;
    ros::spin();
}
