#include "../include/ros_float/uncertainty_preprocessor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unc_preprocessor_node");
    uncertainty_preProcessor processor;
    ros::spin();
}
