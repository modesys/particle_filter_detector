#ifndef WATERDEPTHMARKER_POSTPROCESSOR_H
#define WATERDEPTHMARKER_POSTPROCESSOR
#define PI 3.14159265358979323846  /* pi */

#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


class waterdepthmarker_postProcessor
{
public:
    waterdepthmarker_postProcessor();
    void writePCloudToFile(std::string filename, ros::NodeHandle &n);
    sensor_msgs::PointCloud2 pCloud2Msg;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Timer particleTimer;
    ros::NodeHandle _n;

    void publishPoseXYZ();
    void publishPointCloud();
    void publishTrajectory();

    sensor_msgs::PointCloud2 loadPointCloudFromBag(std::string filename)
    {
        sensor_msgs::PointCloud2 output;
        //reading the bag where the point cloud is recorded
        rosbag::Bag pCloudBag;
        pCloudBag.open(filename, rosbag::bagmode::Read);

        std::vector<std::string> gridTopic;
        gridTopic.push_back(std::string("trajectory"));
        rosbag::View view(pCloudBag, rosbag::TopicQuery(gridTopic));

        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
            if (s != nullptr)
                output = *s;
        }
        pCloudBag.close();
        return output;
    }


private:

    void timerCallback(const ros::TimerEvent&)
    {
        publishPointCloud();
    }

    ros::Publisher particle_pub;

};


#endif // WATERDEPTHMARKER_POSTPROCESSOR_H
