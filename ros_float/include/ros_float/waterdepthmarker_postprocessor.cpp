#include "waterdepthmarker_postprocessor.h"


waterdepthmarker_postProcessor::waterdepthmarker_postProcessor()
{

}

void waterdepthmarker_postProcessor::writePCloudToFile(std::string filename, ros::NodeHandle &n)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ofstream myfile;
    myfile.open ("/home/emanuele/Desktop/cloud.txt");

    if (myfile.is_open()) {
        for(size_t i = 0; i <   cloud->points.size(); ++i)
               myfile << " " << cloud->points[i].x
                      << " " << cloud->points[i].y
                      << " " << cloud->points[i].z << std::endl;
        myfile.close();
    }

    particleTimer = n.createTimer(ros::Duration(1), &waterdepthmarker_postProcessor::timerCallback, this);
    (void) filename;
}

void waterdepthmarker_postProcessor::publishPoseXYZ()
{
    geometry_msgs::PoseArray poseArrayMsg;
    poseArrayMsg.header.stamp = ros::Time::now();
    poseArrayMsg.header.frame_id = "map";

    geometry_msgs::Pose poseMsg;
//    poseMsg.position.x = /* position x */;
//    poseMsg.position.y = /* position x */;
//    poseMsg.position.z = /* position x */;

    poseArrayMsg.poses.insert(poseArrayMsg.poses.begin(), poseMsg);
    particle_pub.publish(poseArrayMsg);
}

void waterdepthmarker_postProcessor::publishPointCloud()
{
    sensor_msgs::PointCloud2 particleCloudMsg;
    particleCloudMsg.header.stamp = ros::Time::now();
    particleCloudMsg.header.frame_id = "map";
    cloud_pub.publish(particleCloudMsg);
}

void waterdepthmarker_postProcessor::publishTrajectory()
{
    sensor_msgs::PointCloud2 particleCloudMsg;
    ros::Publisher pub = _n.advertise<sensor_msgs::PointCloud2>("trajectory", 1, true);

    ros::Rate rate(.1);
    while (_n.ok())
    {
        ros::Time time = ros::Time::now();
        sensor_msgs::PointCloud2 msg;
//        sensor_msgs::PointCloud2::toMessage(particleCloudMsg, msg);
        pub.publish(msg);
//        sensor_msgs::PointCloud2::saveToBag(map, "/home/emanuele/Desktop/grid_map_big_island_5ms.bag","grid_map");
        // wait for next cycle
        rate.sleep();
    }
}
