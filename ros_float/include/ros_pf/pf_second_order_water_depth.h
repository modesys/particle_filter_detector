#ifndef __NON_LINEAR_SYSTEM_MOBILE__
#define __NON_LINEAR_SYSTEM_MOBILE__

//#include <pdf/conditionalpdf.h>
//#include <pdf/gaussian.h>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
//#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/pdf/conditionalpdf.h>

#include <bfl/filter/bootstrapfilter.h>

#include "ros/ros.h"
#include <sstream>
#include <stdint.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "ros_float/altitude.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/GetMap.h>
#include <amcl/map/map.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>


//#include <pdf/linearanalyticmeasurementmodel_gaussianuncertainty.h>
using namespace MatrixWrapper;
using namespace std;
using namespace BFL;
using namespace grid_map;

namespace BFL
{
//this class represents the nonlinear system model
class ROVModel : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    ROVModel(const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(5,1)
    {
        _additiveNoise = additiveNoise;
    }

    virtual ~ROVModel(){}

    // the following two functions must be implemented
    // this function is the function f of the discretized model: x_k+1 = f(x_k, u_k)
    //virtual bool SampleFrom (Sample<ColumnVector>& one_sample, int method=DEFAULT, void * args=NULL);
    virtual bool SampleFrom (Sample<MatrixWrapper::ColumnVector>& one_sample, int method=DEFAULT, void * args=NULL) const;


private:
    Gaussian _additiveNoise;
    map_t* _map;

};

// this class represents the nonlinear measurement model of the USBL
class USBLModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    USBLModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(6,2){
            _measNoise = additiveNoise;
    }

    virtual ~USBLModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise; //make this a 3x3 matrix in the constructor of MyParticleFilter
};


// this class represents the nonlinear measurement model of the Keller
class KellerModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    KellerModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~KellerModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};

// this class represents the nonlinear measurement model of the camera
class CameraModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    CameraModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~CameraModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};

class AltitudeModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    AltitudeModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~AltitudeModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};


class WaterDepthMeasModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    WaterDepthMeasModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~WaterDepthMeasModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};



class MyParticleFilter
{
public:
    MyParticleFilter(ros::NodeHandle &n);
    ~MyParticleFilter()
    {
        delete filter;
        delete sys_model;
        delete usbl_meas_model;
        delete keller_meas_model;
        delete sys_pdf;
        delete keller_meas_pdf;
        delete camera_meas_model;
        delete camera_meas_pdf;
        delete alt_meas_pdf;
        delete altitude_meas_model;
    }

    double roll, pitch, yaw;

    // create grid map
    grid_map::GridMap map;

    BootstrapFilter<ColumnVector,ColumnVector>* filter;

    ros::NodeHandle _n;
    ros::Subscriber usbl_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber kell_sub;
    sensor_msgs::Imu imuMsg;

    ros::Publisher particle_pub;
    ros::Publisher pose_pub;
    ros::Publisher best_pub;


    ros::Publisher pose_withcovariance_pub;

    ros::Publisher acc_pub;

    ros::Timer particleTimer;

    // From the camera Odometry
    ros::Subscriber odometry_sub;
    nav_msgs::Odometry odom;

    ros::Subscriber alt_pub;
//    ros_float::altitude altitudeMsg;

    // save the error message
    ros_float::altitude lastAltitudeReceived;


    geometry_msgs::Pose mostLIkelyPose;



    ros::Publisher altErrorPublisher;

    // save the last measurement from keller
    sensor_msgs::FluidPressure lastKellerReceived;

    geometry_msgs::TwistWithCovarianceStamped twcMsg;


    private:
        double easting, northing, usbl_used_inresampling;
        ros::Duration deltaTimeUpdate = ros::Duration(10);
        ros::Time lastUpdate;

        SystemModel<ColumnVector>* sys_model;
        ROVModel* sys_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* usbl_meas_model;
        USBLModelPdf* usbl_meas_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* keller_meas_model;
        KellerModelPdf* keller_meas_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* camera_meas_model;
        CameraModelPdf* camera_meas_pdf;

        MeasurementModel<ColumnVector, ColumnVector>* altitude_meas_model;
        AltitudeModelPdf* alt_meas_pdf;

        MeasurementModel<ColumnVector, ColumnVector>* water_depth_meas_model;
        WaterDepthMeasModelPdf* water_depth_pdf;


        double operator +(ros_float::altitude lastUpdate)
        {
            return lastAltitudeReceived.altitude + lastUpdate.altitude;
        }


        void camCb(const nav_msgs::OdometryConstPtr& msg)
        {
            double linearVelocity_X = msg->twist.twist.linear.x;
            double linearVelocity_Y = msg->twist.twist.linear.y;

            ColumnVector camMeasurement(2);
            camMeasurement(1) = linearVelocity_X;
            camMeasurement(2) = linearVelocity_Y;

            filter->Update(camera_meas_model, camMeasurement);
        }

        void imuCb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            tf::Quaternion q(msg->orientation.x, msg->orientation.y,
                             msg->orientation.z, msg->orientation.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            tf::Matrix3x3 wf(q);

            ColumnVector lin_acc(3);
            lin_acc(1)= msg->linear_acceleration.x;
            lin_acc(2)= msg->linear_acceleration.y;
            lin_acc(3)= msg->linear_acceleration.z;

            ColumnVector lin_acc_filtered(3);
            ColumnVector g(3);
            g(1) = 0;
            g(2) = 0;
            g(3) = 9.8;

            MatrixWrapper::Matrix rot(3,3);
            rot(1,1) = wf[0][0];
            rot(1,2) = wf[0][1];
            rot(1,3) = wf[0][2];
            rot(2,1) = wf[1][0];
            rot(2,2) = wf[1][1];
            rot(2,3) = wf[1][2];
            rot(3,1) = wf[2][0];
            rot(3,2) = wf[2][1];
            rot(3,3) = wf[2][2];

            lin_acc_filtered = rot*lin_acc;
            lin_acc_filtered = lin_acc_filtered - g;

            ColumnVector lin_acc_filtered_reduced(5);
            lin_acc_filtered_reduced(1) = lin_acc_filtered(1);
            lin_acc_filtered_reduced(2) = lin_acc_filtered(2);

//            lin_acc_filtered_reduced(3) = usbl_used_inresampling;
//            lin_acc_filtered_reduced(4) = lat;
//            lin_acc_filtered_reduced(5) = lon;
//            usbl_used_inresampling = 1.0;

            //filter->Update(sys_model, lin_acc_filtered_reduced);
        }

        void altCb(const ros_float::altitude& altitudeMsg)
        {
            if(!lastUpdate.isValid())
            {
                lastUpdate = altitudeMsg.header.stamp;
            }
            // saving the last altitude message received
            lastAltitudeReceived = altitudeMsg;

            if(lastAltitudeReceived.header.stamp > (lastUpdate + deltaTimeUpdate))
            {
                ColumnVector waterDepth(1);
                waterDepth(1) = lastKellerReceived.fluid_pressure + lastAltitudeReceived.altitude;

                std::cout<<"Water depth update:"<<waterDepth<< std::endl;

                filter->Update(water_depth_meas_model, waterDepth);
                lastUpdate = altitudeMsg.header.stamp;
            }
        }

        void kellCb(const sensor_msgs::FluidPressure& msg)
        {
            ColumnVector press(1);
            press(1) = msg.fluid_pressure;

            // saving the last keller message received
            lastKellerReceived = msg;
            //filter->Update(keller_meas_model, press);
        }

        void usblCb(const nav_msgs::Odometry::ConstPtr& msg)
        {
            easting = msg->pose.pose.position.x;
            northing = msg->pose.pose.position.y;

            ColumnVector measurement(2);
            measurement(1) = easting;
            measurement(2) = northing;

            filter->Update(usbl_meas_model, measurement);
        }

        void timerCallback(const ros::TimerEvent&)
        {
            filter->Update(sys_model);
            publishParticles();
            publishPose();
        }

        // Publishing particles for the whole system
        void publishParticles()
        {
            geometry_msgs::PoseArray particleCloudMsg;
            particleCloudMsg.header.stamp = ros::Time::now();
            particleCloudMsg.header.frame_id = "map";

            std::vector<WeightedSample<ColumnVector>>::iterator sample_it;
            std::vector<WeightedSample<ColumnVector>> samples;

            samples = filter->PostGet()->ListOfSamplesGet();

            // create column vector where to save the best particle
//            ColumnVector bestParticle;
//            double bestweight = -1.0; // review
//            WeightedSample<ColumnVector> bestweight;


            for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
            {
                geometry_msgs::Pose pose;
                ColumnVector sample = (*sample_it).ValueGet();

//                WeightedSample<ColumnVector> tempweight = pose.position.x;

//                double weight = (*sample_it).WeightGet();

//                if(weight >= bestweight && weight != 0.0)
//                {
//                    bestParticle = sample;
//                    bestweight = weight;

//                }
                pose.position.x = sample(1);
                pose.position.y = sample(2);
                pose.position.z = -sample(3);

                particleCloudMsg.poses.insert(particleCloudMsg.poses.begin(), pose);

            }
//            geometry_msgs::Pose pose_most_likely;
//            pose_most_likely.position.x = bestParticle(1);
//            pose_most_likely.position.y = bestParticle(2);
//            pose_most_likely.position.z = -bestParticle(3);

//            best_pub.publish(pose_most_likely);
            particle_pub.publish(particleCloudMsg);
        }

        void publishPose(); // publish of the estimate of the state

        void publishBestLikelyPose(); // publish of the most likely pose to avoid dual hypothesis


    };

} // End namespace BFL

#endif //
