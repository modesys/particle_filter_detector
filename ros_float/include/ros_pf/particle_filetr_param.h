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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <nav_msgs/GetMap.h>
#include <amcl/map/map.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


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
    ROVModel(const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(3,2)
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

    ros::Timer particleTimer;

    ros::Subscriber alt_pub;

    // save the error message
    ros_float::altitude lastAltitudeReceived;
    ros::Publisher altErrorPublisher;


    private:
        SystemModel<ColumnVector>* sys_model;
        ROVModel* sys_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* usbl_meas_model;
        USBLModelPdf* usbl_meas_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* keller_meas_model;
        KellerModelPdf* keller_meas_pdf;

        MeasurementModel<ColumnVector, ColumnVector>* altitude_meas_model;
        AltitudeModelPdf* alt_meas_pdf;


        void imuCb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            tf::Quaternion q(msg->orientation.x, msg->orientation.y,
                             msg->orientation.z, msg->orientation.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            ColumnVector lin_acc(3);
            lin_acc(1)= msg->linear_acceleration.x;
            lin_acc(2)= msg->linear_acceleration.y;
            lin_acc(3)= msg->linear_acceleration.z;

            filter->Update(sys_model, lin_acc);
        }

        void altCb(const ros_float::altitude& altitudeMsg)
        {
            ColumnVector alt(1);
            alt(1) = altitudeMsg.altitude;

            lastAltitudeReceived = altitudeMsg;

            std::cout<<std::endl<<" ALT      "<< alt(1)<<std::endl ;

            filter->Update(altitude_meas_model, alt);
        }

        void kellCb(const sensor_msgs::FluidPressure::ConstPtr& msg)
        {
            ColumnVector press(1);
            press(1) = msg->fluid_pressure;
            filter->Update(keller_meas_model, press);
        }



        void usblCb(const nav_msgs::Odometry::ConstPtr& msg)
        {
            double lat = msg->pose.pose.position.x;
            double lon = msg->pose.pose.position.y;
            double alt = msg->pose.pose.position.z;

            ColumnVector measurement(6);
            measurement(1) = lat;
            measurement(2) = lon;
            measurement(3) = alt;

            measurement(4) = roll;
            measurement(5) = pitch;
            measurement(6) = yaw;

            filter->Update(usbl_meas_model, measurement);
        }


        void timerCallback(const ros::TimerEvent&)
        {
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

            for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
            {
                geometry_msgs::Pose pose;
                ColumnVector sample = (*sample_it).ValueGet();

                pose.position.x = -sample(1);
                pose.position.y = -sample(2);
                pose.position.z = -sample(3);

                particleCloudMsg.poses.insert(particleCloudMsg.poses.begin(), pose);
            }
            particle_pub.publish(particleCloudMsg);
        }


        void publishPose(); // publish of the estimate of the state

    };

} // End namespace BFL

#endif //
