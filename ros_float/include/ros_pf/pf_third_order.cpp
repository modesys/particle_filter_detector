#include "ros/ros.h"
#include "pf_third_order.h"
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng libraries

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE        3

#define MU_SYSTEM_NOISE_X 0.0
#define MU_SYSTEM_NOISE_Y 0.0
#define MU_SYSTEM_NOISE_Z 0.0

#define SIGMA_SYSTEM_NOISE_X 0.01
#define SIGMA_SYSTEM_NOISE_Y 0.01
#define SIGMA_SYSTEM_NOISE_Z 0.001

#define MU_USBL_NOISE_X 0.0
#define MU_USBL_NOISE_Y 0.0
#define MU_USBL_NOISE_Z 0.0
#define SIGMA_USBL_NOISE_X 1.0
#define SIGMA_USBL_NOISE_Y 1.0
#define SIGMA_USBL_NOISE_Z 400.0

#define MU_CAMERA_NOISE_X 0.0
#define MU_CAMERA_NOISE_Y 0.0
#define MU_CAMERA_NOISE_Z 0.0
#define SIGMA_CAMERA_NOISE_X 0.5
#define SIGMA_CAMERA_NOISE_Y 0.5
#define SIGMA_CAMERA_NOISE_Z 0.5

#define MU_KELLER_NOISE 0.0
#define SIGMA_KELLER_NOISE 1.0

#define STATE_SIZE 3
#define MEAS_USBL_SIZE 3
#define MEAS_KELLER_SIZE 1
#define MEAS_CAMERA_SIZE 3


#define PRIOR_MU_X 0.0
#define PRIOR_MU_Y 0.0
#define PRIOR_MU_Z 50.0
#define PRIOR_COV_X 50.0
#define PRIOR_COV_Y 50.0
#define PRIOR_COV_Z 5.0

#define NUM_SAMPLES  1000

namespace BFL
{
//bool ROVModel::SampleFrom (Sample<MatrixWrapper::ColumnVector>& one_sample, int method, void * args)
bool ROVModel::SampleFrom (Sample<ColumnVector>& one_sample, int method, void* args) const
{
    // retreive current state and control input
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector acceleration  = ConditionalArgumentGet(1);

    ColumnVector position(3);
    ColumnVector velocity(3);
    // Position
    position(1) = state(1);
    position(2) = state(2);
    position(3) = state(3);
    // velocity
    velocity(1) = state(4);
    velocity(2) = state(5);
    velocity(3) = state(6);

    Sample<ColumnVector> noise;
    _additiveNoise.SampleFrom(noise, method, args);

    position = (position + velocity + noise.ValueGet());
    velocity = (velocity + noise.ValueGet());

    ColumnVector newstate(position, velocity);
    one_sample.ValueSet(newstate);
    return true;
}


Probability USBLModelPdf::ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);

    double roll = measurement(4);
    double pitch = measurement(5);
    double yaw = measurement(6);

    ColumnVector expected_measurement(3);
    expected_measurement(1) = state(1);
    expected_measurement(2) = state(2);
    expected_measurement(3) = state(3);

    ColumnVector actualmeasurement(3);
    actualmeasurement(1) = measurement(1);
    actualmeasurement(2) = measurement(2);
    actualmeasurement(3) = measurement(3);

    (void) roll;
    (void) pitch;
    (void) yaw;
    // error
    return _measNoise.ProbabilityGet(expected_measurement-actualmeasurement);
}

Probability KellerModelPdf::ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
{
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);
    ColumnVector expected_measurement(1);
    expected_measurement(1) = state(3);

    return _measNoise.ProbabilityGet(expected_measurement-measurement);
}


Probability CameraModelPdf::ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
{
    std::cout<<"Passaggio GET-A"<<std::endl;

    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector vel  = ConditionalArgumentGet(1);

    double linearVelocity_X = measurement(4);
    double linearVelocity_Y = measurement(5);
    double linearVelocity_Z = measurement(6);

//    double twistCovariance_X = measurement(1);
//    double twistCovariance_Y = measurement(2);
//    double twistCovariance_Z = measurement(3);

    ColumnVector expected_measurement(3);
    // Velocity
    expected_measurement(1) = state(4);
    expected_measurement(2) = state(5);
    expected_measurement(3) = state(6);

    ColumnVector actualmeasurement(3);
    actualmeasurement(1) = measurement(1);
    actualmeasurement(2) = measurement(2);
    actualmeasurement(3) = measurement(3);

    (void) linearVelocity_X;
    (void) linearVelocity_Y;
    (void) linearVelocity_Z;
//    (void) twistCovariance_X;
//    (void) twistCovariance_Y;
//    (void) twistCovariance_Z;

    // error
    std::cout<<actualmeasurement<<std::endl;
    std::cout<<expected_measurement<<std::endl;

    std::cout<<_measNoise.ProbabilityGet(expected_measurement-actualmeasurement)<<std::endl;
    return _measNoise.ProbabilityGet(expected_measurement-actualmeasurement);
    std::cout<<"Passaggio GET-C"<<std::endl;
}

MyParticleFilter::MyParticleFilter(ros::NodeHandle &n):_n(n)
{
    // system noise
    ColumnVector sysNoise_Mu(STATE_SIZE);
    sysNoise_Mu(1) = MU_SYSTEM_NOISE_X;
    sysNoise_Mu(2) = MU_SYSTEM_NOISE_Y;
    sysNoise_Mu(3) = MU_SYSTEM_NOISE_Z;

    SymmetricMatrix sysNoise_Cov(STATE_SIZE);
    sysNoise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
    sysNoise_Cov(1,2) = 0.0;
    sysNoise_Cov(1,3) = 0.0;
    sysNoise_Cov(2,1) = 0.0;
    sysNoise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
    sysNoise_Cov(2,3) = 0.0;
    sysNoise_Cov(3,1) = 0.0;
    sysNoise_Cov(3,2) = 0.0;
    sysNoise_Cov(3,3) = SIGMA_SYSTEM_NOISE_Z;

    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf = new ROVModel(system_Uncertainty);
    sys_model = new SystemModel<ColumnVector>(sys_pdf);


    // measurement model of the usbl
    ColumnVector usbl_noise_Mu(MEAS_USBL_SIZE);
    usbl_noise_Mu(1) = MU_USBL_NOISE_X;
    usbl_noise_Mu(2) = MU_USBL_NOISE_Y;
    usbl_noise_Mu(3) = MU_USBL_NOISE_Z;
    SymmetricMatrix usbl_noise_Cov(MEAS_USBL_SIZE);
    usbl_noise_Cov(1,1) = SIGMA_USBL_NOISE_X;
    usbl_noise_Cov(1,2) = 0.0;
    usbl_noise_Cov(1,3) = 0.0;
    usbl_noise_Cov(2,1) = 0.0;
    usbl_noise_Cov(2,2) = SIGMA_USBL_NOISE_Y;
    usbl_noise_Cov(2,3) = 0.0;
    usbl_noise_Cov(3,1) = 0.0;
    usbl_noise_Cov(3,2) = 0.0;
    usbl_noise_Cov(3,3) = SIGMA_USBL_NOISE_Z;

    Gaussian measurement_Uncertainty(usbl_noise_Mu, usbl_noise_Cov);
    usbl_meas_pdf = new USBLModelPdf(measurement_Uncertainty);
    usbl_meas_model = new MeasurementModel<ColumnVector,ColumnVector>(usbl_meas_pdf);


    // measurement model of the keller
    ColumnVector keller_noise_Mu(MEAS_KELLER_SIZE);
    keller_noise_Mu(1) = MU_KELLER_NOISE;
    SymmetricMatrix keller_noise_Cov(MEAS_KELLER_SIZE);
    keller_noise_Cov(1,1) = SIGMA_KELLER_NOISE;

    Gaussian keller_measurement_Uncertainty(keller_noise_Mu, keller_noise_Cov);
    keller_meas_pdf = new KellerModelPdf(keller_measurement_Uncertainty);
    keller_meas_model = new MeasurementModel<ColumnVector,ColumnVector>(keller_meas_pdf);


    // measurement model of the camera
    ColumnVector camera_noise_Mu(MEAS_CAMERA_SIZE);
    camera_noise_Mu(1) = MU_CAMERA_NOISE_X;
    camera_noise_Mu(2) = MU_CAMERA_NOISE_Y;
    camera_noise_Mu(3) = MU_CAMERA_NOISE_Z;
    SymmetricMatrix camera_noise_Cov(MEAS_CAMERA_SIZE);
    camera_noise_Cov(1,1) = SIGMA_CAMERA_NOISE_X;
    camera_noise_Cov(1,2) = 0.0;
    camera_noise_Cov(1,3) = 0.0;
    camera_noise_Cov(2,1) = 0.0;
    camera_noise_Cov(2,2) = SIGMA_CAMERA_NOISE_Y;
    camera_noise_Cov(2,3) = 0.0;
    camera_noise_Cov(3,1) = 0.0;
    camera_noise_Cov(3,2) = 0.0;
    camera_noise_Cov(3,3) = SIGMA_CAMERA_NOISE_Z;

    Gaussian camera_measurement_Uncertainty(camera_noise_Mu, camera_noise_Cov);
    camera_meas_pdf = new CameraModelPdf(camera_measurement_Uncertainty);
    camera_meas_model = new MeasurementModel<ColumnVector,ColumnVector>(camera_meas_pdf);

    // prior distribution
    ColumnVector prior_Mu(STATE_SIZE);
    prior_Mu(1) = PRIOR_MU_X;
    prior_Mu(2) = PRIOR_MU_Y;
    prior_Mu(3) = PRIOR_MU_Z;

    SymmetricMatrix prior_Cov(STATE_SIZE);
    prior_Cov(1,1) = PRIOR_COV_X;
    prior_Cov(1,2) = 0.0;
    prior_Cov(1,3) = 0.0;
    prior_Cov(2,1) = 0.0;
    prior_Cov(2,2) = PRIOR_COV_Y;
    prior_Cov(2,3) = 0.0;
    prior_Cov(3,1) = 0.0;
    prior_Cov(3,2) = 0.0;
    prior_Cov(3,3) = PRIOR_COV_Z;

    Gaussian prior(prior_Mu, prior_Cov);
    vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
    prior.SampleFrom(prior_samples, NUM_SAMPLES,CHOLESKY, NULL);
    MCPdf<ColumnVector> prior_discr(NUM_SAMPLES,3);
    prior_discr.ListOfSamplesSet(prior_samples);

    filter = new BootstrapFilter<ColumnVector,ColumnVector>(&prior_discr, 0, NUM_SAMPLES/4.0);

    // init ROS stuff
    imu_sub = n.subscribe("/imu/data_w_orientation", 1000, &MyParticleFilter::imuCb, this);
    usbl_sub = n.subscribe("/usbl/pose_projected", 1000, &MyParticleFilter::usblCb, this);
    kell_sub= n.subscribe("/filter/fluid_pressure/depth/", 1000, &MyParticleFilter::kellCb, this);
    odometry_sub = n.subscribe("/cam/odometry", 1000, &MyParticleFilter::camCb, this);

    particle_pub = _n.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/estimate",1);

    particleTimer = n.createTimer(ros::Duration(1), &MyParticleFilter::timerCallback, this);


    }
};


