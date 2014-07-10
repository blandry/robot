#include "gray_driver.h"

GrayDriver::GrayDriver(std::string topic_name, int queue_size, int rate)
    :it_(n_),
    loop_rate_(rate),
    device_opened_(false)
{
    pub_ = it_.advertise(topic_name, queue_size);
    bridge.encoding = sensor_msgs::image_encodings::MONO8;
}

GrayDriver::~GrayDriver()
{
}

void GrayDriver::openDevice()
{
    if(kinect_driver_.loadDevice()){
        ROS_INFO("Kinect device opened");
        device_opened_ = true;
    }else{
        ROS_ERROR("Could not open kinect device");
        device_opened_ = false;
    }
}

bool GrayDriver::isOpened()
{
    return device_opened_;
}

void GrayDriver::publish()
{ 
    bool ok = kinect_driver_.retrieve( bridge.image, CV_CAP_OPENNI_GRAY_IMAGE );

    if(ok){ 
        pub_.publish( bridge.toImageMsg() );
    }
}

void GrayDriver::sleep()
{
    loop_rate_.sleep();
}
