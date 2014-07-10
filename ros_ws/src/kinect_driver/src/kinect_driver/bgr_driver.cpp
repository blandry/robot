#include "bgr_driver.h"

BGRDriver::BGRDriver(std::string topic_name, int queue_size, int rate)
    :it_(n_),
    loop_rate_(rate),
    device_opened_(false)
{
    pub_ = it_.advertise(topic_name, queue_size);
    bridge.encoding = sensor_msgs::image_encodings::BGR8;
}

BGRDriver::~BGRDriver()
{
}

void BGRDriver::openDevice()
{
    if(kinect_driver_.loadDevice()){
        ROS_INFO("Kinect device opened");
        device_opened_ = true;
    }else{
        ROS_ERROR("Could not open kinect device");
        device_opened_ = false;
    }
}

bool BGRDriver::isOpened()
{
    return device_opened_;
}

void BGRDriver::publish()
{ 
    bool ok = kinect_driver_.retrieve( bridge.image, CV_CAP_OPENNI_BGR_IMAGE );

    if(ok){ 
        pub_.publish( bridge.toImageMsg() );
    }
}

void BGRDriver::sleep()
{
    loop_rate_.sleep();
}
