#include "driver.h"

Driver::Driver(std::string topic_name, int queue_size, int channel, int rate)
    :it_(n_),
    loop_rate_(rate),
    device_opened_(false),
    channel_(channel)
{
  switch(channel_)
  {
    case BGR:
      bridge_.encoding = sensor_msgs::image_encodings::BGR8;
      ROS_INFO("BGR channel choosed");
      break;
    case GRAY:
      bridge_.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case DEPTH_MAP:
      bridge_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      break;
    case POINT_CLOUD:
      bridge_.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
      break;
    case DISPARITY_8C:
      bridge_.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case DISPARITY_32F:
      bridge_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;
    case VALID_DEPTH_MASK:
      bridge_.encoding = sensor_msgs::image_encodings::MONO8; 
  } 
    pub_ = it_.advertise(topic_name, queue_size);
}

Driver::~Driver()
{
}

void Driver::openDevice()
{
    if(kinect_driver_.loadDevice()){
        ROS_INFO("Kinect device opened");
        device_opened_ = true;
    }else{
        ROS_ERROR("Could not open kinect device");
        device_opened_ = false;
    }
}

bool Driver::isOpened()
{
    return device_opened_;
}

void Driver::publish()
{ 
  bool ok = false;
  switch(channel_)
  {
    case BGR: 
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_BGR_IMAGE );
      break;
    case GRAY:
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_GRAY_IMAGE );
      break;
    case DEPTH_MAP:
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_DEPTH_MAP );
      break;
    case POINT_CLOUD:
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_POINT_CLOUD_MAP );
      break;
    case DISPARITY_8C:
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_DISPARITY_MAP );
      break;
    case DISPARITY_32F:
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_DISPARITY_MAP_32F );
      break;
    case VALID_DEPTH_MASK:
      ok = kinect_driver_.retrieve( bridge_.image, CV_CAP_OPENNI_VALID_DEPTH_MASK );
      break; 
  } 

    if(ok){ 
        pub_.publish( bridge_.toImageMsg() );
    }
}

void Driver::sleep()
{
    loop_rate_.sleep();
}
