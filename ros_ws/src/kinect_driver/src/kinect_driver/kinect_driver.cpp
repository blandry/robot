#include "kinect_driver.h"

KinectDriver::KinectDriver()
{ 
}

KinectDriver::~KinectDriver()
{
    kinect_device_.release();
}

bool KinectDriver::loadDevice()
{
    kinect_device_.open( CV_CAP_OPENNI ); 

    if(kinect_device_.isOpened()) return true;
    else return false;
} 

bool KinectDriver::set(int prop_id, double value)
{
    return kinect_device_.set( prop_id, value );
}

double KinectDriver::get( int prop_id )
{
    return kinect_device_.get( prop_id );
}

bool KinectDriver::retrieve( cv::Mat &img, int channel)
{
    if(!kinect_device_.grab()) return false;

    return kinect_device_.retrieve( img, channel );
}
