#ifndef BGR_DRIVER_H
#define BGR_DRIVER_H

#include "kinect_driver.h"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

class BGRDriver
{
    public:
        explicit BGRDriver(std::string, int, int=30);
        ~BGRDriver();

        void openDevice();
        bool isOpened();
        void publish();
        void sleep();

    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Publisher pub_;
        ros::Rate loop_rate_;

        KinectDriver kinect_driver_; 
        bool device_opened_; 

        cv_bridge::CvImage bridge;
};

#endif
