#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grabber");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);

    image_transport::Publisher image_pub = it.advertise("arm/frame_raw", 1);

    ros::Rate FPS_HZ(30);

    cv::VideoCapture capture(0);
	
        cv_bridge::CvImage frame_bridge;
        frame_bridge.encoding = "bgr8";

	while (ros::ok())
	{
            capture >> frame_bridge.image;

            cv::flip(frame_bridge.image, frame_bridge.image, 1);

            image_pub.publish(frame_bridge.toImageMsg());

            FPS_HZ.sleep();
	}

    return 0;

}
