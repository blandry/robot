#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

void frame_raw_callback(const sensor_msgs::ImageConstPtr& frame_raw)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(frame_raw);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::namedWindow("webcam_threshold");

    if(cv_ptr->image.data) cv::imshow("webcam_threshold", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "windowT");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);

    if(argc == 2) image_transport::Subscriber sub = it.subscribe(argv[1], 1, frame_raw_callback);

    image_transport::Subscriber sub = it.subscribe("kinect/frame_raw", 1, frame_raw_callback);

    ros::spin();

    return 0;
}
