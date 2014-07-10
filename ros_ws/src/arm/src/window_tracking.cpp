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
        cv_ptr = cv_bridge::toCvCopy(frame_raw, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::namedWindow("tracking");

    cv::imshow("tracking", cv_ptr->image);
    cv::waitKey(1);

}

    

int main(int argc, char **argv)
{
    ros::init(argc, argv, "window_tracking");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);

    image_transport::Subscriber sub = it.subscribe("arm/tracking", 1, frame_raw_callback);

    ros::spin();

    return 0;
}
