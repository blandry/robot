#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_generator"); //First thing to call on a ros node.

    //The ros::NodeHandle objects starts the node. The first node handle starts it, and the last destroyed, destroys the node.
    ros::NodeHandle nh;

    //When publishing/subscribing images, it is recommended
    //to use an ImageTransport object to generate the publishers and subscribers.
    image_transport::ImageTransport it(nh);

    image_transport::Publisher bgr_image_pub  = it.advertise("bgr_image", 1); //advertise in the bgr_image topic.
    image_transport::Publisher gray_image_pub = it.advertise("gray_image", 1); //advertise in the gray_image topic.

    cv::VideoCapture kinect; //Creates a VideoCapture object to manage the kinect device, that is, grab the images and set configurations.
    kinect.open(CV_CAP_OPENNI); //opens the device.  

    while(!kinect.isOpened())
    {
        //if not opened, print an error message, wait half a second and try to open it again.
        ROS_ERROR("Kinect device not opened, it may not be connected");
        ros::Duration(0.5).sleep(); 
        kinect.open(CV_CAP_OPENNI);
    }

    //Providing some configurations.
    ROS_INFO("Image generator frame width: %f", kinect.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_WIDTH));
    ROS_INFO("Image generator frame height: %f", kinect.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Image generator FPS: %f", kinect.get(CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_FPS));

    //CvImage objects are resposible for converting a cv::Mat object into a ros sensor_msgs::Image 
    cv_bridge::CvImage bgr_image;
    cv_bridge::CvImage gray_image; 

    //Sets encoding.
    bgr_image.encoding  = sensor_msgs::image_encodings::BGR8;
    gray_image.encoding = sensor_msgs::image_encodings::MONO8;

    ROS_DEBUG("Sleeping time (3 seconds)");
    ros::Duration(3).sleep(); //wait 3 seconds so that everything goes right

    ros::Rate FPS(30);
    while(ros::ok())
    {
        
        if(kinect.grab())
        {
            //DEPTH_MAP
            if(kinect.retrieve(bgr_image.image, CV_CAP_OPENNI_BGR_IMAGE)){
                bgr_image_pub.publish(bgr_image.toImageMsg()); 
            } 
            else
            {
                ROS_WARN("Not able to retrieve a BGR_IMAGE from IMAGE_GENERATOR");
            }

            if(kinect.retrieve(gray_image.image, CV_CAP_OPENNI_GRAY_IMAGE)){
                gray_image_pub.publish(gray_image.toImageMsg());
            }
            else
            {
                ROS_WARN("Not able to retrieve a GRAY_IMAGE from IMAGE_GENERATOR");
            }
        } 
        else
        {
            ROS_WARN("Not able to grab next frame");
        }
        
        FPS.sleep();
    }

    return 0;
} 
