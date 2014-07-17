#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "depth_generator"); //First thing to call on a ros node.

    //The ros::NodeHandle objects starts the node. The first node handle starts it, and the last destroyed, destroys the node.
    ros::NodeHandle nh;

    //When publishing/subscribing images, it is recommended
    //to use an ImageTransport object to generate the publishers and subscribers.
    image_transport::ImageTransport it(nh);

    image_transport::Publisher depth_map_pub         = it.advertise("depth_map", 1); //advertise in the depth_map topic.
    image_transport::Publisher point_cloud_pub       = it.advertise("point_cloud_map", 1); //advertise in the point_cloud_map topic.
    image_transport::Publisher disparity_map_pub     = it.advertise("disparity_map", 1); //advertise in the disparity_map topic.
    image_transport::Publisher disparity_map_32f_pub = it.advertise("disparity_map_32f", 1); //advertise in the disparity_map_32f topic.
    image_transport::Publisher valid_mask_pub        = it.advertise("valid_depth_mask", 1); //advertise in the valid_depth_mask topic.

    cv::VideoCapture kinect; //Creates a VideoCapture object to manage the kinect device, that is, grab the images and set configurations.
    kinect.open(CV_CAP_OPENNI); //opens the device.  

    while(!kinect.isOpened())
    {
        //if not opened, print an error message, wait half a second and try to open it again.
        ROS_ERROR("Kinect device not opened, it may not be connected");
        ros::Duration(0.5).sleep(); 
        kinect.open(CV_CAP_OPENNI);
    }

    //Setting Registration, and providing some configurations.
    ROS_INFO("Setting Registration on, depth generator is now alligned with image generator");
    kinect.set(CV_CAP_PROP_OPENNI_REGISTRATION, 1);
    ROS_INFO("Depth generator frame width: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_FRAME_WIDTH));
    ROS_INFO("Depth generator frame height: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Depth generator frame max depth: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH));
    ROS_INFO("Depth generator FPS: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_FPS));
    ROS_INFO("Depth generator registration: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_FRAME_HEIGHT));
    ROS_INFO("Depth generator baseline: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_OPENNI_BASELINE));
    ROS_INFO("Depth generator focal length: %f", kinect.get(CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_OPENNI_FOCAL_LENGTH));

    //CvImage objects are resposible for converting a cv::Mat object into a ros sensor_msgs::Image

    cv_bridge::CvImage depth_map;
    cv_bridge::CvImage point_cloud;
    cv_bridge::CvImage disparity_map;
    cv_bridge::CvImage disparity_map_32f;
    cv_bridge::CvImage valid_mask; 

    depth_map.encoding         = sensor_msgs::image_encodings::TYPE_16UC1;
    point_cloud.encoding       = sensor_msgs::image_encodings::TYPE_32FC3;
    disparity_map.encoding     = sensor_msgs::image_encodings::MONO8;
    disparity_map_32f.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    valid_mask.encoding        = sensor_msgs::image_encodings::MONO8;

    ROS_DEBUG("Sleeping time (3 seconds)");
    ros::Duration(3).sleep(); //wait 5 seconds so that everything goes right

    ros::Rate FPS(30);
    while(ros::ok())
    {
        
        if(kinect.grab())
        {
            //retrieves a DEPTH_MAP
            if(kinect.retrieve(depth_map.image, CV_CAP_OPENNI_DEPTH_MAP)){
                //if it was possible to retrieve the map, publish it
                depth_map_pub.publish(depth_map.toImageMsg()); 
            } 
            else
            {
                //if a depth_map could not be retrieved, show a warning
                ROS_WARN("Not able to retrieve a DEPTH_MAP image");
            }
            
            //retrieves a point_cloud
            if(kinect.retrieve(point_cloud.image, CV_CAP_OPENNI_POINT_CLOUD_MAP)){
                //if it was possible to retrieve the map, publish it
                point_cloud_pub.publish(point_cloud.toImageMsg());
            }
            else
            {
                //else show a warning
                ROS_WARN("Not able to retrieve a POINT_CLOUD_MAP from DEPTH_GENERATOR");
            }

            //retrieves a disparity_map
            if(kinect.retrieve(disparity_map.image, CV_CAP_OPENNI_DISPARITY_MAP)){ 
                //if it was possible to retrieve the map, publish it
                disparity_map_pub.publish(disparity_map.toImageMsg());
            }
            else
            {
                //else show a warning
                ROS_WARN("Not able to retrieve a DISPARITY_MAP from DEPTH_GENERATOR");
            }

            //retrieves a disparity_map_32f
            if(kinect.retrieve(disparity_map_32f.image, CV_CAP_OPENNI_DISPARITY_MAP_32F)){ 
                //if it was possible to retrieve the map, publish it
                disparity_map_32f_pub.publish(disparity_map_32f.toImageMsg());
            }
            else
            {
                //else show a warning
                ROS_WARN("Not able to retrieve a DISPARITY_MAP_32F from DEPTH_GENERATOR"); 
            }

            if(kinect.retrieve(valid_mask.image, CV_CAP_OPENNI_VALID_DEPTH_MASK)){ 
                //if it was possible to retrieve the mask, publish it
                valid_mask_pub.publish(valid_mask.toImageMsg());
            }
            else
            {
                //else show a warning
                ROS_WARN("Not able to retrieve a VALID_DEPTH_MASK from DEPTH_GENERATOR"); 
            }

        } 
        else
        {
            ROS_WARN("Could not grab next frame");
        }
        
        FPS.sleep();
    }

    return 0;
} 
