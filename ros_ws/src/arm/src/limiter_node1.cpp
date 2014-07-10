#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

class limiter_sb
{
    public:

        limiter_sb()
            :it(n_)
        {

            lh = 0;
            ls = 0;
            lv = 0;

            uh = 180;
            us = 255;
            uv = 255;

            win_name = "limiter";
            setWindowParams();

            pub = it.advertise("arm/thresholded_frame", 1);
            sub = it.subscribe("arm/frame_raw", 1, &limiter_sb::frame_raw_callback, this);
        }

        void frame_raw_callback(const sensor_msgs::ImageConstPtr& frame_raw)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(frame_raw, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::blur(cv_ptr->image, cv_ptr->image, cv::Size(5,5));
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);
            cv::blur(cv_ptr->image, cv_ptr->image, cv::Size(5,5));

            cv::Mat output;
            output.rows = cv_ptr->image.rows;
            output.cols = cv_ptr->image.cols;

            cv::inRange(cv_ptr->image, cv::Scalar(lh, ls, lv), cv::Scalar(uh, us, uv), output);

            cv_bridge::CvImage bridge;

            bridge.image = output;
            bridge.encoding = sensor_msgs::image_encodings::MONO8;

            pub.publish(bridge.toImageMsg());

            cv::imshow(win_name, output);
            cv::waitKey(1);
        }



    private:

        int lh, ls, lv, uh, us, uv;
        std::string win_name;

        ros::NodeHandle n_;

        image_transport::ImageTransport it;

        image_transport::Subscriber sub;
        image_transport::Publisher pub;

        inline void setWindowParams()
        {
            cv::namedWindow(win_name);

            cv::createTrackbar("lower hue", win_name, &lh, 180);
            cv::createTrackbar("lower saturate", win_name, &ls, 255);
            cv::createTrackbar("lower value", win_name, &lv, 255);

            cv::createTrackbar("upper hue", win_name, &uh, 180);
            cv::createTrackbar("upper saturate", win_name, &us, 255);
            cv::createTrackbar("upper value", win_name, &uv, 255);
            
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "limiter1");

    limiter_sb limiter;

    ros::spin();

    return 0;
}
