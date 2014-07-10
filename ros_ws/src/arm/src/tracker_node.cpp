#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "arm/coordinates.h"

class tracker
{
        
    public:

        tracker()
            :it(n_)
        {
            pub = it.advertise("arm/tracking", 1);
            sub_r = it.subscribe("arm/frame_raw", 1, &tracker::frame_raw_callback, this);
            sub_t = it.subscribe("arm/thresholded_frame", 1, &tracker::frame_thresholded_callback, this);

            pubPoints = n_.advertise<arm::coordinates>("arm/tracked_coordinates", 1);

            noise = 1000;
        }

        void frame_raw_callback(const sensor_msgs::ImageConstPtr& frame_raw)
        {
            try
            {
                bridge_raw = cv_bridge::toCvCopy(frame_raw, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

        }

        void frame_thresholded_callback(const sensor_msgs::ImageConstPtr& thresholded_frame)
        {
cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(thresholded_frame, sensor_msgs::image_encodings::MONO8);                                
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }

            cv::Moments moment= cv::moments(cv_ptr->image, true);

            double moment10 = moment.m10;
            double moment01 = moment.m01;
            double area = moment.m00;


            if(area < noise)
            {
                last_XPos = -1;
                last_YPos = -1;
                current_XPos = -1;
                current_YPos = -1;
            }
            else
            {
                current_XPos = moment10/area;
                current_YPos = moment01/area;
            }

            if(current_XPos >= 0 && current_YPos >= 0 && last_XPos >= 0 && last_YPos >= 0)
            {
                cv::line(bridge_raw->image, cv::Point(current_XPos, current_YPos), cv::Point(last_XPos, last_YPos), cv::Scalar(0, 0, 255), 4);
            }

            pub.publish(bridge_raw->toImageMsg());

            points.X = current_XPos;
            points.Y = current_YPos;

            points.X_changed = current_XPos == last_XPos ? 0 : 1;
            points.Y_changed = current_YPos == last_YPos ? 0 : 1;


            pubPoints.publish(points);

            last_XPos = current_XPos;

            last_YPos = current_YPos;
        }



    private:

        int lh, ls, lv, uh, us, uv;
        std::string win_name;

        ros::NodeHandle n_;

        ros::Publisher pubPoints;

        image_transport::ImageTransport it;

        image_transport::Subscriber sub_r;
        image_transport::Subscriber sub_t;
        image_transport::Publisher pub;

        cv_bridge::CvImagePtr bridge_raw;

        int noise;

        int last_XPos, last_YPos, current_XPos, current_YPos;

        arm::coordinates points;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");

    tracker limiter;

    ros::spin();

    return 0;
}
