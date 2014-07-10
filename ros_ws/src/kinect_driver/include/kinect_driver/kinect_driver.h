#ifndef KINECT_DRIVER_H
#define KINECT_DRIVER_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class KinectDriver
{
    public:
        KinectDriver();
        ~KinectDriver();

        bool loadDevice();
        bool set(int, double);
        double get(int);

        bool retrieve(cv::Mat&, int);

    private:
        cv::VideoCapture kinect_device_;
};

#endif
