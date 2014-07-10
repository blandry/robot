#ifndef LIMITER_H
#define LIMITER_H

#include "opencv2/opencv.hpp"

class limiter
{
public:

    limiter();
    ~limiter();

    void setWindowParams();
    void run();
    void get_capture(int CameraID);

    int lower_H;
    int lower_S;
    int lower_V;

    int upper_H;
    int upper_S;
    int upper_V;

    void getLimits(int &lh, int &ls, int &lv, int &uh, int &us, int &uv);

    void blur_flip();
    void get_frame();

private:

    //integrers to hold the limits



    //Mat images to hold the frames

    cv::Mat original_frame;
    cv::Mat hsv_frame;
    cv::Mat binary_frame;

    //capture

    cv::VideoCapture capture;

    //threshold

    void threshold_smooth();


    //the scalars threshold

    cv::Scalar hsv_limits[2];

    //windows' names

    std::string win_name_1; //->threshold frame
    std::string win_name_2; //->original frame

    bool wasCreated;
};

#endif // LIMITER_H
