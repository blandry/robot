#include "limiter.hpp"

int lower_H = 0;
int lower_S = 0;
int lower_V = 0;

int upper_H = 180;
int upper_S = 255;
int upper_V = 255;

limiter::limiter()
{
    limiter::lower_H = 0;
    limiter::lower_S = 0;
    limiter::lower_V = 0;

    limiter::upper_H = 180;
    limiter::upper_S = 255;
    limiter::upper_V = 255;

    limiter::win_name_1 = "webcam";
    limiter::win_name_2 = "threshold";

    limiter::wasCreated = true;
}

limiter::~limiter()
{
    if(limiter::original_frame.data) limiter::original_frame.release();
    if(limiter::hsv_frame.data) limiter::hsv_frame.release();

    if(limiter::wasCreated){

        cv::destroyWindow(limiter::win_name_1);
        cv::destroyWindow(limiter::win_name_2);

    }

    if(limiter::capture.isOpened()) limiter::capture.release();
}

void limiter::setWindowParams()
{


    cv::namedWindow(limiter::win_name_1);
    cv::namedWindow(limiter::win_name_2);
    limiter::wasCreated = true;

    cv::createTrackbar("lower hue limit", limiter::win_name_2, &lower_H, 180);
    cv::createTrackbar("lower saturate limit", limiter::win_name_2, &lower_S, 255);
    cv::createTrackbar("lower value limit", limiter::win_name_2, &lower_V, 255);

    cv::createTrackbar("upper hue limit", limiter::win_name_2, &upper_H, 180);
    cv::createTrackbar("upper saturate limit", limiter::win_name_2, &upper_S, 255);
    cv::createTrackbar("upper value limit", limiter::win_name_2, &upper_V, 255);

}

void limiter::threshold_smooth()
{
    if(limiter::hsv_frame.data){

        cv::inRange(limiter::hsv_frame, cv::Scalar(lower_H,lower_S,lower_V), cv::Scalar(upper_H,upper_S,upper_V), limiter::binary_frame);

    }else std::cout << "error";
}

void limiter::get_capture(int CameraID)
{
    limiter::capture.open(CameraID);

    if(!limiter::capture.isOpened()) exit(0);
}

void limiter::get_frame()
{
    //limiter::capture.open(0);

    limiter::capture >> limiter::original_frame;

    cv::flip(limiter::original_frame, limiter::original_frame, 1);
}

void limiter::blur_flip()
{
    cv::blur(limiter::original_frame, limiter::original_frame, cv::Size(5,5));
    cv::cvtColor(limiter::original_frame, limiter::hsv_frame, CV_BGR2HSV);
    cv::blur(limiter::hsv_frame, limiter::hsv_frame, cv::Size(5,5));
}

void limiter::run()
{

    while(1)
    {
        limiter::get_frame();
        limiter::blur_flip();

        limiter::threshold_smooth();

        cv::imshow(limiter::win_name_1, limiter::original_frame);
        cv::imshow(limiter::win_name_2, limiter::binary_frame);

        if(cv::waitKey(30) == 27) break;
    }
}

void limiter::getLimits(int &lh, int &ls, int &lv, int &uh, int &us, int &uv)
{
    lh = lower_H;
    ls = lower_S;
    lv = lower_V;

    uh = upper_H;
    us = upper_S;
    uv = upper_V;
}
