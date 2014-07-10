#include "opencv2/opencv.hpp"
#ifndef TRACKER_HPP
#define TRACKER_HPP

#define x_times 6

class tracker
{
public:

    tracker();
    ~tracker();

    void getCapture(int cameraID); //ok
    void getFrame(); //ok
    void setRanges(cv::Scalar lower_hsv, cv::Scalar upper_hsv);
    void track();
    void visual_track(); //ok
    //function to theshold the frame
    void threshold(); //ok
    int getQuadrant();

    int getXpos();
    int getYpos();

    bool getChangedYstate();
    bool getChangedXstate();

private:

    __inline void grid();

    //capture variable
    cv::VideoCapture webcam_capture;

    //Mat images to hold the frames
    cv::Mat original_frame;
    cv::Mat hsv_frame;
    cv::Mat Binary_frame;
    cv::Mat blurred_frame;
    cv::Mat temp;
    cv::Mat row;

    //Mat images to hold the quadrant rois
    cv::Mat roiUp;
    cv::Mat roiDown;
    cv::Mat roiLeft;
    cv::Mat roiRight;

    //variables to hold the coordinates of the object
    int current_XPos;
    int last_XPos;
    int current_Ypos;
    int last_Ypos;

    //variables to hold the HSV lower and higher limits
    int upper_H;
    int upper_S;
    int upper_V;

    int lower_H;
    int lower_S;
    int lower_V;

    //moments
    cv::Moments moments;

    //position control
    bool isUp();
    bool isDown();
    bool isLeft();
    bool isRight();

    int quadrant;

    //highlight
    __inline void highlightUp();
    __inline void highlightDown();
    __inline void highlightLeft();
    __inline void highlightRight();

    //contador
    int contador1;
    int contador2;
    int contador3;
    int contador4;

    //window name

    std::string window_name;

    bool wasCreated;

    bool wasChangedY;
    bool wasChangedX;
};

#endif // TRACKER_HPP
