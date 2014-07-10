#include "tracker.hpp"

tracker::tracker()
{
    tracker::wasCreated = false;
    tracker::wasChangedX = false;
    tracker::wasChangedY = false;

    tracker::contador1 = 0;
    tracker::contador2 = 0;
    tracker::contador3 = 0;
    tracker::contador4 = 0;

    tracker::window_name = "tracking";
}

tracker::~tracker()
{
    if(wasCreated) cv::destroyWindow(tracker::window_name);
}

void tracker::getCapture(int cameraID)
{
    tracker::webcam_capture.open(cameraID);
}

void tracker::getFrame()
{

    tracker::webcam_capture >> tracker::original_frame;
    cv::flip(tracker::original_frame, tracker::original_frame, 1);
    tracker::original_frame.copyTo(tracker::temp);
    tracker::grid();
    tracker::row = cv::imread("/home/edno/projetos/connepi/seta.png");
    cv::add(tracker::temp, tracker::row, tracker::temp);

    cv::blur(tracker::original_frame, tracker::blurred_frame, cv::Size(5,5));

    cv::cvtColor(tracker::blurred_frame, tracker::hsv_frame, CV_BGR2HSV);
    cv::blur(tracker::hsv_frame, tracker::hsv_frame, cv::Size(5,5));

    /**SETTING ROIS*/

    tracker::roiUp    = tracker::temp(cv::Rect(213,0  ,213,160));
    tracker::roiDown  = tracker::temp(cv::Rect(213,320,213,160));
    tracker::roiLeft  = tracker::temp(cv::Rect(0  ,160,213,160));
    tracker::roiRight = tracker::temp(cv::Rect(426,160,213,160));

}

void tracker::threshold()
{
    cv::inRange(tracker::hsv_frame, cv::Scalar(tracker::lower_H, tracker::lower_S, tracker::lower_V), cv::Scalar(tracker::upper_H, tracker::upper_S, tracker::upper_V), tracker::Binary_frame);
}

void tracker::grid()
{
    //horizontal lines
    cv::line(tracker::temp, cv::Point(0,160), cv::Point(640,160), cv::Scalar(0,0,0));
    cv::line(tracker::temp, cv::Point(0,320), cv::Point(640,320), cv::Scalar(0,0,0));

    //vertical lines
    cv::line(tracker::temp, cv::Point(213,0), cv::Point(213,480), cv::Scalar(0,0,0));
    cv::line(tracker::temp, cv::Point(426,0), cv::Point(426,480), cv::Scalar(0,0,0));
}

bool tracker::isUp()
{
    if( (tracker::current_XPos > 213) && (tracker::current_XPos < 426) && (tracker::current_Ypos < 160) ) return true;

    else return false;
}

bool tracker::isDown()
{
    if( (tracker::current_XPos > 213) && (tracker::current_XPos < 426) && (tracker::current_Ypos > 320) ) return true;
    else return false;
}

bool tracker::isLeft()
{
    if( (tracker::current_XPos < 213) && (tracker::current_Ypos > 160) && (tracker::current_Ypos < 320) ) return true;

    else return false;
}

bool tracker::isRight()
{
    if( (tracker::current_XPos > 426) && (tracker::current_Ypos > 160) && (tracker::current_Ypos < 320) ) return true;

    else return false;
}

void tracker::track()
{
    if(tracker::isUp()){

        tracker::contador1++;

        tracker::contador2 = 0;
        tracker::contador3 = 0;
        tracker::contador4 = 0;

        tracker::quadrant = 1;

    }else if(tracker::isDown()){

        tracker::contador2++;

        tracker::contador1 = 0;
        tracker::contador3 = 0;
        tracker::contador4 = 0;

        tracker::quadrant = 2;

    }else if(tracker::isLeft()){

        tracker::contador3++;

        tracker::contador1 = 0;
        tracker::contador2 = 0;
        tracker::contador4 = 0;

        tracker::quadrant = 3;

    }else if(tracker::isRight()){

        tracker::contador4++;

        tracker::contador1 = 0;
        tracker::contador2 = 0;
        tracker::contador3 = 0;

        tracker::quadrant = 4;

    }else{

        tracker::contador1 = 0;
        tracker::contador2 = 0;
        tracker::contador3 = 0;
        tracker::contador4 = 0;

        tracker::quadrant = -1;
    }
}

void tracker::highlightUp()
{

    cv::add(tracker::roiUp, 60, tracker::roiUp);
}

void tracker::highlightDown()
{

    cv::add(tracker::roiDown, 60, tracker::roiDown);
}

void tracker::highlightLeft()
{

    cv::add(tracker::roiLeft, 60, tracker::roiLeft);
}

void tracker::highlightRight()
{

    cv::add(tracker::roiRight, 60, tracker::roiRight);
}

void tracker::setRanges(cv::Scalar lower_hsv, cv::Scalar upper_hsv)
{
    tracker::lower_H = lower_hsv[0];
    tracker::lower_S = lower_hsv[1];
    tracker::lower_V = lower_hsv[2];

    tracker::upper_H = upper_hsv[0];
    tracker::upper_S = upper_hsv[1];
    tracker::upper_V = upper_hsv[2];
}

void tracker::visual_track()
{
    cv::namedWindow(tracker::window_name);
    tracker::wasCreated = true;

    tracker::moments = cv::moments(tracker::Binary_frame, true);

    double moment10 = tracker::moments.m10;
    double moment01 = tracker::moments.m01;
    double area = tracker::moments.m00;

    if(area < 1000){

        tracker::last_XPos = -1;
        tracker::last_Ypos = -1;
        tracker::current_XPos = -1;
        tracker::current_Ypos = -1;
    }

    else {

        tracker::current_XPos = moment10/area;
        tracker::current_Ypos = moment01/area;

        //tracker::quadrant = tracker::track();
    }

    if(tracker::current_XPos >=0 && tracker::current_Ypos >=0 && tracker::last_XPos >=0 && tracker::last_Ypos >=0){

        cv::line(tracker::temp, cv::Point(tracker::current_XPos, tracker::current_Ypos), cv::Point(tracker::last_XPos, tracker::last_Ypos), cv::Scalar(0, 0, 255), 4);
    }

    if(tracker::last_Ypos == tracker::current_Ypos) tracker::wasChangedY = false;
    else tracker::wasChangedY = true;

    if(tracker::last_XPos == tracker::current_XPos) tracker::wasChangedX = false;
    else tracker::wasChangedX = true;

    tracker::last_XPos = tracker::current_XPos;
    tracker::last_Ypos = tracker::current_Ypos;

    //highlight

    tracker::track();

    switch(tracker::quadrant)
    {
    case 1:
        if(contador1 > x_times) tracker::highlightUp();
        break;

    case 2:
        if(contador2 > x_times) tracker::highlightDown();
        break;

    case 3:
        if(contador3 > x_times) tracker::highlightLeft();
        break;

    case 4:
        if(contador4 > x_times) tracker::highlightRight();
        break;
    }

    cv::imshow(tracker::window_name, tracker::temp);

}

int tracker::getQuadrant()
{
    return tracker::quadrant;
}

int tracker::getXpos()
{
    return tracker::current_XPos;
}

int tracker::getYpos()
{
    return tracker::current_Ypos;
}

bool tracker::getChangedYstate()
{
    return tracker::wasChangedY;
}

bool tracker::getChangedXstate()
{
    return tracker::wasChangedX;
}
