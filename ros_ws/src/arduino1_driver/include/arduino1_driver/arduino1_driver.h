#ifndef ARDUINO1_DRIVER_H
#define ARDUINO1_DRIVER_H

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "CmdMessenger/CmdMessenger.h"

class ArduinoDriver
{
    public:
        ArduinoDriver();
        ~ArduinoDriver();

        void publishNt(const float& range);
        void publishSt(const float& range);
        void publishEt(const float& range);
        void publishWt(const float& range);

        void setNtPublisher(ros::Publisher publisher);

        void readCommands();

    private:
        cmd::CmdMessenger arduino_; //arduino low level access

        sensor_msgs::Range hc_sr04_; //Message to hold the range data.

        ros::Publisher pub_nt_;
        ros::Publisher pub_st_;
        ros::Publisher pub_et_;
        ros::Publisher pub_wt_;
};

#endif
