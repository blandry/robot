#include "ros/ros.h"
#include "ArduinoDriver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "arduino_driver");

    ArduinoDriver arduino;
    arduino.openMcuCommunication();

    while(ros::ok())
    {
        arduino.feedInSerialData();
        ros::spinOnce();
    }

    return 0;
}
