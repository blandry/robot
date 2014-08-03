#ifndef ARDUINO_DRIVER_H
#define ARDUINO_DRIVER_H

#include <math.h>

#include "ros/ros.h" 
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

#include "CmdMessenger/CmdMessenger.h"

/*!
 * Anonymous enum that defines the commands used by the CmdMesseger
 * messaging system.
 */
enum
{
    kultra_nt,
    kultra_st,
    kultra_et,
    kultra_wt,
    kright_motor,
    kleft_motor
};

class ArduinoDriver
{
    public:

        //CmdMessenger

        /*!
         * Constructor. It creates the node handles and sets the necessary configurations.
         */
        ArduinoDriver();

        /*!
         * Dtor
         */
        ~ArduinoDriver();

        /*!
         * CmdMessenger callback that reads the distance given by the ultrassonic sensor
         * placed in the north position of the robot.
         *
         * \param command The received message from the microcontroller containing the calculated distance.
         */
        void ultrassonicNorthCallBack(cmd::CmdReceived& command);

        /*!
         * CmdMessenger callback that reads the distance given by the ultrassonic sensor
         * placed in the south position of the robot.
         *
         * \param command The received message from the microcontroller containging the calculated distance.
         */
        void ultrassonicSouthCallBack(cmd::CmdReceived& command);

        /*!
         * CmdMessenger callback that reads the distance given by the ultrassonic sensor
         * placed in the east position of the robot.
         *
         * \param command The received message from the microcontroller containing the calculated distance.
         */
        void ultrassonicEastCallBack(cmd::CmdReceived& command);

        /*!
         * CmdMessenger callback that reads the distance given by the ultrassonic sensor
         * placed in the west position of the robot.
         *
         * \param command The received message from the microcontroller containging the calculated distance.
         */
        void ultrassonicWestCallBack(cmd::CmdReceived& command);

        /*!
         * Opens the micro controller unit communication. It looks for a
         * private parameter called 'serial_port' and another called 'baudrate'. If the baudrate
         * private param is not set it will be set 9600 as default. If the serial port is not set
         * it will wait for the param to be set
         */
        void openMcuCommunication();

        /*!
         * Feeds in the serial data from the MCU and then reads and processes the messages.
         */
        void feedInSerialData();

        /*!
         * It subscribes to a topic containging a geometry_msgs::Twist.
         * Based on the received message, it will set the motors speed.
         *
         * \param velocity The gemoetry_msgs::Twist message defining a desired velocity
         * for the robot.
         */
        void velocityCallBack(const geometry_msgs::Twist& velocity);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_; //private node handle to set private parameters like the serial port name

        //These will publish ultrassonic data
        ros::Publisher pub_ultra_nt_;
        ros::Publisher pub_ultra_st_;
        ros::Publisher pub_ultra_et_;
        ros::Publisher pub_ultra_wt_;

        sensor_msgs::Range hc_sr04_; //the ultrassonic sensor message.

        ros::Subscriber sub_velocity_; //subscriber that controls the motors of the robot.

        //CmdMessenger
        cmd::CmdMessenger mcu_;

};

#endif



























