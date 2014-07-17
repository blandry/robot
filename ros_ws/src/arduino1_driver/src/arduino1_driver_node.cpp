#define _USE_MATH_DEFINES

#include <math.h> 

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include "CmdMessenger/CmdMessenger.h"

ros::Publisher pub_nt, pub_st, pub_et, pub_wt; 

sensor_msgs::Range hc_sr04_nt, hc_sr04_st, hc_sr04_et, hc_sr04_wt, hc_sr04;

/*!
 * Enum representing the commands (must be the same in arduino and here)
 */
enum
{
    kultra_nt,
    kultra_st,
    kultra_et,
    kultra_wt
};

/*!
 * Processes the kultra_nt command (The data from the ultrassonic sensor placed in the north side)
 *
 * \param command The received command.
 */
void ultra_nt_cb(cmd::CmdReceived& command);

/*!
 * Processes the kultra_st command  (The data from the ultrassonic sensor placed in the south side)
 *
 * \param command The received command.
 */
void ultra_st_cb(cmd::CmdReceived& command);

/*!
 * Processes the kultra_et command (The data from the ultrassonic sensor placed in the east side)
 *
 * \param command The received command.
 */
void ultra_et_cb(cmd::CmdReceived& command);

/*!
 * Processes the kultra_wt command (The data from the ultrassonic sensor placed in the west side)
 *
 * \param command the received command.
 */
void ultra_wt_cb(cmd::CmdReceived& command);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "arduino1_driver"); 
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    pub_nt = nh.advertise<sensor_msgs::Range>("ultrassonic/north", 1);
    pub_st = nh.advertise<sensor_msgs::Range>("ultrassonic/south", 1);
    pub_et = nh.advertise<sensor_msgs::Range>("ultrassonic/east", 1);
    pub_wt = nh.advertise<sensor_msgs::Range>("ultrassonic/west", 1);

    hc_sr04.radiation_type = sensor_msgs::Range::ULTRASOUND;
    hc_sr04.field_of_view = M_PI/12;
    hc_sr04.min_range = 0.020;
    hc_sr04.max_range = 4.00;

    hc_sr04_nt = hc_sr04;
    hc_sr04_st = hc_sr04;
    hc_sr04_et = hc_sr04;
    hc_sr04_wt = hc_sr04;

    std::string port; //to hold the port name
    int baudrate; //to hold the baudrate

//    port = "/dev/ttyACM0";
//    baudrate = 9600;

    nh_private.param<std::string>("serial_port", port, "/dev/ttyACM0");
    nh_private.param<int>("baudrate", baudrate, 9600);
    cmd::CmdMessenger arduino(port, baudrate);

    if(arduino.isOpen()){ //if the port is opened, print some information
        ROS_INFO("Opened device in the port: %s", arduino.getPort().c_str());
        ROS_INFO("Baudrate: %d", arduino.getBaudrate());
    }
    else
    {//if the port is not opened, keep sending message and trying to open it
        while(!arduino.isOpen())
        {
            ROS_ERROR("Port not opened");
            arduino.open();
            ros::Duration(0.5).sleep(); 
        }
    } 

    arduino.attach(kultra_nt, ultra_nt_cb);
    arduino.attach(kultra_st, ultra_st_cb);

    while(ros::ok())
    {
        arduino.feedInSerialData();
        ros::spinOnce();
    }
    
    return 0;
}

void ultra_nt_cb(cmd::CmdReceived& command)
{
    float dist = (float) command.parseInt();
    hc_sr04_nt.range = dist/100;
    pub_nt.publish(hc_sr04_nt);
} 

void ultra_st_cb(cmd::CmdReceived& command)
{
    float dist = (float) command.parseInt();
    hc_sr04_st.range = dist/100;
    pub_st.publish(hc_sr04_st);
}

void ultra_et_cb(cmd::CmdReceived& command)
{
    int dist = command.parseInt();
    hc_sr04_et.range = (float)dist/100;
    pub_et.publish(hc_sr04_et);
}

void ultra_wt_cb(cmd::CmdReceived& command)
{
    int dist = command.parseInt();
    hc_sr04_wt.range = (float)dist/100;
    pub_wt.publish(hc_sr04_wt);
} 
