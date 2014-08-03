#include "ArduinoDriver.h"

/**********CTOR | DTOR**********/

ArduinoDriver::ArduinoDriver()
    :nh_(),
    nh_private_("~")
{
    //publishers
    pub_ultra_nt_ = nh_.advertise<sensor_msgs::Range>("ultrassonic/north", 1);
    pub_ultra_st_ = nh_.advertise<sensor_msgs::Range>("ultrassonic/south", 1);
    pub_ultra_et_ = nh_.advertise<sensor_msgs::Range>("ultrassonic/east", 1);
    pub_ultra_wt_ = nh_.advertise<sensor_msgs::Range>("ultrassonic/west", 1);

    //subscriber
    sub_velocity_ = nh_.subscribe("/robot/cmd_vel", 1, &ArduinoDriver::velocityCallBack, this);

    //ultrassonic sensor
    hc_sr04_.radiation_type = sensor_msgs::Range::ULTRASOUND;
    hc_sr04_.field_of_view = M_PI/12;
    hc_sr04_.min_range = 0.02;
    hc_sr04_.max_range = 4.00;

    //micro controller unit
    mcu_.attach(kultra_nt, &ArduinoDriver::ultrassonicNorthCallBack, *this);
    mcu_.attach(kultra_st, &ArduinoDriver::ultrassonicSouthCallBack, *this);
    mcu_.attach(kultra_et, &ArduinoDriver::ultrassonicEastCallBack, *this);
    mcu_.attach(kultra_wt, &ArduinoDriver::ultrassonicWestCallBack, *this);
}

//empty dtor
ArduinoDriver::~ArduinoDriver(){}

/**********CMD MESSENGER***********/

//Callbacks
void ArduinoDriver::ultrassonicNorthCallBack(cmd::CmdReceived& command)
{
    sensor_msgs::Range hc_sr04_nt = hc_sr04_;
    int dist = command.parseInt(); //the data is read in centimeters.
    hc_sr04_nt.range = (float)dist/100; //the published range has to be in meters, so we convert it.  
    hc_sr04_nt.header.stamp = ros::Time::now();
    pub_ultra_nt_.publish(hc_sr04_nt);
}

void ArduinoDriver::ultrassonicSouthCallBack(cmd::CmdReceived& command)
{
    sensor_msgs::Range hc_sr04_st = hc_sr04_;
    int dist = command.parseInt(); //the data is read in centimeters.
    hc_sr04_st.range = (float)dist/100; //the published range has to be in meters, so we convert it.
    hc_sr04_st.header.stamp = ros::Time::now();
    pub_ultra_st_.publish(hc_sr04_st);
}

void ArduinoDriver::ultrassonicEastCallBack(cmd::CmdReceived& command)
{
    sensor_msgs::Range hc_sr04_et = hc_sr04_;
    int dist = command.parseInt(); //the data is read in centimeters.
    hc_sr04_et.range = (float)dist/100; //the published range has to be in meters, so we convert it.
    hc_sr04_et.header.stamp = ros::Time::now();
    pub_ultra_et_.publish(hc_sr04_et); 
}

void ArduinoDriver::ultrassonicWestCallBack(cmd::CmdReceived& command)
{
    sensor_msgs::Range hc_sr04_wt = hc_sr04_;
    int dist = command.parseInt(); //the data is read in centimeters.
    hc_sr04_wt.range = (float)dist/100; //the published range has to be in meters, so we convert it.
    hc_sr04_wt.header.stamp = ros::Time::now();
    pub_ultra_wt_.publish(hc_sr04_wt);
}

void ArduinoDriver::openMcuCommunication()
{
    int baudrate; //serial communication baudrate, default is 9600 bits/sec
    std::string serial_port("");

    nh_private_.param<int>("baudrate", baudrate, 9600); //checks the parameter, and if not found sets the default
    if(baudrate < 0) //check the validity of the parameter set.
    {
        ROS_WARN("Invalid baudrate parameter value... Setting to its default value 9600");
        mcu_.setBaudRate(9600); //sets the baudrate to its default value
    }
    else
    {
        mcu_.setBaudRate(baudrate); //sets the baudrate.
    }

    while(!nh_private_.getParam("serial_port", serial_port)) //check if the serial port name was set
    {
        //while not set, wait for it to be.
        ROS_ERROR("Serial port name not set. Please, set the private parameter 'serial_port'");
        ros::Duration(0.5).sleep();
    }

    mcu_.setPort(serial_port); //sets the serial port
    mcu_.open(); //opens the serial communication.

    if(mcu_.isOpen()){
        ROS_INFO("Opened device in port: %s", mcu_.getPort().c_str());
        ROS_INFO("Baudrate: %d", mcu_.getBaudrate());
    }
}

void ArduinoDriver::feedInSerialData()
{
    mcu_.feedInSerialData();
}

/**********ROS**********/

void ArduinoDriver::velocityCallBack(const geometry_msgs::Twist& velocity)
{

}
