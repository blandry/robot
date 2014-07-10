#include "driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bgr_driver_node");
    
    Driver driver("kinect/bgr/image_raw", 1, BGR);

    do
    {
        driver.openDevice();
    } while(!driver.isOpened());

    while(ros::ok())
    {
      driver.publish();
      driver.sleep();
    }

    return 0;
}
