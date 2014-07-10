#include "driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gray_driver_node");
    
    Driver driver("kinect/gray/image_raw", 1, GRAY);

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
