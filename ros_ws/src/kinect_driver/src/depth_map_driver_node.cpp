#include "driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "depth_map_driver_node");
    
    Driver driver("kinect/depth/depth_map", 1, DEPTH_MAP);

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
