#include "driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "valid_mask_driver_node");
    
    Driver driver("kinect/depth/valid_depth_mask", 1, VALID_DEPTH_MASK);

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
