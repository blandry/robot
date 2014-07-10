cmake_minimum_required(VERSION 2.8.3)
project(kinect_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  image_transport
)

catkin_package()  #NUNCA ESQUEÇA ISSO POR FAVOR

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include/kinect_driver/
)

## Declare a cpp library
 add_library(kinect_driver src/${PROJECT_NAME}/kinect_driver.cpp)

 ##The main driver to ros
 add_library(driver src/${PROJECT_NAME}/driver.cpp)
 target_link_libraries(driver kinect_driver) 

## Declare a cpp executable
## BGR driver
 add_executable(bgr_driver_node src/bgr_driver_node.cpp )
 target_link_libraries(bgr_driver_node  driver opencv_core opencv_highgui ${catkin_LIBRARIES}) 

## Gray driver
 add_executable(gray_driver_node src/gray_driver_node.cpp )
 target_link_libraries(gray_driver_node driver opencv_core opencv_highgui ${catkin_LIBRARIES}) 

## Depth map driver
add_executable(depth_map_driver_node ./src/depth_map_driver_node.cpp)
target_link_libraries(depth_map_driver_node driver opencv_core opencv_highgui ${catkin_LIBRARIES})

## Detph mask driver
add_executable(valid_mask_driver_node ./src/valid_mask_driver_node.cpp)
target_link_libraries(valid_mask_driver_node driver opencv_core opencv_highgui ${catkin_LIBRARIES})
