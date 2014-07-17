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
)

## Declare a cpp executable

########## Image Generator ##########

 # Image generator
add_executable(image_generator_node ./src/image_generator.cpp)
target_link_libraries(image_generator_node driver opencv_core opencv_highgui ${catkin_LIBRARIES})

########## Depth Generator ##########

# Depth generator
add_executable(depth_generator_node ./src/depth_generator.cpp)
target_link_libraries(depth_generator_node driver opencv_core opencv_highgui ${catkin_LIBRARIES}) 
