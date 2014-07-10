cmake_minimum_required(VERSION 2.8.3)
project(arm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  message_generation
)

add_message_files(
  FILES
  coordinates.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(CATKIN_DEPENDS message_runtime)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include/arm/
) 

## Declare a cpp executable
# add_executable(arm_node src/arm_node.cpp)
add_executable(window src/window.cpp)
target_link_libraries(window opencv_core opencv_highgui opencv_imgproc ${catkin_LIBRARIES})

add_executable(windowT src/windowT.cpp)
target_link_libraries(windowT opencv_core opencv_highgui opencv_imgproc ${catkin_LIBRARIES})

add_executable(window_tracking src/window_tracking.cpp)
target_link_libraries(window_tracking opencv_core opencv_highgui opencv_imgproc ${catkin_LIBRARIES})

add_executable(limiter src/limiter_node1.cpp)
target_link_libraries(limiter opencv_core opencv_highgui opencv_imgproc ${catkin_LIBRARIES})

add_executable(tracker src/tracker_node.cpp)
target_link_libraries(tracker opencv_core opencv_highgui opencv_imgproc ${catkin_LIBRARIES})
