cmake_minimum_required(VERSION 3.0.2)
project(vision_matrice100)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-std=gnu++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

add_message_files(
	FILES
	DurationStamped.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(aruco_detection_node src/aruco_detection.cpp)
target_link_libraries(aruco_detection_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_executable(rosbag_rotate_node src/rosbag_rotate.cpp)
target_link_libraries(rosbag_rotate_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_executable(image_filter_node src/feature_extraction.cpp)
target_link_libraries(image_filter_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(f_extraction_depth_channel_node src/f_extraction_depth_channel.cpp)
target_link_libraries(f_extraction_depth_channel_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(vs_sn_node src/visual_servo_space_null.cpp)
target_link_libraries(vs_sn_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(imu_gravity_node src/add_gravity_IMU.cpp)
target_link_libraries(imu_gravity_node
   ${catkin_LIBRARIES}
)
add_executable(lidar_to_dron_node src/lidar_to_dron.cpp)
target_link_libraries(lidar_to_dron_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}

)


