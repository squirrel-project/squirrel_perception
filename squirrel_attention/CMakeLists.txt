cmake_minimum_required(VERSION 2.8.3)
project(squirrel_attention)

find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs geometry_msgs cv_bridge image_transport squirrel_object_perception_msgs)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(V4R REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS squirrel_object_perception_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

add_executable(attention_controller src/AttentionController.cpp)
add_executable(attention_motion src/AttentionMotion.cpp)
add_executable(attention_legs src/AttentionLegs.cpp)

target_link_libraries(attention_controller ${catkin_LIBRARIES})

target_link_libraries(attention_motion ${catkin_LIBRARIES})

target_link_libraries(attention_legs ${catkin_LIBRARIES})

find_package(rostest REQUIRED)
