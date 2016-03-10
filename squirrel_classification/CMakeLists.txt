cmake_minimum_required(VERSION 2.8.3)
project(squirrel_classification)

find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs roscpp rospy sensor_msgs squirrel_object_perception_msgs std_msgs visualization_msgs)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)

find_package(V4R REQUIRED)
catkin_package(
  INCLUDE_DIRS include #  LIBRARIES squirrel_classification
 CATKIN_DEPENDS #  DEPENDS system_lib
 geometry_msgs sensor_msgs squirrel_object_perception_msgs std_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(squirrel_classification src/shape_simple_classifier_node.cpp)

target_link_libraries(squirrel_classification ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})

add_dependencies(squirrel_classification ${catkin_EXPORTED_TARGETS})
install(TARGETS squirrel_classification RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
IF(${V4R_FOUND})
	message(STATUS V4R found.)
	message(V4R_INCLUDE_DIRS: ${V4R_INCLUDE_DIRS})
ELSE()
	message(STATUS V4R not found)
ENDIF(${V4R_FOUND})
