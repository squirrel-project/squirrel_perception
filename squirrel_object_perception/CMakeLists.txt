cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_perception)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        dynamic_reconfigure
        geometry_msgs
        pcl_conversions
        roscpp
        roslib
        rospy
        sensor_msgs
        squirrel_object_perception_msgs
        squirrel_planning_knowledge_msgs
        std_msgs
        std_srvs
        tf
        tf2
)
find_package(PCL 1.7 REQUIRED)

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
catkin_python_setup()

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/squirrel_look_for_objects.cfg)

catkin_package(
  INCLUDE_DIRS #   include 
 CATKIN_DEPENDS
        actionlib
        actionlib_msgs
        dynamic_reconfigure
        geometry_msgs
        pcl_conversions
        roscpp
        roslib
        rospy
        sensor_msgs
        squirrel_object_perception_msgs
        squirrel_planning_knowledge_msgs
        std_msgs
        std_srvs
        tf
        tf2
)

#include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(squirrel_look_for_objects src/squirrel_look_for_objects_server.cpp)

#add_dependencies(squirrel_look_for_objects squirrel_look_for_objects_gencfg)
add_dependencies(squirrel_look_for_objects ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(squirrel_look_for_objects
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
catkin_install_python(PROGRAMS src/squirrel_object_perception/squirrel_look_for_objects.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})