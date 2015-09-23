cmake_minimum_required(VERSION 2.8.3)
project(squirrel_active_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             cv_bridge
             squirrel_object_perception_msgs
             v4r
             tf2
             tf
             pcl_ros
             std_msgs
             geometry_msgs
             sensor_msgs
             octomap_msgs
             octomap_ros
             nav_msgs
             message_generation
             actionlib
             actionlib_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ViewpointEntropy.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  EntropyMap.srv
  EntropyMapViz.srv
  CoveragePlan.srv
  CoveragePlanFile.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  octomap_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES squirrel_segmentation
 CATKIN_DEPENDS squirrel_object_perception_msgs v4r tf2 std_msgs sensor_msgs geometry_msgs octomap_msgs octomap_ros nav_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

ADD_LIBRARY(common OBJECT src/active_exploration.cpp src/active_exploration_utils.cpp src/pc_utils.cpp src/visualization_utils.cpp src/math_utils.cpp src/file_utils.cpp src/transform_utils.cpp src/io_utils.cpp src/octomap_utils.cpp src/ground_removal.cpp)

add_executable(squirrel_active_exploration src/squirrel_active_exploration.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(squirrel_active_exploration ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS} ${OCTOMAP_LIBRARIES})
add_dependencies(squirrel_active_exploration v4r)

add_executable(squirrel_run_with_dataset src/squirrel_run_with_dataset.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(squirrel_run_with_dataset ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS} ${OCTOMAP_LIBRARIES})
add_dependencies(squirrel_run_with_dataset v4r)

add_executable(squirrel_run_robot_experiment src/squirrel_run_robot_experiment.cpp src/robot_controller.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(squirrel_run_robot_experiment ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS} ${OCTOMAP_LIBRARIES})
add_dependencies(squirrel_run_robot_experiment v4r)

add_executable(fake_action_server src/fake_action_server.cpp)
target_link_libraries(fake_action_server ${catkin_LIBRARIES})
add_dependencies(fake_action_server v4r ${squirrel_object_perception_msgs_EXPORTED_TARGETS})

add_executable(robot_controller test/test_robot_controller.cpp src/robot_controller.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(robot_controller ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS} ${OCTOMAP_LIBRARIES})
add_dependencies(robot_controller v4r)

add_executable(test_octomap_utils test/test_octomap_utils.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(test_octomap_utils ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS} ${OCTOMAP_LIBRARIES})

add_executable(test_math_utils test/test_math_utils.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(test_math_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(save_segment_indices test/save_segment_indices.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(save_segment_indices ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS} ${OCTOMAP_LIBRARIES})
add_dependencies(save_segment_indices v4r)

add_executable(squirrel_entropy_map src/entropy_map.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(squirrel_entropy_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_entropy_map v4r)

add_executable(test_entropy_map test/test_entropy_map.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(test_entropy_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(test_entropy_map v4r)

add_executable(squirrel_map_coverage src/squirrel_map_coverage.cpp src/octomap_utils.cpp)
target_link_libraries(squirrel_map_coverage ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(squirrel_map_coverage_server src/squirrel_map_coverage_server.cpp src/octomap_utils.cpp)
target_link_libraries(squirrel_map_coverage_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(test_map_coverage_server test/test_map_coverage_server.cpp src/octomap_utils.cpp)
target_link_libraries(test_map_coverage_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(PointCloud2PCD src/PointCloud2PCD.cpp)
target_link_libraries(PointCloud2PCD ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pose_to_transform src/pose_to_transform.cpp src/math_utils.cpp src/file_utils.cpp)
target_link_libraries(pose_to_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(refine_transformations src/refine_transformations.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(refine_transformations ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(manual_adjust_point_cloud src/manual_adjust_point_cloud.cpp $<TARGET_OBJECTS:common>)
target_link_libraries(manual_adjust_point_cloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS squirrel_active_exploration squirrel_active_exploration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_active_exploration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
