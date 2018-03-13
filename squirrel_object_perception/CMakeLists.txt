cmake_minimum_required(VERSION 2.8.3)
project(squirrel_object_perception)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        cv_bridge
        dynamic_reconfigure
        geometry_msgs
        mongodb_store
        nav_msgs
        octomap_msgs
        pcl_conversions
	pcl_ros
        robotino_msgs
        roscpp
        rospy
        sensor_msgs
        squirrel_object_perception_msgs
        squirrel_planning_knowledge_msgs
        std_msgs
        std_srvs
        tf
        tf2
        visualization_msgs
	v4r_object_recognition_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV 2.4 REQUIRED)

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
catkin_python_setup()

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/squirrel_look_for_objects.cfg)

catkin_package(
  INCLUDE_DIRS include CATKIN_DEPENDS
        actionlib
        actionlib_msgs
        cv_bridge
        dynamic_reconfigure
        geometry_msgs
        mongodb_store
        nav_msgs
        octomap_msgs
        pcl_conversions
	pcl_ros
        robotino_msgs
        roscpp
        rospy
        sensor_msgs
        squirrel_object_perception_msgs
        squirrel_planning_knowledge_msgs
        std_msgs
        std_srvs
        tf
        tf2
        visualization_msgs
	v4r_object_recognition_msgs
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

add_executable(squirrel_look_for_objects src/squirrel_look_for_objects_server.cpp)
add_executable(squirrel_call_haf_grasping src/call_haf_grasping.cpp)
add_executable(squirrel_find_dynamic_objects src/squirrel_find_dynamic_objects_server.cpp src/OctomapLib.cpp)
add_executable(squirrel_look_for_objects_in_hand src/squirrel_look_for_objects_in_hand.cpp)
add_executable(squirrel_recognize_objects src/squirrel_recognize_objects.cpp)
#add_executable(squirrel_dilate_octomap src/octomap_test.cpp src/OctomapLib.cpp)
add_executable(track_robot src/RobotTracker.cpp)
add_executable(squirrel_find_lumps_via_waypoints src/squirrel_find_lumps_via_waypoints.cpp)
add_executable(squirrel_check_grasp_success src/squirrel_check_grasp_success.cpp)
add_executable(squirrel_post_process_static_octomap src/post_process_static_octomap.cpp src/OctomapLib.cpp)
add_executable(squirrel_fuse_lumps_static_octomap src/squirrel_fuse_lumps_static_octomap.cpp src/OctomapLib.cpp)


#add_dependencies(squirrel_look_for_objects squirrel_look_for_objects_gencfg)
add_dependencies(squirrel_look_for_objects ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_find_dynamic_objects ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_look_for_objects_in_hand ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_recognize_objects ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(squirrel_dilate_octomap ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(track_robot ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_find_lumps_via_waypoints ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_check_grasp_success ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_post_process_static_octomap ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(squirrel_fuse_lumps_static_octomap ${squirrel_object_perception_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(squirrel_call_haf_grasping
  ${catkin_LIBRARIES}
)
target_link_libraries(squirrel_look_for_objects
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(squirrel_look_for_objects_in_hand
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(squirrel_recognize_objects
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(squirrel_find_dynamic_objects 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES})

target_link_libraries(squirrel_find_lumps_via_waypoints 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES})

target_link_libraries(squirrel_check_grasp_success
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES})

target_link_libraries(squirrel_post_process_static_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES})

target_link_libraries(squirrel_fuse_lumps_static_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES})

#target_link_libraries(squirrel_dilate_octomap ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
target_link_libraries(track_robot ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS squirrel_look_for_objects squirrel_find_dynamic_objects RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS src/squirrel_object_perception/squirrel_look_for_objects.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
