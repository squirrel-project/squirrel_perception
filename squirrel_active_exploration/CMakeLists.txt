cmake_minimum_required(VERSION 2.8.3)
project(squirrel_active_exploration)

find_package(catkin REQUIRED COMPONENTS
             actionlib
             actionlib_msgs
             cv_bridge
             eigen_conversions
             geometry_msgs
             message_generation
             move_base
             move_base_msgs
             nav_msgs
             octomap_msgs
             octomap_ros
             pcl_conversions
             pcl_ros
             roscpp
             rospy
             sensor_msgs
             squirrel_object_perception_msgs
             std_msgs
             std_srvs
             tf
             tf2
             visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
 INCLUDE_DIRS include #  LIBRARIES squirrel_segmentation
 CATKIN_DEPENDS #  DEPENDS system_lib
 cv_bridge eigen_conversions geometry_msgs message_runtime move_base move_base_msgs nav_msgs octomap_msgs octomap_ros pcl_conversions pcl_ros sensor_msgs squirrel_object_perception_msgs std_msgs std_srvs tf tf2 visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

ADD_LIBRARY(squirrel_active_exploration_common OBJECT src/active_exploration.cpp src/active_exploration_utils.cpp src/pc_utils.cpp src/visualization_utils.cpp src/math_utils.cpp src/file_utils.cpp src/transform_utils.cpp src/io_utils.cpp src/octomap_utils.cpp src/ground_removal.cpp)
add_dependencies(squirrel_active_exploration_common ${catkin_EXPORTED_TARGETS})

#add_executable(squirrel_active_exploration src/squirrel_active_exploration.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(squirrel_active_exploration ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#add_dependencies(squirrel_active_exploration V4R)

add_executable(squirrel_run_with_dataset src/squirrel_run_with_dataset.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
target_link_libraries(squirrel_run_with_dataset ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(squirrel_run_with_dataset V4R)

add_executable(squirrel_run_robot_experiment src/squirrel_run_robot_experiment.cpp src/robot_controller.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
target_link_libraries(squirrel_run_robot_experiment ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(squirrel_run_robot_experiment V4R)

#add_executable(fake_action_server src/fake_action_server.cpp)
#target_link_libraries(fake_action_server ${catkin_LIBRARIES})
#add_dependencies(fake_action_server V4R ${squirrel_object_perception_msgs_EXPORTED_TARGETS})

#add_executable(robot_controller test/test_robot_controller.cpp src/robot_controller.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(robot_controller ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#add_dependencies(robot_controller V4R)

#add_executable(test_octomap_utils test/test_octomap_utils.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(test_octomap_utils ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

#add_executable(save_segment_indices test/save_segment_indices.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(save_segment_indices ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#add_dependencies(save_segment_indices V4R)

add_executable(squirrel_entropy_map src/entropy_map.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
target_link_libraries(squirrel_entropy_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})
add_dependencies(squirrel_entropy_map V4R)

#add_executable(test_entropy_map test/test_entropy_map.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(test_entropy_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})
#add_dependencies(test_entropy_map V4R)

#add_executable(squirrel_map_coverage src/squirrel_map_coverage.cpp src/octomap_utils.cpp)
#target_link_libraries(squirrel_map_coverage ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#add_dependencies(squirrel_map_coverage ${catkin_EXPORTED_TARGETS} ${squirrel_active_exploration_EXPORTED_TARGETS})

add_executable(squirrel_map_coverage_server src/squirrel_map_coverage_server.cpp src/octomap_utils.cpp)
target_link_libraries(squirrel_map_coverage_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(squirrel_map_coverage_server ${catkin_EXPORTED_TARGETS} ${squirrel_active_exploration_EXPORTED_TARGETS})

#add_executable(test_map_coverage_server test/test_map_coverage_server.cpp src/octomap_utils.cpp)
#target_link_libraries(test_map_coverage_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
#add_dependencies(test_map_coverage_server ${catkin_EXPORTED_TARGETS} ${squirrel_active_exploration_EXPORTED_TARGETS})

#add_executable(PointCloud2PCD src/PointCloud2PCD.cpp)
#target_link_libraries(PointCloud2PCD ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(PointCloud2PCD ${catkin_EXPORTED_TARGETS} ${squirrel_active_exploration_EXPORTED_TARGETS})

#add_executable(pose_to_transform src/pose_to_transform.cpp src/math_utils.cpp src/file_utils.cpp)
#target_link_libraries(pose_to_transform ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(pose_to_transform ${catkin_EXPORTED_TARGETS} ${squirrel_active_exploration_EXPORTED_TARGETS})

#add_executable(refine_transformations src/refine_transformations.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(refine_transformations ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(manual_adjust_point_cloud src/manual_adjust_point_cloud.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(manual_adjust_point_cloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(squirrel_active_exploration_server src/squirrel_active_exploration_server.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
target_link_libraries(squirrel_active_exploration_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

#add_executable(test_active_exploration_server test/test_active_exploration_server.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(test_active_exploration_server ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

#add_executable(test_active_exploration_server_robot test/test_active_exploration_server_robot.cpp $<TARGET_OBJECTS:squirrel_active_exploration_common>)
#target_link_libraries(test_active_exploration_server_robot ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

install(TARGETS squirrel_active_exploration_server squirrel_map_coverage_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
