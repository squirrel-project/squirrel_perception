cmake_minimum_required(VERSION 2.8.3)
project(squirrel_tracking)

find_package(catkin REQUIRED)
find_package(V4R REQUIRED)
find_package(catkin REQUIRED COMPONENTS cv_bridge gazebo_msgs image_transport pcl_conversions roscpp rospy sensor_msgs squirrel_object_perception_msgs tf tf2 mongodb_store)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)

add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)
#find_package(Boost 1.54.0 EXACT REQUIRED COMPONENTS thread program_options serialization regex filesystem system)

catkin_package(
 INCLUDE_DIRS include #  LIBRARIES squirrel_tracking
 CATKIN_DEPENDS #  DEPENDS system_lib
 cv_bridge gazebo_msgs image_transport pcl_conversions rospy sensor_msgs squirrel_object_perception_msgs tf tf2 mongodb_store
)

include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(object_tracker src/object_tracker.cpp)
add_executable(lump_tracker src/lump_tracker.cpp)

target_link_libraries(object_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS} ceres glog cholmod amd klu btf spqr camd ccolamd cxsparse umfpack colamd ldl csparse blas lapack boost_thread boost_program_options boost_serialization boost_regex boost_filesystem boost_system)
add_dependencies(object_tracker V4R ${catkin_EXPORTED_TARGETS})

target_link_libraries(lump_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})

add_dependencies(lump_tracker ${catkin_EXPORTED_TARGETS})
install(TARGETS object_tracker lump_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
catkin_install_python(PROGRAMS scripts/fake_simulation_tracker.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
