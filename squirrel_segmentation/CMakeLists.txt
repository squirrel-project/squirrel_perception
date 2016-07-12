cmake_minimum_required(VERSION 2.8.3)
project(squirrel_segmentation)

find_package(catkin REQUIRED COMPONENTS cv_bridge pcl_conversions pcl_ros roscpp rospy sensor_msgs squirrel_object_perception_msgs std_msgs tf tf2 visualization_msgs)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(V4R REQUIRED)

#target_link_libraries(squirrel_segmentation_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})
#add_dependencies(squirrel_segmentation_test )

find_package(rostest REQUIRED)

catkin_package(
 INCLUDE_DIRS include #  LIBRARIES squirrel_segmentation
 CATKIN_DEPENDS #  DEPENDS system_lib
 cv_bridge pcl_conversions pcl_ros sensor_msgs squirrel_object_perception_msgs std_msgs tf tf2 visualization_msgs 
)

#message(STATUS "MY PCL_INCLUDE_DIRS" ${PCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

#message(STATUS "MY V4R_LIBS" ${V4R_LIBS})
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "MY ${_variableName}=${${_variableName}}")
# endforeach()
# link_directories(${V4R_LIB_DIR})

#add_executable(squirrel_segmentation_incremental src/squirrel_segmentation_incremental.cpp)
#add_executable(squirrel_segmentation_visualization src/squirrel_segmentation_visualization.cpp)
#add_executable(squirrel_segmentation src/squirrel_segmentation.cpp)
add_executable(squirrel_segmentation_popout src/squirrel_segmentation_popout.cpp)
##add_executable(squirrel_segmentation_test src/test.cpp)

#target_link_libraries(squirrel_segmentation_incremental ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})
#add_dependencies(squirrel_segmentation_incremental )
#
#target_link_libraries(squirrel_segmentation_visualization ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS})
#add_dependencies(squirrel_segmentation_visualization )
#
#target_link_libraries(squirrel_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${V4R_LIBS}
#cholmod
#amd
#klu
#btf
#spqr
#camd
#ccolamd
#cxsparse
#umfpack
#colamd
#ldl
#csparse
#)
#add_dependencies(squirrel_segmentation )
#

target_link_libraries(squirrel_segmentation_popout ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${V4R_LIBS})
add_dependencies(squirrel_segmentation_popout ${catkin_EXPORTED_TARGETS})
  
# find_package(gtest)

# Disables tests as they break on the new buildbot
#if(CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(squirrel_segmentation_test test/squirrel_segmentation_test.cpp)
#  catkin_add_gtest(squirrel_segmentation_incremental_test test/squirrel_segmentation_incremental_test.cpp)
#  if(TARGET squirrel_segmentation_incremental_test)
#    target_link_libraries(squirrel_segmentation_incremental_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
#  endif()
#  if(TARGET squirrel_segmentation_test)
#    target_link_libraries(squirrel_segmentation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
#  endif()
#install(TARGETS squirrel_segmentation_popout RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
#SET(CMAKE_BUILD_TYPE Release)

#rosbuild_add_gtest_build_flags(squirrel_segmentation_incremental_test)
# add_rostest(test/squirrel_segmentation_incremental_test.test)

# catkin_add_gtest(squirrel_segmentation_incremental_test test/squirrel_segmentation_incremental_test.cpp) # WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
# if(TARGET squirrel_segmentation_incremental_test)
#   target_link_libraries(squirrel_segmentation_incremental_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})# ${V4R_LIBS}
# 
#  add_rostest(test/squirrel_segmentation_incremental_test.test)
#  add_rostest(test/squirrel_segmentation_test.test)
#
#endif()
