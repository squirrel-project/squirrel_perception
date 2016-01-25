cmake_minimum_required(VERSION 2.8.3)
project(squirrel_attention)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamixel_msgs geometry_msgs image_transport roscpp rospy sensor_msgs squirrel_object_perception_msgs std_msgs) #pcl_conversions

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
find_package(V4R REQUIRED)

#target_link_libraries(squirrel_attention_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
#add_dependencies(squirrel_attention_test v4r)

####find_package(rostest REQUIRED)

catkin_package(
 INCLUDE_DIRS include #  LIBRARIES squirrel_attention
 CATKIN_DEPENDS #  DEPENDS system_lib
 cv_bridge dynamixel_msgs image_transport sensor_msgs squirrel_object_perception_msgs std_msgs
)

#message(STATUS "MY PCL_INCLUDE_DIRS" ${PCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

#message(STATUS "MY V4R_LIBS" ${V4R_LIBS})
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "MY ${_variableName}=${${_variableName}}")
# endforeach()
# link_directories(${V4R_LIB_DIR})

# add_executable(squirrel_attention_location src/squirrel_attention_location.cpp)
# add_executable(squirrel_attention_color src/squirrel_attention_color.cpp)
# add_executable(squirrel_attention_itti src/squirrel_attention_itti.cpp)
# add_executable(squirrel_attention_symmetry src/squirrel_attention_symmetry.cpp)
# add_executable(squirrel_attention_3Dsymmetry src/squirrel_attention_3Dsymmetry.cpp)
# add_executable(squirrel_attention_height src/squirrel_attention_height.cpp)
# add_executable(squirrel_attention_surfaceorientation src/squirrel_attention_surfaceorientation.cpp)
add_executable(attention_controller src/AttentionController.cpp)
add_executable(attention_motion src/AttentionMotion.cpp)
add_executable(attention_legs src/AttentionLegs.cpp)

#add_executable(squirrel_attention_test src/test.cpp)

# target_link_libraries(squirrel_attention_location ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_location v4r)
# 
# target_link_libraries(squirrel_attention_color ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_color v4r)
# 
# target_link_libraries(squirrel_attention_itti ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_itti v4r)
# 
# target_link_libraries(squirrel_attention_symmetry ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_symmetry v4r)
# 
# target_link_libraries(squirrel_attention_3Dsymmetry ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_3Dsymmetry v4r)
# 
# target_link_libraries(squirrel_attention_height ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_height v4r)
# 
# target_link_libraries(squirrel_attention_surfaceorientation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
# add_dependencies(squirrel_attention_surfaceorientation v4r)

target_link_libraries(attention_controller ${catkin_LIBRARIES})

target_link_libraries(attention_motion ${catkin_LIBRARIES})

target_link_libraries(attention_legs ${catkin_LIBRARIES})

add_dependencies(attention_controller ${catkin_EXPORTED_TARGETS})
add_dependencies(attention_motion ${catkin_EXPORTED_TARGETS})
add_dependencies(attention_legs ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(squirrel_attention_color_test test/squirrel_attention_color_test.cpp)

  catkin_add_gtest(squirrel_attention_itti_test test/squirrel_attention_itti_test.cpp)
  
  catkin_add_gtest(squirrel_attention_symmetry_test test/squirrel_attention_symmetry_test.cpp)
  
  catkin_add_gtest(squirrel_attention_3Dsymmetry_test test/squirrel_attention_3Dsymmetry_test.cpp)
  
  catkin_add_gtest(squirrel_attention_height_test test/squirrel_attention_height_test.cpp)
  
  catkin_add_gtest(squirrel_attention_surfaceorientation_test test/squirrel_attention_surfaceorientation_test.cpp)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(squirrel_attention_location_test test/squirrel_attention_location_test.cpp)
  if(TARGET squirrel_attention_location_test)
    target_link_libraries(squirrel_attention_location_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  if(TARGET squirrel_attention_color_test)
    target_link_libraries(squirrel_attention_color_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  if(TARGET squirrel_attention_itti_test)
    target_link_libraries(squirrel_attention_itti_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  if(TARGET squirrel_attention_symmetry_test)
    target_link_libraries(squirrel_attention_symmetry_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  if(TARGET squirrel_attention_3Dsymmetry_test)
    target_link_libraries(squirrel_attention_3Dsymmetry_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  if(TARGET squirrel_attention_height_test)
    target_link_libraries(squirrel_attention_height_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  if(TARGET squirrel_attention_surfaceorientation_test)
    target_link_libraries(squirrel_attention_surfaceorientation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
install(TARGETS attention_controller attention_motion attention_legs RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  add_rostest(test/squirrel_attention_location_test.test)
  add_rostest(test/squirrel_attention_color_test.test)
  add_rostest(test/squirrel_attention_itti_test.test)
  add_rostest(test/squirrel_attention_symmetry_test.test)
  add_rostest(test/squirrel_attention_3Dsymmetry_test.test)
  add_rostest(test/squirrel_attention_height_test.test)
  add_rostest(test/squirrel_attention_surfaceorientation_test.test)
endif()