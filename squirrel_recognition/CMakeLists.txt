cmake_minimum_required(VERSION 2.8.3)
project(squirrel_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS roscpp rospy squirrel_object_perception_msgs genmsg cv_bridge sensor_msgs image_transport tf tf2)

find_package( PCL 1.7 REQUIRED )
find_package( OpenCV 2.4 REQUIRED )
find_package( V4R REQUIRED )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy squirrel_object_perception_msgs tf tf2
)

###########
## Build ##
###########

add_definitions(-std=c++0x)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(squirrel_recognizer src/recognizer_ros.cpp ${incs})
add_dependencies(squirrel_recognizer squirrel_object_perception_msgs_generate_messages_cpp)
add_executable(test_squirrel_recognizer src/test_recognizer.cpp ${incs})
add_dependencies(test_squirrel_recognizer squirrel_object_perception_msgs_generate_messages_cpp)

add_executable(squirrel_recognizer2d src/recognizer2d_ros.cpp ${incs})
add_dependencies(squirrel_recognizer2d squirrel_object_perception_msgs_generate_messages_cpp)
add_executable(test_squirrel_recognizer2d src/test_recognizer2d.cpp ${incs})
add_dependencies(test_squirrel_recognizer2d squirrel_object_perception_msgs_generate_messages_cpp)

add_executable(squirrel_recognizer_wizard src/recognizer_wizard.cpp ${incs})
add_dependencies(squirrel_recognizer_wizard squirrel_object_perception_msgs_generate_messages_cpp)
add_executable(test_squirrel_recognizer_wizard src/test_recognizer_wizard.cpp ${incs})
add_dependencies(test_squirrel_recognizer_wizard squirrel_object_perception_msgs_generate_messages_cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

target_link_libraries(squirrel_recognizer
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(test_squirrel_recognizer
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(squirrel_recognizer2d
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(test_squirrel_recognizer2d
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(squirrel_recognizer_wizard
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)

target_link_libraries(test_squirrel_recognizer_wizard
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${V4R_LIBS}
)


#############
## Install ##
#############

#
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasping_recognition_master.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
