SET(V4R_UTILS 1 CACHE BOOL "Add description here." )
IF(V4R_UTILS)

PROJECT(v4rUtils)

find_package(Boost REQUIRED COMPONENTS filesystem regex)
#find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
#include_directories("/usr/include/eigen3") 

set(UTILS_CPP 
 distributions.cpp 
 #filesystemhdl.cpp
 filesystem_utils.cpp
 #timehdl.cpp
)
set(UTILS_H
 distributions.h 
 #filesystemhdl.h
 filesystem_utils.h
 #timehdl.h
 math.h
)

include_directories(${EIGEN_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED ${UTILS_CPP} ${UTILS_H})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

#INSTALL(FILES ${UTILS_H} DESTINATION include/v4r/utils)
#INSTALL(TARGETS v4rutils LIBRARY DESTINATION lib/)

ENDIF(V4R_UTILS)

