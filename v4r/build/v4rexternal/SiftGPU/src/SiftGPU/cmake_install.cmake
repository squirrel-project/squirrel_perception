# Install script for directory: /home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4rexternal/SiftGPU/src/SiftGPU" TYPE DIRECTORY FILES "")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rsiftgpu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rsiftgpu.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rsiftgpu.so"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/lib/libv4rsiftgpu.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rsiftgpu.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rsiftgpu.so")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rsiftgpu.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4rexternal/SiftGPU/src/SiftGPU" TYPE FILE FILES
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/CLTexImage.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/GLTexImage.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/ProgramCU.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/PyramidCU.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/SiftMatchCU.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/CuTexImage.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/LiteWindow.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/ProgramGLSL.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/PyramidGL.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/SiftMatch.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/FrameBufferObject.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/ProgramCG.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/ProgramGPU.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/ShaderMan.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/SiftPyramid.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/GlobalUtil.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/ProgramCL.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/PyramidCL.h"
    "/home/mz/work/SQUIRREL/code/catkin_ws/src/object_perception/v4r/v4rexternal/SiftGPU/src/SiftGPU/SiftGPU.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

