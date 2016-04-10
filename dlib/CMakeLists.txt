cmake_minimum_required(VERSION 2.8.3)
project(dlib)

find_package(catkin REQUIRED)
catkin_package(
)

include(ExternalProject) 
externalproject_add( dlib
 URL "http://dlib.net/files/dlib-18.18.tar.bz2" 
 # local path included for testing
 #URL "/home/bajo/Downloads/dlib-18.18.tar.bz2"
 URL_HASH MD5=288d609c13b1c3c1957c6a12d0052e98
 DOWNLOAD_NAME dlib-18.18.tar.bz2 
 CONFIGURE_COMMAND ""
 BUILD_COMMAND ""
 #INSTALL_COMMAND ""
 INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
                     <SOURCE_DIR>/dlib/ ${CATKIN_DEVEL_PREFIX}/include/dlib
)

ExternalProject_Get_Property(dlib source_dir)
set(DLIB_PATH ${source_dir})
