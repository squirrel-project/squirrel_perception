cmake_minimum_required(VERSION 2.8.3)
project(v4r_lib)
find_package(PCL REQUIRED)

MESSAGE( CATKIN_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} )
include(ExternalProject)
ExternalProject_Add( V4R
 #GIT_REPOSITORY "https://github.com/strands-project/v4r"
 GIT_REPOSITORY "git@rgit.acin.tuwien.ac.at:markus-bajones/v4r.git"
 GIT_TAG "fix_glfw3"
 CMAKE_ARGS ../V4R -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}/../install
 BUILD_COMMAND make
 INSTALL_COMMAND make install 
)

ExternalProject_Get_Property( V4R
  SOURCE_DIR BINARY_DIR INSTALL_DIR)
set(V4R_INCLUDE_DIRS
  "${SOURCE_DIR}"
)
set (V4R_DIR
 "${BINARY_DIR}"
)

