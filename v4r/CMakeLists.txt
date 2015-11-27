cmake_minimum_required(VERSION 2.8.3)
project(v4r_lib)
find_package(PCL REQUIRED)

include(ExternalProject)
externalproject_add( V4R
 GIT_REPOSITORY "git@rgit.acin.tuwien.ac.at:markus-bajones/v4r.git"
 GIT_TAG "master"
 CMAKE_ARGS ../V4R -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
 BUILD_COMMAND ${MAKE} 
 INSTALL_COMMAND make install
)

externalproject_get_property( V4R
  SOURCE_DIR BINARY_DIR INSTALL_DIR)
set(V4R_INCLUDE_DIRS
  "${SOURCE_DIR}"
)
set (V4R_DIR
 "${BINARY_DIR}"
)

