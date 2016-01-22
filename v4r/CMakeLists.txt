cmake_minimum_required(VERSION 2.8.3)
project(v4r_lib)
find_package(PCL REQUIRED)

include(ExternalProject)
externalproject_add( V4R
 URL "https://rgit.acin.tuwien.ac.at/markus-bajones/v4r/repository/archive.tar.gz?ref=squirrel_integration_meeting_january"
 URL_HASH MD5=c9962f25a38808d4ec8bc61ab6d30936
 DOWNLOAD_NAME v4r.tar.gz
 #GIT_REPOSITORY "https://rgit.acin.tuwien.ac.at/markus-bajones/v4r.git"
 #GIT_TAG "master"
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

