object_perception
=================

Repository for object perception related SQUIRREL packages.

# TUW v4r library and PCL

At the moment, TUW's v4r library needs a particular PCL version to run. This will be fixed in the future. In order to avoid conflicts with other pcl versions (i.e. the built-in ros-pcl version), please follow these steps:

1.) check out PCL as follows into a local directory:

    git clone https://github.com/arbeitor/pcl.git ~/pcl_v4r_fork --branch reconstruction_workshop

2.) If you have an Intel core-i processor 4th generation (Haswell) or newer you need to change the needed version of GCC from 4.2 to 4.6 in the file ~/pcl_v4r_fork/cmake/pcl_find_sse.cmake on line 17.

3.)  build PCL

    mkdir ~/pcl_v4r_fork/build
    cd ~/pcl_v4r_fork/build
    cmake ..
    make ( -j4  if you want to use multiple cores)

**IMPORTANT: DO NOT INSTALL PCL to avoid conflicts with other versions!**

4.) tell catkin to use _pcl_v4r_fork_ for packages dependent on this particular pcl version:

    catkin_make -DPCL_DIR=~/pcl_v4r_fork/build -j1
