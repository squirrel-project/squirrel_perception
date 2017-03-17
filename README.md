squirrel_perception
===================

Travis-CI: [![Build Status](https://travis-ci.org/squirrel-project/squirrel_perception.svg?branch=indigo_dev)](https://travis-ci.org/squirrel-project/squirrel_perception)

Technical Maintainer: bajo (Markus "Bajo" Bajones, TU Wien)

Repository for object perception related SQUIRREL packages.

# Important

Starting February 2017 we use a different repository and package name for the v4r library. Therefore you need to follow these steps.

1. Delete the old ros-indigo-v4r debian package.
`sudo apt-get purge ros-indigo-v4r`

2. Make sure all v4r folders in /usr/local/include, /usr/local/share, /opt/ros/indigo/include, and /opt/ros/indigo/share are deleted. If not, delete them manually.

3. Setup the debian package repository for the new v4r debian package.

```roscd squirrel_perception ```

bash: ``` ./setup.bash```

zsh: ```./setup.zsh```

4. Install the new v4r debian package through one of the following options
  * Manually with `sudo apt-get install v4r`
  * With rosdep using the command ```rosdep install --from-paths $CATKIN_WS/src/squirrel_perception -i -y```

5. Delete the build and devel folder in your catkin workspace


The [Wiki](https://github.com/squirrel-project/squirrel_perception/wiki) provides tutorials (i.e. How to create a 3D object model).
