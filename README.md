squirrel_perception
===================

[![Build Status](https://travis-ci.com/squirrel-project/squirrel_perception.svg?token=ZW4zPsKpxxD4UyghzW3C&branch=indigo_dev)](https://travis-ci.com/squirrel-project/squirrel_perception)

Technical Maintainer: bajo (Markus "Bajo" Bajones, TU Wien)

Repository for object perception related SQUIRREL packages.

Install dependencies for v4r.

```rosdep install --from-paths $CATKIN_WS/src/squirrel_perception -i -y```

As v4r is a CMake based package, we can not use catkin_make to build the packages.
Use catkin build instead.

```sudo apt-get install python-catkin-tools```

```catkin build -DCMAKE_BUILD_TYPE=Release```
