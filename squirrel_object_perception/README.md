<a id="top"/> 
# squirrel_object_perception

This repo holds the find_dynamic_objects and look_for_objects nodes. 

Technical Maintainer: [bajo](https://github.com/bajo (Markus "Bajo" Bajones, TU Wien) - markus.bajones@gmail.com

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_object_perception -i -y
```
## 2. Execution: <a id="2--execution"/> 
```
roslaunch squirrel_object_perception startup.launch**
```

## 3. Software architecture <a id="3--software-architecture"/> 

look_for_objects![look_for_objects](squirrel_look_for_objects.png "Architecture squirrel_look_for_objects")
find_dynamic_objects ![find_dynamic_objects](squirrel_find_dynamic_objects.png "Architecture squirrel_find_dynamic_objects")

<a href="#top">top</a>
