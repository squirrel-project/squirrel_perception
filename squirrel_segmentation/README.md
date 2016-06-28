<a id="top"/> 
# squirrel_segmentation

The current segmentation is based on plane detection and Euclidean clustering. Segmented objects are discarded if they are too tall or too far away from the robot. A service is provided that returns the segmented object closest to the robot which was not already returned. A more sophisticated solution based on attention cues will be provided in the near future.  

Technical Maintainer: [edith](https://github.com/edith-langer (Edith Langer, TU Wien) - langer@acin.tuwien.ac.at

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_segmentation -i -y
```
## 2. Execution: <a id="2--execution"/> 
```
roslaunch squirrel_segmentation startup.launch
```

## 3. Software architecture <a id="3--software-architecture"/> 

head_pose_estimation 
estimate_focus ![estimate_focus](segmentation.png "Architecture")

<a href="#top">top</a>
