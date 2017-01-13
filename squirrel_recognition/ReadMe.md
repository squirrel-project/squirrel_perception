<a id="top"/> 
# squirrel_recognition

The current recognizer uses mutiple pipeline using SHOT and SIFT descriptors and then hypotheses verification to produce reliable recognized objects. A service is provided that returns the recognized objects. 

Technical Maintainer: [jean-baptiste](https://github.com/jibweb) (Jean-Baptiste Weibel, TU Wien) - weibel@acin.tuwien.ac.at

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

####ROS packages
The ROS packages dependencies can be installed with the command:
```
rosdep install --from-path squirrel_recognition -i -y
```

The recognizer also expect to find object models in `squirrel_object_perception/data/models`

## 2. Execution: <a id="2--execution"/> 
```
roslaunch squirrel_recognition startup.launch
```

To test the recognizer you can use 
```
rosrun squirrel_recognition test_squirrel_recognizer
```
which will recognize the object in the camera feed or in a directory pcd file (if the `directory` param is set)

## 3. Software architecture <a id="3--software-architecture"/> 

![recognizer](recognizer.png "Architecture")

<a href="#top">top</a>
