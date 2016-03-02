# Prerequisites
The tracker will search for its models in the directory specified in the launch file, per default:
```
squirrel_perception/squirrel_object_perception/data/models
```

The tracker is listening to topics, and TF:
```
/kinect/rgb/camera_info
/kinect/rgb/image_rect_color
kinect_rgb_optical_frame
```

The tracker will need the scene database running, which happens when the whole scenario system is started up, or e.g. using:
```
sudo service mongodb stop
roslaunch squirrel_planning_launch squirrel_planning_system.launch
```

# Starting
To start the tracker call:
```
roslaunch squirrel_tracking startup.launch
```

# Testing

To test the tracker you first need to create an object in the scene database. Normally this happens as part of looking actions. You can also manually inject an object, here with ID 'object1', the category name 'mueslibox' and bounding sphere size '0.3':
```
rosrun squirrel_object_perception inject_object.py -i object1 -c mueslibox -s 0.3
```

Not you are ready to start tracking object1:
```
rosservice call /squirrel_start_object_tracking "object_id: data: 'object1'" 
```

Once the tracker has latched onto the object, you will see output in the tracker's console, like:
```
[ INFO] [1454008740.670228963]: squirrel_tracking: conf 0.542, pos 0.111 0.132 0.803
```

To stop the tracker call:
```
rosservice call /squirrel_stop_object_tracking
```
