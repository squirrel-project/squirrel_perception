# Prerequisites
The tracker will search for its models in the directory specified in the launch file, per default:
```
squirrel_perception/squirrel_object_perception/data/models
```

# Starting
To start the tracker call:
```
roslaunch squirrel_tracking startup.launch
```


The tracker will need the mongo_db running, e.g. 
```
roslaunch squirrel_planning_launch squirrel_planning_system.launch
```


Then you need to 

rosservice call /squirrel_start_object_tracking "object_id: data: 'object1'" 
rosservice call /squirrel_stop_object_tracking
