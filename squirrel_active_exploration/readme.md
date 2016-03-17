squirrel_active_exploration
===========================


Maintanier: <br />
[Tim Patten] (https://github.com/tpatten) <br />
t.patten@acfr.usyd.edu.au <br />
Australian Centre for Field Robotics, The University of Sydney

## Overview

The squirrel_active_exploration package evaluates the utility of potential viewpoints in an environment. It can be used to plan the next-best-view by comparing the utility of candidate viewpoints and selecting the one with the highest utility. The package operates with RGB-D data in the form of point clouds (.pcd). This can be from a dataset or from a hardware device. For use with a dataset, the clouds must be aligned to a common map frame with a set of transformation files (see [Willow Garage dataset](https://repo.acin.tuwien.ac.at/tmp/permanent/dataset_index.php) for an example). <br />
During the online operation, a belief about each object is maintained consisting of its pose and class. The planner considers future viewpoints and determines the next view that will best improve the beliefs. This is done by maximising a utility function. <br />
A number of alternative planning strategies are implemented and can be chosen by the user.

For details, see [Patten et al.](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7349156&filter%3DAND%28p_IS_Number%3A7163696%29)

Bibtex <br />
@article {Patten2016, <br />
    author = {T. Patten and M. Zillich and R. Fitch and M. Vincze and S. Sukkarieh}, <br />
    journal = {IEEE Robotics and Automation Letters}, <br />
    title = {Viewpoint Evaluation for Online 3-D Active Object Classification}, <br />
    year = {2016}, <br />
    volume = {1}, <br />
    number = {1}, <br />
    pages = {73-81}, <br />
    month = {Jan} }

## Requirements
*squirrel_active_exploration* requires additional nodes to run. <br />
Classification from [squirrel_classification] (https://github.com/squirrel-project/squirrel_perception/blob/indigo_dev/squirrel_classification/launch/startup.launch). <br />
Segmentation from [squirrel_segmentation](https://github.com/squirrel-project/squirrel_perception/tree/indigo_dev/squirrel_segmentation/launch) (use the incremental version). <br />
Entropy map from [entropy_map.launch] (https://github.com/squirrel-project/squirrel_perception/blob/indigo_dev/squirrel_active_exploration/launch/entropy_map.launch). <br />
Robot controller (only for real experiments with a robot) from [robot_controller.launch](https://github.com/squirrel-project/squirrel_perception/blob/indigo_dev/squirrel_active_exploration/launch/robot_controller.launch). <br />
These components communicate with the squirrel_active_exploration module through [ros services](http://wiki.ros.org/Services). <br />
Before running active_exploration, these other ros nodes must be running.

## Entropy maps
This is precomputed training data. It stores an entropy values for each viewpoint of a training model. These entropy values are looked up, during the viewpoint evaluation process, to determine the utility of candidate viewpoints. <br />
How to train:
  * Perform training as per [squirrel_classification] (https://github.com/squirrel-project/squirrel_perception/tree/indigo_dev/squirrel_classification) by creating a directory containing all model files.
  * Run `entropy_map.launch` with the same directory and descriptor parameters as used for training the classifier. Set *classification* parameter to TRUE and *inspect* parameter to FALSE (this prints out the data for each model).
  * `entropy_map.launch` will generate files with in the training directory that contain the entropy information as well information
about pose, point clouds, point cloud centroids, classification probablities and an occupancy tree file.

## Running with dataset
Run the launch file `run_dataset.launch`. <br />
This will load the point cloud and transform files specified in the data directory. The transforms will be used to convert each point cloud into a common frame and the viewpoint locations will be computed. Each viewpoint will be considered a candidate viewpoint during planning. As the program runs it will be selecting the next best view and each view will be removed from the future options. The program will save each viewpoint along with the observed objects with their class distribution. Additionally, a *results.txt* file
is saved that stores information summarising each viewpoint. The format for each line is
```
x_pos	y_pos	z_pos	num_observed_objects	total_entropy	total_entropy+entropy_unseen_objects
```

[Click here] (https://github.com/tpatten/squirrel_perception/blob/indigo_dev/squirrel_active_exploration/parameters_run_with_dataset.txt) file for an explanation about the parameters.

## Running with robot
Run the launch file `run_robot_experiment.launch`, remembering that `robot_controller.launch` is also running first. <br />
This will load a file that contains the viewpoints in the environment to treat as candidate viewpoints (for experiments in Patten et al. these were determined as points on a circle that encompassed the objects). Each viewpoint is considered a candidate viewpoint by the planner. As each location is visited it is removed from the list. The point clouds are captured and saved to a file. The planner then loads the file. Helper files for improving the alignment is provided by manual_adjust_point_cloud.cpp. The program will save each viewpoint along with the observed objects with their class distribution. Additionally, a *results.txt file* is saved that stores information summarising each viewpoint. The format for each line is
```
x_pos	y_pos	z_pos	num_observed_objects	total_entropy	total_entropy+entropy_unseen_objects
```

[Click here] (https://github.com/tpatten/squirrel_perception/blob/indigo_dev/squirrel_active_exploration/parameters_run_with_robot.txt) file for an explanation about the parameters.

## Running as a ros service
TODO

## Map coverage
The map coverage component generates a list of waypoints that when combined view all parts of a map. The algorithm assumes a sensor with a 360 degree field of view, so it is expected that the robot performs a 360 degree turn at each waypoint. <br />
The algorithm works be ray tracing through an [octomap](http://octomap.github.io/) occupancy grid. Initially a grid search is performed in the map to determine a set of locations that are free for the robot to move to (i.e. locations in the map that are occupied or unreachable are not considered). At each location, the number of grid cells that will be observed are counted. For speed ups, ray tracing is oinly performed for locations on the floor of the map. <br />
Once the locations are analysed, a subset of locations are selected that maximally cover the map. The optimization process greedily selects locations that will add the most number of unobserved grid cells to the observed set. <br />
First run
```
roslaunch squirrel_active_exploration squirrel_map_coverge_server.launch
````
This starts the map coverage server. Each call to this will generate the waypoints that cover the input map. This is a ros service that can be called in a number of different ways. <br />
An example is provided by running
```
roslaunch squirrel_active_exploration squirrel_map_coverge.launch
````
which will call the map coverage server with a map file (.bt or .ot). This launch specifies some important parameters.
* *tree_depth*: is the resolution of the tree to perform the ray tracing search, setting it to 14 rather than the default 16 returns a result much faster with similar quality
* *robot_height*: specifies the height of the sensor and seeds the start point for every ray tracing operation
* *robot_outer_range*: specifies the range of the onboard sensor
* *robot_inner_range*: specifies an inner radius of which the robot cannot observe (e.g. the sensor cannot observe locations on the floor that are too close due to the specific angle of the sensor)
* *robot_radius*: specifies the size of the robot, used to determine how near the map locations be to occupied space
* *grid_step*: the resolution of the grid search of the locations in the map
* *maximum_iterations*: specifies the maximum number of iterations in the optimization for the subset of locations
* *visualize*: boolean flag to visualize the resulting waypoints

The server also operates with [ros octomap](http://wiki.ros.org/octomap) messages. This is offered by a different service that takes as input the same as previously described but instead a ros octomap topic instead of a file. Internally, a tree object is extracted from the topic and the function is exatcly the same as if the input were a file.

The output of the ros service is a list of points.

The definitions for the messages are found in [CoveragePlanFile.srv](https://github.com/squirrel-project/squirrel_common/blob/indigo_dev/squirrel_object_perception_msgs/srv/CoveragePlanFile.srv) and [CoveragePlan.srv](https://github.com/squirrel-project/squirrel_common/blob/indigo_dev/squirrel_object_perception_msgs/srv/CoveragePlan.srv) for usage with an input file or a ros topic.
