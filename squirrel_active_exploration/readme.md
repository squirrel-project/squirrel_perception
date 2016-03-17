squirrel_active_exploration
===========================


Maintanier: <br />
Tim Patten <br />
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

## Entropy Maps
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
