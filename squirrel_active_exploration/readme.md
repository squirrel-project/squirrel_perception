squirrel_active_exploration
===========================

Maintanier: <br />
Tim Patten, t.patten@acfr.usyd.edu.au

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

The parameters are:
  * *save_directory*: the directory to save all data (point clouds, probabilities and results)
  * *data_directory*: the directory to load all data from (point clouds and transformations)
  * *image_file*: a necassary file for call the segmentation algorithm
  * entropy_order_file*: a precomputed file that specifies the individual entropy of each view (only necessary for best_to_worst and
worst_to_best methods)
  * *views_limit_file*: a file specifying the number of views within each dataset, the dataset used will automatically be found in this file
  * *single_class_test*: boolean to specify if the dataset is only one object (changes behaviour of algorithm)
  * *reverse_transforms*: boolean to specify if the transformation files are in fact inverse transforms (this is the case for the 
Willow Garage dataset, not the case for the TUW dataset)
  * *load_segmentation*: boolean to specify if precomputed segmentation files can be used (it is helpful when generating results to
precompute the segmentation so that it does not have to happen everytime)
  * *variance*: the sigma value for exponential factor in the utility function (see Patten et al.)
  * *plan_type*: sets the planning mode (the options are worst_to_best_entropy, best_to_worst_entropy, worst_to_best_probability,
best_to_worst_probability, nearest_location_area, nearest_location_min_entropy, nearest_location_max_probability, random, max_area,
max_area_unoccluded, min_class_entropy, min_class_entropy_unoccluded, max_class_probability, max_class_probability_unoccluded,
min_view_classification_entropy, max_view_classification_probability, clockwise, anticlockwise)
  * *start_index*: the index of the dataset point clouds to begin with
  * *maximum_iterations*: the maximum number of iterations (if specified very large this will stop once all views have been analysed)
  * *loaded_views_limit*: the maximum number of views in the dataset to use (some datasets are broken and contain strange files that need
to be ignored, however in general just set this to the number of files in the dataset)
  * *expected_number_objects*: the number of objects in the scene (Willow Garage dataset all have 6 objects)
  * *expected_number_classes*: the number of classes that were used for training (this and the previous parameter are only important when
computing the entropy of unseen objects, these can be ignored of that is not important)
  * *visualize*: boolean flag to view the intermediate steps of the algorithm (the user must then interact in order to move onto next stages)
  * *save*: boolean flag to save results data
  * *generate_order*: boolean flag to generate the entropy_order file (parameter 4), if set this will ignore the planning and exit once all
views are analysed
  * *visualize_views_and_exit*: boolean flag to command the program to load the dataset and visualize the viewpoint locations, use this to
verify that the dataset is in working order
  * *max_object_distance*: after segmentation, remove objects further away than this distance (helpful to ignore walls, etc)
  * *min_object_height*: after segmentation, remove objects that have a height less than this value
  * *max_object_height*: after segmentation, remove objects that have a height greater than this value
  * *min_object_length*: after segmentation, remove objects that have a length less than this value
  * *max_object_lenght*: after segmentation, remove objects that have a length greater than this value
  * *table_height_threshold*: after segmentation, remove objects that have a centroid below the table height
  * *voxel_overlap*: the amount of overlap between segments in different frames to be considered the same object (see Patten et al.)

## Running with robot
Run the launch file `run_robot_experiment.launch`, remembering that `robot_controll.launch` is also running first. <br />
This will load a file that contains the viewpoints in the environment to treat as candidate viewpoints (for experiments in Patten et al. these were determined as points on a circle that encompassed the objects). Each viewpoint is considered a candidate viewpoint by the planner. As each location is visited it is removed from the list. The point clouds are captured and saved to a file. The planner then loads the file. Helper files for improving the alignment is provided by manual_adjust_point_cloud.cpp. The program will save each viewpoint along with the observed objects with their class distribution. Additionally, a results.txt file is saved that stores information summarising each viewpoint. The format for each line is
```
x_pos	y_pos	z_pos	num_observed_objects	total_entropy	total_entropy+entropy_unseen_objects
```

The parameters are:
  * *save_directory*: the directory to save all data (point clouds, probabilities and results)
  * *map_locations_file*: the file that stores the absolute locations of the candidate viewpoints
  * *store_points_directory*: the directory to store the point cloud files captured from the sensor (stored as .pcd)
  * *image_file*: a necassary file for call the segmentation algorithm
  * *variance*: the sigma value for exponential factor in the utility function (see Patten et al.)
  * *plan_type*: sets the planning mode (the options are worst_to_best_entropy, best_to_worst_entropy, worst_to_best_probability,
best_to_worst_probability, nearest_location_area, nearest_location_min_entropy, nearest_location_max_probability, random, max_area,
max_area_unoccluded, min_class_entropy, min_class_entropy_unoccluded, max_class_probability, max_class_probability_unoccluded,
min_view_classification_entropy, max_view_classification_probability, clockwise, anticlockwise)
  * *expected_number_objects*: the number of objects in the scene (Willow Garage dataset all have 6 objects)
  * *expected_number_classes*: the number of classes that were used for training (this and the previous parameter are only important when
computing the entropy of unseen objects, these can be ignored of that is not important)
  * *scene_center_x*: the x location of the center of the scene, required in order to focus the gaze of the robot when viewing the objects
  * *scene_center_y*: the y location of the center of the scene
  * *kinect_height*: the height of the sensor on the robot, relative to the floor
  * *visualize*: boolean flag to view the intermediate steps of the algorithm (the user must then interact in order to move onto next stages)
  * *save*: boolean flag to save results data
  * *max_object_distance*: after segmentation, remove objects further away than this distance (helpful to ignore walls, etc)
  * *min_object_height*: after segmentation, remove objects that have a height less than this value
  * *max_object_height*: after segmentation, remove objects that have a height greater than this value
  * *min_object_length*: after segmentation, remove objects that have a length less than this value
  * *max_object_lenght*: after segmentation, remove objects that have a length greater than this value
  * *table_height_threshold*: after segmentation, remove objects that have a centroid below the table height (not used)
  * *voxel_overlap*: the amount of overlap between segments in different frames to be considered the same object (see Patten et al.)

## Running as a ros service
TODO
