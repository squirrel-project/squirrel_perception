#!/bin/bash

###
# VARIABLES
###
SQUIRREL_DIRECTORY=/home/tpat8946/ros_ws/squirrel/src/squirrel_perception/squirrel_active_exploration
SAVE_DIRECTORY=/home/tpat8946/Data/TUW/Results/online_dataset
DATA_DIRECTORY=/home/tpat8946/Data/TUW/Datasets/TUW_GH30_online_dataset/GH30_online_set_001/min_class_entropy_unoccluded
ENTROPY_FILE_NAME=$SQUIRREL_DIRECTORY/data/config/entropy_order/training_set_3/GH30_online_set_001_min_class_entropy_unoccluded.txt
IMAGE_FILE=/home/tpat8946/ros_ws/squirrel/src/squirrel_perception/squirrel_active_exploration/data/test45.png
VIEWS_LIMIT=$SQUIRREL_DIRECTORY/data/config/views_limit.txt
SINGLE_CLASS_TEST=''
REVERSE_TRANSFORMS=false
LOAD_SEGMENTATION=false
# Don't need to change these variables
VARIANCE=0.5
MAXIMUM_ITERATIONS=50
LOADED_VIEWS_LIMIT=10
EXPECTED_NUMBER_OBJECTS=9
EXPECTED_NUMBER_CLASSES=15
#MAX_OBJECT_DISTANCE=1.5
#MIN_OBJECT_HEIGHT=0.075
#MAX_OBJECT_HEIGHT=1.0
#MIN_OBJECT_LENGTH=0.01
#MAX_OBJECT_LENGTH=1.0
#TABLE_HEIGHT_THRESHOLD=0.5
MAX_OBJECT_DISTANCE=1.5
MIN_OBJECT_HEIGHT=0.05
MAX_OBJECT_HEIGHT=2.0
MIN_OBJECT_LENGTH=0.005
MAX_OBJECT_LENGTH=2.0
TABLE_HEIGHT_THRESHOLD=0.5
VOXEL_OVERLAP=0.4
VISUALIZE_FLAG=false
VISUALIZE_VIEWS_AND_EXIT=false

###
# Compute the entropy order file
###
SAVE_FLAG=false
DUMMY_PLAN=worst_to_best_entropy
GENERATE_ORDER_FLAG=false
DUMMY_START_INDEX=-1
# Run
if [ "$GENERATE_ORDER_FLAG" = "true" ]; then
	echo "Computing the entropy order"
	rosrun squirrel_active_exploration squirrel_run_with_dataset \
		_save_directory:=$SAVE_DIRECTORY \
		_data_directory:=$DATA_DIRECTORY \
		_image_file:=$IMAGE_FILE \
		_entropy_order_file:=$ENTROPY_FILE_NAME \
		_views_limit_file:=$VIEWS_LIMIT \
		_single_class_test:=$SINGLE_CLASS_TEST \
		_reverse_transforms:=$REVERSE_TRANSFORMS \
		_load_segmentation:=$LOAD_SEGMENTATION \
		_variance:=$VARIANCE \
		_plan_type:=$DUMMY_PLAN \
		_start_index:=$DUMMY_START_INDEX \
		_maximum_iterations:=$MAXIMUM_ITERATIONS \
		_loaded_views_limit:=$LOADED_VIEWS_LIMIT \
		_expected_number_objects:=$EXPECTED_NUMBER_OBJECTS \
		_expected_number_classes:=$EXPECTED_NUMBER_CLASSES \
		_visualize:=$VISUALIZE_FLAG \
		_save:=$SAVE_FLAG \
		_generate_order:=$GENERATE_ORDER_FLAG \
		_visualize_views_and_exit:=$VISUALIZE_VIEWS_AND_EXIT \
		_max_object_distance:=$MAX_OBJECT_DISTANCE \
		_min_object_height:=$MIN_OBJECT_HEIGHT \
		_max_object_height:=$MAX_OBJECT_HEIGHT \
		_min_object_length:=$MIN_OBJECT_LENGTH \
		_max_object_length:=$MAX_OBJECT_LENGTH \
		_table_height_threshold:=$TABLE_HEIGHT_THRESHOLD \
		_voxel_overlap:=$VOXEL_OVERLAP
fi

###
# Run the available planners and save the results
###
SAVE_FLAG=true
GENERATE_ORDER_FLAG=false
#PLANS=(max_class_prob_unoccluded)
#PLANS=(best_to_worst_prob best_to_worst_entropy \
#       max_class_prob_unoccluded min_class_entropy_unoccluded max_area_unoccluded \
#       max_class_prob min_class_entropy max_area \
#       nearest_max_prob nearest_min_entropy nearest_area \
#       random worst_to_best_prob worst_to_best_entropy)
#PLANS=(best_to_worst_entropy \
#       max_class_prob min_class_entropy max_area \
#       nearest_max_prob nearest_min_entropy nearest_area \
#       random worst_to_best_entropy)
PLANS=(random)
START_INDEX_MAX=9
# Run
for (( s=9; s<=$START_INDEX_MAX; s++ ))
do
	for p in "${PLANS[@]}"
	do
		echo "Running planner $p"
		rosrun squirrel_active_exploration squirrel_run_with_dataset \
			_save_directory:=$SAVE_DIRECTORY \
			_data_directory:=$DATA_DIRECTORY \
			_image_file:=$IMAGE_FILE \
			_entropy_order_file:=$ENTROPY_FILE_NAME \
			_views_limit_file:=$VIEWS_LIMIT \
			_single_class_test:=$SINGLE_CLASS_TEST \
			_reverse_transforms:=$REVERSE_TRANSFORMS \
			_load_segmentation:=$LOAD_SEGMENTATION \
			_variance:=$VARIANCE \
			_plan_type:=$p \
			_start_index:=$s \
			_maximum_iterations:=$MAXIMUM_ITERATIONS \
			_loaded_views_limit:=$LOADED_VIEWS_LIMIT \
			_expected_number_objects:=$EXPECTED_NUMBER_OBJECTS \
			_expected_number_classes:=$EXPECTED_NUMBER_CLASSES \
			_visualize:=$VISUALIZE_FLAG \
			_save:=$SAVE_FLAG \
			_generate_order:=$GENERATE_ORDER_FLAG \
			_visualize_views_and_exit:=$VISUALIZE_VIEWS_AND_EXIT \
			_max_object_distance:=$MAX_OBJECT_DISTANCE \
			_min_object_height:=$MIN_OBJECT_HEIGHT \
			_max_object_height:=$MAX_OBJECT_HEIGHT \
			_min_object_length:=$MIN_OBJECT_LENGTH \
			_max_object_length:=$MAX_OBJECT_LENGTH \
			_table_height_threshold:=$TABLE_HEIGHT_THRESHOLD \
			_voxel_overlap:=$VOXEL_OVERLAP
	done
done
