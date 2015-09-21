#!/bin/bash

###
# VARIABLES
###
#DATA_SETS=(object_01.pcd object_02.pcd object_03.pcd object_08.pcd object_18.pcd object_22.pcd object_23.pcd)
#CLASS_TYPE=(bottle bottle tin_can tin_can bottle can can)
#DATA_SETS=(coffee_container.pcd muller_milch_banana.pcd opencv_book.pcd cisco_phone.pcd)
#CLASS_TYPE=(can bottle book telephone)
#DATA_SETS=(set_00007 set_00011 set_00012)
#CLASS_TYPE=('' '' '')
#EXPECTED_NUMBER_OBJECTS=(6 6 9)
#DATA_PATH=/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set

DATA_SETS=(T_09_willow_dataset T_10_willow_dataset T_11_willow_dataset T_12_willow_dataset T_13_willow_dataset T_14_willow_dataset T_15_willow_dataset)
CLASS_TYPE=''
EXPECTED_NUMBER_OBJECTS=6
DATA_PATH=/home/tpat8946/Data/TUW/Datasets/willow_dataset_training_models_gt/test_set

RESULTS_PATH=/home/tpat8946/Data/TUW/Results
TRAINING_SET=training_set_3
SQUIRREL_DIRECTORY=/home/tpat8946/ros_ws/squirrel_active_exploration/src/squirrel_active_exploration
VIEWS_LIMIT=$SQUIRREL_DIRECTORY/data/config/views_limit.txt
REVERSE_TRANSFORMS=true
LOAD_SEGMENTATION=true
# Don't need to change these variables
VARIANCE=0.5
MAXIMUM_ITERATIONS=50
LOADED_VIEWS_LIMIT=50
EXPECTED_NUMBER_CLASSES=15
#MAX_OBJECT_DISTANCE=1.5
#MIN_OBJECT_HEIGHT=0.075
#MAX_OBJECT_HEIGHT=1.0
#MIN_OBJECT_LENGTH=0.01
#MAX_OBJECT_LENGTH=1.0
#TABLE_HEIGHT_THRESHOLD=0.5
MAX_OBJECT_DISTANCE=1.5
MIN_OBJECT_HEIGHT=0.05
MAX_OBJECT_HEIGHT=1.0
MIN_OBJECT_LENGTH=0.01
MAX_OBJECT_LENGTH=2.0
TABLE_HEIGHT_THRESHOLD=0.6
VOXEL_OVERLAP=0.5
VISUALIZE_FLAG=false
VISUALIZE_VIEWS_AND_EXIT=false

###
# RUN the experiments
###
for i in "${!DATA_SETS[@]}"; do
	d_set=${DATA_SETS[$i]}
	c_type=$CLASS_TYPE
	save_dir=$RESULTS_PATH/$TRAINING_SET
	data_dir=$DATA_PATH/$d_set
	name_no_pcd="${d_set%.*}"
	entropy_file_name=$SQUIRREL_DIRECTORY/data/config/entropy_order/$TRAINING_SET/$name_no_pcd.txt
	e_objects=$EXPECTED_NUMBER_OBJECTS
	
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
			_save_directory:=$save_dir \
			_data_directory:=$data_dir \
			_entropy_order_file:=$entropy_file_name \
			_views_limit_file:=$VIEWS_LIMIT \
			_single_class_test:=$c_type \
			_reverse_transforms:=$REVERSE_TRANSFORMS \
			_load_segmentation:=$LOAD_SEGMENTATION \
			_variance:=$VARIANCE \
			_plan_type:=$DUMMY_PLAN \
			_start_index:=$DUMMY_START_INDEX \
			_maximum_iterations:=$MAXIMUM_ITERATIONS \
			_loaded_views_limit:=$LOADED_VIEWS_LIMIT \
			_expected_number_objects:=$e_objects \
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
	PLANS=(best_to_worst_entropy best_to_worst_prob \
		   max_class_prob_unoccluded min_class_entropy_unoccluded \
           max_class_prob min_class_entropy max_area \
           nearest_max_prob nearest_min_entropy nearest_area \
           random worst_to_best_prob worst_to_best_entropy)
	START_INDEX_MIN=10
	START_INDEX_MAX=16
	# Run
	for (( s=$START_INDEX_MIN; s<$START_INDEX_MAX; s++ ))
	do
		for p in "${PLANS[@]}"
		do
			echo "Running planner $p"
			rosrun squirrel_active_exploration squirrel_run_with_dataset \
				_save_directory:=$save_dir \
				_data_directory:=$data_dir \
				_entropy_order_file:=$entropy_file_name \
				_views_limit_file:=$VIEWS_LIMIT \
				_single_class_test:=$c_type \
				_reverse_transforms:=$REVERSE_TRANSFORMS \
				_load_segmentation:=$LOAD_SEGMENTATION \
				_variance:=$VARIANCE \
				_plan_type:=$p \
				_start_index:=$s \
				_maximum_iterations:=$MAXIMUM_ITERATIONS \
				_loaded_views_limit:=$LOADED_VIEWS_LIMIT \
				_expected_number_objects:=$e_objects \
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
done
