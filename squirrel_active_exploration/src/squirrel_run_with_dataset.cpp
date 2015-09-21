#include "squirrel_active_exploration/active_exploration.h"
#include "squirrel_active_exploration/visualization_utils.h"
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace pcl;

#define _DATA_DIR "/home/tpat8946/Data/TUW/willow_dataset_training_models_gt/test_set/T_01_willow_dataset"
#define _SQUIRREL_DIRECTORY "/home/tpat8946/ros_ws/squirrel_active_exploration/src/squirrel_active_exploration"
#define _VIEWS_LIMIT_FILE "views_limit.txt"
#define _SINGLE_CLASS_TEST "null"
#define _REVERSE_TRANSFORMS 0
#define _LOAD_SEGMENTATION 0
#define _VARIANCE 0.5
#define _PLAN_TYPE "max_area"
#define _EXPECTED_NUMBER_OBJECTS -1
#define _EXPECTED_NUMBER_CLASSES -1
#define _START_INDEX -1
#define _MAXIMUM_ITERATIONS 5
#define _LOADED_VIEWS_LIMIT 100
#define _VISUALIZE_FLAG 0
#define _SAVE_FLAG 0
#define _GENERATE_VIEW_ORDER 0
#define _VISUALIZE_VIEWS_EXIT 0

bool parse_input(const ros::NodeHandle &n, string &data_dir, string &entropy_order_file, string &views_lim_file, string &single_class_test,
                 bool &reverse_transforms, bool &load_segmentation_indices, double &variance, SIM_TYPE &sim,
                 int &expected_number_objects, int &expected_number_classes, int &max_iters, int &start_index, int &loaded_views_limit,
                 bool &do_visualize, bool &do_saving, bool &do_generate_order, bool &visualize_views_and_exit)
{
    // Default values
    data_dir = _DATA_DIR;
    entropy_order_file = _SQUIRREL_DIRECTORY;
    views_lim_file = _VIEWS_LIMIT_FILE;
    single_class_test = _SINGLE_CLASS_TEST;
    reverse_transforms = _REVERSE_TRANSFORMS;
    load_segmentation_indices = _LOAD_SEGMENTATION;
    variance = _VARIANCE;
    expected_number_objects = _EXPECTED_NUMBER_OBJECTS;
    expected_number_classes = _EXPECTED_NUMBER_CLASSES;
    max_iters = _MAXIMUM_ITERATIONS;
    loaded_views_limit = _LOADED_VIEWS_LIMIT;
    do_visualize = _VISUALIZE_FLAG;
    do_saving = _SAVE_FLAG;
    do_generate_order = _GENERATE_VIEW_ORDER;
    visualize_views_and_exit = _VISUALIZE_VIEWS_EXIT;
    // Read the arguments
    n.getParam("data_directory", data_dir);
    n.getParam("entropy_order_file", entropy_order_file);
    n.getParam("views_limit_file", views_lim_file);
    n.getParam("single_class_test", single_class_test);
    n.getParam("reverse_transforms", reverse_transforms);
    n.getParam("load_segmentation", load_segmentation_indices);
    n.getParam("variance", variance);
    n.getParam("expected_number_objects", expected_number_objects);
    n.getParam("expected_number_classes", expected_number_classes);
    n.getParam("start_index", start_index);
    n.getParam("maximum_iterations", max_iters);
    n.getParam("loaded_views_limit", loaded_views_limit);
    n.getParam("visualize", do_visualize);
    n.getParam("save", do_saving);
    n.getParam("generate_order", do_generate_order);
    n.getParam("visualize_views_and_exit", visualize_views_and_exit);
    // Get plan type and convert to correct format
    string plan_type = _PLAN_TYPE;
    n.getParam("plan_type", plan_type);
    sim = MAX_AREA;
    if (boost::iequals(plan_type,"worst_to_best_entropy") || boost::iequals(plan_type,"worst_entropy") ||
        boost::iequals(plan_type,"worst_to_best_ent") || boost::iequals(plan_type,"worst_ent"))
        sim = WORST_TO_BEST_ENTROPY;
    else if (boost::iequals(plan_type,"best_to_worst_entropy") || boost::iequals(plan_type,"best_entropy") ||
             boost::iequals(plan_type,"best_to_worst_ent") || boost::iequals(plan_type,"best_ent"))
        sim = BEST_TO_WORST_ENTROPY;
    else if (boost::iequals(plan_type,"worst_to_best_probability") || boost::iequals(plan_type,"worst_probability") ||
             boost::iequals(plan_type,"worst_to_best_prob") || boost::iequals(plan_type,"worst_prob"))
        sim = WORST_TO_BEST_PROB;
    else if (boost::iequals(plan_type,"best_to_worst_probability") || boost::iequals(plan_type,"best_probability") ||
             boost::iequals(plan_type,"best_to_worst_prob") || boost::iequals(plan_type,"best_prob"))
        sim = BEST_TO_WORST_PROB;
    else if (boost::iequals(plan_type,"nearest_location_area") || boost::iequals(plan_type,"nearest_area"))
        sim = NEAREST_AREA;
    else if (boost::iequals(plan_type,"nearest_location_min_entropy") || boost::iequals(plan_type,"nearest_min_entropy") ||
             boost::iequals(plan_type,"nearest_location_entropy") || boost::iequals(plan_type,"nearest_entropy"))
        sim = NEAREST_MIN_ENTROPY;
    else if (boost::iequals(plan_type,"nearest_location_max_probability") || boost::iequals(plan_type,"nearest_max_probability") ||
             boost::iequals(plan_type,"nearest_location_probability") || boost::iequals(plan_type,"nearest_probability") ||
             boost::iequals(plan_type,"nearest_location_max_prob") || boost::iequals(plan_type,"nearest_max_prob") ||
             boost::iequals(plan_type,"nearest_location_prob") || boost::iequals(plan_type,"nearest_prob") )
        sim = NEAREST_MAX_CLASS_PROB;
    else if (boost::iequals(plan_type, "random") || boost::iequals(plan_type,"rand"))
        sim = RANDOM;
    else if (boost::iequals(plan_type,"max_area") || boost::iequals(plan_type,"area"))
        sim = MAX_AREA;
    else if (boost::iequals(plan_type,"max_area_unoccluded") || boost::iequals(plan_type,"area_unoccluded"))
        sim = MAX_AREA_UNOCCLUDED;
    else if (boost::iequals(plan_type,"min_class_entropy") || boost::iequals(plan_type,"min_class") || boost::iequals(plan_type,"entropy"))
        sim = MIN_CLASS_ENTROPY;
    else if (boost::iequals(plan_type,"min_class_entropy_unoccluded") || boost::iequals(plan_type,"min_class_unoccluded") ||
             boost::iequals(plan_type,"min_entropy_unoccluded") || boost::iequals(plan_type,"entropy_unoccluded"))
        sim = MIN_CLASS_ENTROPY_UNOCCLUDED;
    else if (boost::iequals(plan_type,"max_class_probability") || boost::iequals(plan_type,"max_class") ||
             boost::iequals(plan_type,"probability") || boost::iequals(plan_type,"max_class_prob") || boost::iequals(plan_type,"prob"))
        sim = MAX_CLASS_PROB;
    else if (boost::iequals(plan_type,"max_class_probability_unoccluded") || boost::iequals(plan_type,"max_class_unoccluded") ||
             boost::iequals(plan_type,"max_probability_unoccluded") || boost::iequals(plan_type,"probability_unoccluded") ||
             boost::iequals(plan_type,"max_class_prob_unoccluded") || boost::iequals(plan_type,"max_prob_unoccluded") ||
             boost::iequals(plan_type,"prob_unoccluded"))
        sim = MAX_CLASS_PROB_UNOCCLUDED;
    else if (boost::iequals(plan_type,"min_view_classification_entropy"))
        sim = MIN_VIEW_CLASSIFICATION_ENTROPY;
    else if (boost::iequals(plan_type,"max_view_classification_probability") || boost::iequals(plan_type,"max_view_classification_prob"))
        sim = MAX_VIEW_CLASSIFICATION_PROB;
    else
    {
        ROS_ERROR("squirrel_run_planner::parse_input : could not interpret plan type input %s", plan_type.c_str());
        return false;
    }
    // Print the input to console
    ROS_INFO(" -- PARAMETERS --");
    ROS_INFO("  Data directory = %s", data_dir.c_str());
    ROS_INFO("  Entropy order file = %s", entropy_order_file.c_str());
    ROS_INFO("  Views limit file = %s", views_lim_file.c_str());
    ROS_INFO("  Single class test = %s", single_class_test.c_str());
    if (reverse_transforms)
        ROS_INFO("  Reverse transforms = TRUE");
    else
        ROS_INFO("  Reverse transforms = FALSE");
    if (load_segmentation_indices)
        ROS_INFO("  Load segmentations = TRUE");
    else
        ROS_INFO("  Load segmentations = FALSE");
    ROS_INFO("  Variance = %.4f", variance);
    ROS_INFO("  Simulation type = %s", plan_type.c_str());
    ROS_INFO("  Expected number objects = %i", expected_number_objects);
    ROS_INFO("  Expected number classes = %i", expected_number_classes);
    ROS_INFO("  Start index = %i", start_index);
    ROS_INFO("  Maximum iterations = %i", max_iters);
    ROS_INFO("  Loaded views limit = %i", loaded_views_limit);
    if (do_visualize)
        ROS_INFO("  Visualization = ON");
    else
        ROS_INFO("  Visualization = OFF");
    if (do_saving)
        ROS_INFO("  Saving = ON");
    else
        ROS_INFO("  Saving = OFF");
    if (do_generate_order)
        ROS_INFO("  Generating order = ON");
    else
        ROS_INFO("  Generating order = OFF");
    if (visualize_views_and_exit)
        ROS_INFO("  Visualizing views in data set = ON");
    else
        ROS_INFO("  Visualizing views in data set = OFF");
    ROS_INFO(" ----------------");

    return true;
}

bool load_data(const string &data_dir, const bool &reverse_transforms, const bool &load_segmentation_indices,
               const int &loaded_views_limit, vector<Eigen::Vector4f> &camera_poses, vector<Eigen::Vector4f> &transformed_locations,
               vector<PointCloud<PointT> > &clouds, vector<Eigen::Matrix4f> &transforms, vector<vector<vector<int> > > &indices,
               sensor_msgs::Image &in_image)
{
    if (load_segmentation_indices)
    {
        if (!load_test_directory_with_segment_indices(data_dir, reverse_transforms, camera_poses, clouds, transforms, indices))
        {
            ROS_ERROR("squirrel_run_planner::load_data : could not load directory with segmentation indices %s", data_dir.c_str());
            return false;
        }
    }
    else
    {
        indices.clear();
        if (!load_test_directory(data_dir, reverse_transforms, camera_poses, clouds, transforms))
        {
            ROS_ERROR("squirrel_run_planner::load_data : could not load directory %s", data_dir.c_str());
            return false;
        }
    }
    // Transform the locations and get the current location
    if (camera_poses.size() == 0)
    {
        ROS_ERROR("squirrel_run_planner::load_data : no available map locations");
        return false;
    }
    // Remove some vector entries
    if (loaded_views_limit < camera_poses.size())
    {
        camera_poses.erase(camera_poses.begin() + loaded_views_limit, camera_poses.end());
        clouds.erase(clouds.begin() + loaded_views_limit, clouds.end());
        transforms.erase(transforms.begin() + loaded_views_limit, transforms.end());
        indices.erase(indices.begin() + loaded_views_limit, indices.end());
    }
    transformed_locations.resize(camera_poses.size());
    for (vector<Eigen::Vector4f>::size_type i = 0; i < camera_poses.size(); ++i)
        transformed_locations[i] = transform_eigvec(camera_poses[i], transforms[i]);

    // Load the image
    string image_name = "/home/tpat8946/ros_ws/squirrel_active_exploration/src/squirrel_active_exploration/data/test45.png";
    cv::Mat image = cv::imread(image_name,-1);
    cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // Convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "saliency_map";
    cv_ptr->encoding = "mono8";
    cv_ptr->image = image;
    cv_ptr->toImageMsg(in_image);

    return true;
}

void visualize_views(const vector<Eigen::Vector4f> &transformed_locations)
{
    PointCloud<PointT> vec;
    for (int i = 0; i < transformed_locations.size(); ++i)
    {
        PointCloud<PointT> pt;
        pt.resize(1);
        pt.points[0].x = transformed_locations[i][0];
        pt.points[0].y = transformed_locations[i][1];
        pt.points[0].z = transformed_locations[i][2];
        vec.push_back(pt.points[0]);
    }
    visualization_u::visualize_point_cloud(vec);
}

bool adjust_start_index(int &start_index, vector<int> &view_order, const vector<PointCloud<PointT> > &clouds, const SIM_TYPE &sim)
{
    if (sim == WORST_TO_BEST_ENTROPY || sim == WORST_TO_BEST_PROB || sim == BEST_TO_WORST_ENTROPY || sim == BEST_TO_WORST_PROB)
    {
        // Exit if no valid view order was found
        if (view_order.size() == 0)
        {
            ROS_ERROR("squirrel_run_planner::adjust_start_index : no valid view order for the planner");
            return false;
        }
        // If the index is larger than the available views then exit
        if (start_index < 0)
            ROS_WARN("squirrel_run_planner::adjust_start_index : start index is negative, running from worst location");
        if (start_index < 0 || (start_index >= 0 && start_index < clouds.size()))
        {
            // Ignore the start index and set to the view order element
            start_index = view_order.back();
            view_order.pop_back();
            // If want to iterate from worst to best then need to reverse the vector
            if (sim == WORST_TO_BEST_ENTROPY || sim == WORST_TO_BEST_PROB)
                reverse(view_order.begin(), view_order.end());
        }
        else
        {
            ROS_ERROR("squirrel_run_planner::adjust_start_index : start index %i is larger than the views in view_order %lu",
                      start_index, view_order.size());
            return false;
        }
    }
    else
    {
        // Using the other planners does not follow the view order,
        // unless if start_index < 0 then must begin with the worst entropy location
        if (start_index < 0)
        {
            // Exit if no valid view order was found
            if (view_order.size() == 0)
            {
                ROS_ERROR("squirrel_run_planner::adjust_start_index : no valid view order to start the planner at the worst entropy location");
                return false;
            }
            // otherwise set the last element
            start_index = view_order.back();
            view_order.pop_back();
        }
        else
        {
            // If the index is larger than the available views then exit
            if (start_index > clouds.size())
            {
                ROS_ERROR("squirrel_run_planner::adjust_start_index : start index %u is larger than the views in view_order %lu",
                          start_index, view_order.size());
                return false;
            }
        }
    }
    return true;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    // Seed the random number generator
    srand(time(NULL));

    // Create and initialize the active exploration object
    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);

    // Get the input
    string data_dir;
    string entropy_order_file;
    string views_lim_file;
    string single_class_test;
    bool reverse_transforms;
    bool load_segmentation_indices;
    double variance;
    SIM_TYPE sim;
    int expected_number_objects;
    int expected_number_classes;
    int start_index;
    int max_iters;
    int loaded_views_limit;
    bool do_visualize;
    bool do_saving;
    bool do_generate_order;
    bool visualize_views_and_exit;
    if (!parse_input(*exp->get_ros_node_handle(), data_dir, entropy_order_file, views_lim_file, single_class_test, reverse_transforms,
                     load_segmentation_indices, variance, sim, expected_number_objects, expected_number_classes, max_iters, start_index,
                     loaded_views_limit, do_visualize, do_saving, do_generate_order, visualize_views_and_exit))
    {
        ROS_ERROR("squirrel_run_planner::main : could not parse the input");
        return EXIT_FAILURE;
    }

    // Try to load the views limit file
    int loaded_views_limit_from_file = loaded_views_limit;
    if (!visualize_views_and_exit)
    {
        if (load_views_limit_from_file(views_lim_file, data_dir, loaded_views_limit_from_file))
        {
            ROS_INFO("squirrel_run_planner::main : successfully loaded views limit from file %s", views_lim_file.c_str());
            ROS_INFO("squirrel_run_planner::main : views limit is %u", loaded_views_limit_from_file);
            loaded_views_limit = loaded_views_limit_from_file;
        }
        else
        {
            ROS_ERROR("squirrel_run_planner::main : could not load views limit from file %s", views_lim_file.c_str());
        }
    }

    // Try to load the entropy_order file
    bool valid_view_order = true;
    vector<int> view_order;
    if (do_generate_order || visualize_views_and_exit)
        valid_view_order = false;
    else
        view_order = load_entropy_order_file(entropy_order_file, data_dir, sim);
    if (view_order.size() == 0)
    {
        ROS_WARN("squirrel_run_planner::main : could not load entropy order file %s", entropy_order_file.c_str());
        valid_view_order = false;
    }
    else if (view_order.size() != loaded_views_limit)
    {
        ROS_WARN("squirrel_run_planner::main : views limit is set to %u but has available %lu", loaded_views_limit, view_order.size());
        if (view_order.size() < loaded_views_limit)
            loaded_views_limit = view_order.size();
        else
            view_order.erase(view_order.begin() + loaded_views_limit, view_order.end());
        // Check the sizes again
        if (view_order.size() == loaded_views_limit)
        {
            ROS_WARN("squirrel_run_planner::main : view_order.size() and loaded_views_limit set to %lu", view_order.size());
        }
        else
        {
            ROS_WARN("squirrel_run_planner::main : view_order.size() is %lu and loaded_views_limit is %u",
                     view_order.size(), loaded_views_limit);
            if (exp)
                delete exp;
            return EXIT_FAILURE;
        }
    }

    // Load test directory
    vector<Eigen::Vector4f> camera_poses;
    vector<Eigen::Vector4f> transformed_locations;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<vector<vector<int> > > indices;
    sensor_msgs::Image in_image;
    if (!load_data(data_dir, reverse_transforms, load_segmentation_indices, loaded_views_limit, camera_poses, transformed_locations,
                   clouds, transforms, indices, in_image))
    {
        ROS_ERROR("squirrel_run_planner::main : could not load the data");
        if (exp)
            delete exp;
        return EXIT_FAILURE;
    }
    ROS_INFO("Clouds size is %lu", clouds.size());

    // Visualize the locations
    if (visualize_views_and_exit)
    {
        visualize_views(transformed_locations);
        if (exp)
            delete exp;
        return EXIT_SUCCESS;
    }

    // Set the view order
    if (do_generate_order)
    {
        start_index = 0;
        // Iterate through the views in order of loaded files
        for (vector<PointCloud<PointT> >::size_type i = 1; i < clouds.size(); ++i)
            view_order.push_back(i);
    }
    else
    {
        if (!adjust_start_index(start_index, view_order, clouds, sim))
        {
            ROS_ERROR("squirrel_run_planner::main : could not adjust the start index");
            if (exp)
                delete exp;
            return EXIT_FAILURE;
        }
    }

    // Write the directory name to the front of the entropy order file
    if (do_generate_order)
    {
        // Delete the file if it exists
        if (!boost::filesystem::exists(entropy_order_file))
            remove(entropy_order_file.c_str());
        // Save to file
        ofstream out;
        out.open(entropy_order_file.c_str());
        // Write results
        if (out.is_open())
        {
            // Write the directory name
            out << data_dir << endl;
            out.close();
        }
        else
        {
            ROS_WARN("squirrel_run_planner::main : could not open file to save entropy %s", entropy_order_file.c_str());
        }
    }

    // Turn on visualization
    if (do_visualize)
        exp->turn_on_visualization();
    // Turn on saving
    if (!do_generate_order && !visualize_views_and_exit)
    {
        if (do_saving)
            exp->turn_on_saving(expected_number_objects, expected_number_classes);
    }

    // Extract the current data
    Eigen::Vector4f current_location = transformed_locations[start_index];
    sensor_msgs::PointCloud2 in_cloud;
    toROSMsg(clouds[start_index], in_cloud);
    Eigen::Matrix4f in_transform = transforms[start_index];
    current_location = transformed_locations[start_index];
    vector<vector<int> > in_indices;
    if (indices.size() > 0)
        in_indices = indices[start_index];
    // Remove element from the lists
    clouds.erase(clouds.begin() + start_index);
    transforms.erase(transforms.begin() + start_index);
    transformed_locations.erase(transformed_locations.begin() + start_index);
    if (indices.size() > 0)
        indices.erase(indices.begin() + start_index);
    // Subtract 1 from each new view_order number larger than the selected index
    for (vector<int>::size_type i = 0; i < view_order.size(); ++i)
    {
        if (view_order[i] > start_index)
            view_order[i] = view_order[i] - 1;
    }
    // Loop through the clouds in the directory
    int count = 0;
    while (ros::ok())
    {
        // Process the point cloud
        exp->set_data(in_cloud, in_image, in_transform);
        // Initialize visualizer
        if (!exp->initialize_visualization())
        {
            ROS_ERROR("squirrel_run_planner::main : error in initializing visualization");
            if (exp)
                delete exp;
            return EXIT_FAILURE;
        }
        if (load_segmentation_indices && in_indices.size() > 0)
        {
            ROS_INFO("squirrel_run_planner::main : updating hypotheses with segmentation indices");
            if (!exp->update_hypotheses(in_cloud, in_image, in_transform, in_indices))
            {
                ROS_ERROR("squirrel_run_planner::main : error when updating hypotheses point cloud");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }
        else
        {
            // Update the hypotheses
            ROS_INFO("squirrel_run_planner::main : updating hypotheses");
            if (!exp->update_hypotheses(in_cloud, in_image, in_transform))
            {
                ROS_ERROR("squirrel_run_planner::main : error when updating hypotheses point cloud");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }

        if (do_visualize)
        {
            if (!exp->visualize())
            {
                ROS_ERROR("squirrel_run_planner::main : error when visualizing the point cloud segments");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }

        // Plan
        // It will return the index of the best location in transformed locations
        ROS_INFO("squirrel_run_planner::main : planning");
        int next_best_index = start_index;
        // If this is generating
        if (do_generate_order)
        {
            // Save to file
            ofstream out;
            out.open(entropy_order_file.c_str(), ios::app);
            // Write results
            if (out.is_open())
            {
                // Write the index i
                out << count;
                // Write the total entropy
                vector<double> ent = exp->get_entropies();
                double ent_sum = 0;
                for (vector<double>::size_type i = 0; i < ent.size(); ++i)
                    ent_sum += ent[i];
                out << " " << ent_sum;
                // If the single class test is set
                double class_prob = 0;
                if (single_class_test.size() > 0 && strcmp(single_class_test.c_str(), "null") != 0 &&
                    strcmp(single_class_test.c_str(), "NULL") != 0)
                {
                    // Get the class result for this class
                    vector<squirrel_object_perception_msgs::Classification> class_estimates = exp->get_class_estimates();
                    // Find the classification probability for this class
                    for (size_t i = 0; i < class_estimates.size(); ++i)
                    {
                        for (size_t j = 0; j < class_estimates[i].class_type.size(); ++j)
                        {
                            if (strcmp(rem_backslash(class_estimates[i].class_type[j].data).c_str(), single_class_test.c_str()) == 0)
                                class_prob += class_estimates[i].confidence[j];
                        }
                    }
                }
                out << " " << class_prob << endl;
                out.close();
            }
            else
            {
                ROS_WARN("squirrel_run_planner::main : could not open file to save entropy %s", entropy_order_file.c_str());
            }
            // Clear the data
            exp->clear();
            // If reached the end
            if (view_order.size() == 0)
            {
                ROS_INFO("squirrel_run_planner::main : no more available view points");
                break;
            }
            // Adjust all indices larger than the current index
            for (vector<int>::size_type i = 0; i < view_order.size(); ++i)
            {
                if (view_order[i] > next_best_index)
                    view_order[i] = view_order[i] - 1;
            }
            // Get the next index in the list
            next_best_index = view_order.front();
            // Remove this element from the list
            view_order.erase(view_order.begin());
            start_index = next_best_index;
        }
        else if (sim == WORST_TO_BEST_ENTROPY || sim == BEST_TO_WORST_ENTROPY || sim == WORST_TO_BEST_PROB || sim == BEST_TO_WORST_PROB)
        {
            if (view_order.size() == 0)
            {
                ROS_INFO("squirrel_run_planner::main : no more available view points");
                break;
            }
            // Adjust all indices larger than the current index
            for (vector<int>::size_type i = 0; i < view_order.size(); ++i)
            {
                if (view_order[i] > next_best_index)
                    view_order[i] = view_order[i] - 1;
            }
            // Get the next index in the list
            next_best_index = view_order.front();
            // Remove this element from the list
            view_order.erase(view_order.begin());
            start_index = next_best_index;
        }
        else
        {
            next_best_index = -1;
            if (!exp->plan(next_best_index, sim, variance, transformed_locations))
            {
                ROS_ERROR("squirrel_run_planner::main : error when planning");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }

        // Get the next view
        if (next_best_index >= 0 && next_best_index < transformed_locations.size())
        {
            // Get the new current location
            toROSMsg(clouds[next_best_index], in_cloud);
            in_transform = transforms[next_best_index];
            current_location = transformed_locations[next_best_index];
            in_indices.clear();
            if (indices.size() > 0)
            {
                if (indices[next_best_index].size() > 0)
                    in_indices = indices[next_best_index];
            }
            // Remove element from the lists
            clouds.erase(clouds.begin() + next_best_index);
            transforms.erase(transforms.begin() + next_best_index);
            transformed_locations.erase(transformed_locations.begin() + next_best_index);
            if (indices.size() > 0)
                indices.erase(indices.begin() + next_best_index);
        }
        else
        {
            ROS_WARN("squirrel_run_planner::main : next best index %u is invalid", next_best_index);
            break;
        }

        ++count;
        if (count >= max_iters)
        {
            ROS_ERROR("squirrel_run_planner::main : reached the maximum iterations");
            break;
        }
    }

    if (exp)
        delete exp;

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

