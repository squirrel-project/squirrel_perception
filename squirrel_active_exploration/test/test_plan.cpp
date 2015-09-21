#include "squirrel_active_exploration/active_exploration.h"
#include "squirrel_active_exploration/visualization_utils.h"

#define _SQUIRREL_DIRECTORY "/home/tpat8946/ros_ws/squirrel_active_exploration/src/squirrel_active_exploration"

using namespace std;
using namespace pcl;

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    // Seed the random number generator
    srand(time(NULL));

    // Set to true to visualize the planning
    bool do_visualize = false;
    bool do_save = false;
    int num_classes = 15;
    double variance = 0.5;  // using 0.5, smaller values means will find the closest point available, larger values means will go to max entropy location
    SIM_TYPE sim = MAX_AREA_UNOCCLUDED;
    int max_iters = 50;
    bool visualize_views_and_exit = false;

    //string test_dir = "/home/tpat8946/Data/TUW/willow_dataset_training_models_gt/test_set/T_01_willow_dataset";
    //bool reverse_transform = true;
    //bool load_segmentation_indices = false;
    string test_dir = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/training_data/fruchtmolke.pcd";
    string entropy_order_file = (string)_SQUIRREL_DIRECTORY + "/data/config/entropy_order/training_set_2/fruchtmolke.txt";
    bool reverse_transform = false;
    bool load_segmentation_indices = true;
    int num_objects = 1;
    int max_views_in_set = 16;  // 20 for asus_box, 16 for fruchtmolke

    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);
    if (do_visualize)
        exp->turn_on_visualization();
    if (do_save)
        exp->turn_on_saving(num_objects, num_classes);

    // Load test directory
    vector<Eigen::Vector4f> camera_poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<vector<int> > indices;
    if (load_segmentation_indices)
    {
        if (!load_model_training_directory(test_dir, reverse_transform, camera_poses, clouds, transforms, indices))
        {
            ROS_ERROR("TEST_plan::main : could not load test directory %s", test_dir.c_str());
            if (exp)
                delete exp;
            return EXIT_FAILURE;
        }
        // Remove some vector entries
        if (max_views_in_set < camera_poses.size())
        {
            camera_poses.erase(camera_poses.begin() + max_views_in_set, camera_poses.end());
            clouds.erase(clouds.begin() + max_views_in_set, clouds.end());
            transforms.erase(transforms.begin() + max_views_in_set, transforms.end());
            indices.erase(indices.begin() + max_views_in_set, indices.end());
        }
    }
    else
    {
        indices.clear();
        if (!load_test_directory(test_dir, reverse_transform, camera_poses, clouds, transforms))
        {
            ROS_ERROR("TEST_plan::main : could not load test directory %s", test_dir.c_str());
            if (exp)
                delete exp;
            return EXIT_FAILURE;
        }
    }
    // Transform the locations and get the current location
    Eigen::Vector4f current_location;
    vector<Eigen::Vector4f> transformed_locations;
    if (camera_poses.size() == 0)
    {
        ROS_ERROR("TEST_plan::main : no available map locations");
        if (exp)
            delete exp;
        return EXIT_FAILURE;
    }
    transformed_locations.resize(camera_poses.size());
    for (vector<Eigen::Vector4f>::size_type i = 0; i < camera_poses.size(); ++i)
        transformed_locations[i] = transform_eigvec(camera_poses[i], transforms[i]);

    // Visualize the locations
    if (visualize_views_and_exit)
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
        if (exp)
            delete exp;
        return EXIT_SUCCESS;
    }

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
    sensor_msgs::Image in_image;
    cv_ptr->toImageMsg(in_image);

    // Try to load the entropy_order file
    int start_index = 0;
    vector<int> view_order = load_entropy_order_file(entropy_order_file, test_dir, sim);
    bool valid_view_order = true;
    if (view_order.size() == 0)
    {
        ROS_WARN("TEST_plan::main : could not load entropy order file %s", entropy_order_file.c_str());
        valid_view_order = false;
    }
    else if (view_order.size() != clouds.size())
    {
        ROS_WARN("TEST_plan::main : loaded %lu views but expecting %lu", view_order.size(), clouds.size());
        valid_view_order = false;
    }
    if (valid_view_order)
    {
        start_index = view_order.back();
        view_order.pop_back();
        // If want to iterate from worst to best then need to reverse the vector
        if (sim == WORST_TO_BEST_ENTROPY || sim == WORST_TO_BEST_PROB)
            reverse(view_order.begin(), view_order.end());
    }
    else
    {
        // Iterate through the views in order of loaded files
        for (vector<PointCloud<PointT> >::size_type i = 1; i < clouds.size(); ++i)
            view_order.push_back(i);
    }

    // Extract the current data
    sensor_msgs::PointCloud2 in_cloud;
    toROSMsg(clouds[start_index], in_cloud);
    Eigen::Matrix4f in_transform = transforms[start_index];
    current_location = transformed_locations[start_index];
    vector<vector<int> > in_indices;
    if (indices.size() > 0)
        in_indices.push_back (indices[start_index]);
    // Remove element from the lists
    clouds.erase(clouds.begin() + start_index);
    transforms.erase(transforms.begin() + start_index);
    transformed_locations.erase(transformed_locations.begin() + start_index);
    if (indices.size() > 0)
        indices.erase(indices.begin() + start_index);
    // Subtract 1 from each new view_order number
    for (vector<int>::size_type i = 0; i < view_order.size(); ++i)
            view_order[i] = view_order[i] - 1;
    // Loop through the clouds in the directory
    int count = 0;
    while (ros::ok())
    {
        // Process the point cloud
        exp->set_data(in_cloud, in_image, in_transform);
        // Initialize visualizer
        if (!exp->initialize_visualization())
        {
            ROS_ERROR("TEST_plan::main : error in initializing visualization");
            return false;
        }
        if (load_segmentation_indices && in_indices.size() > 0)
        {
            ROS_INFO("TEST_plan::main : updating hypotheses");
            if (!exp->update_hypotheses(in_cloud, in_image, in_transform, in_indices))
            {
                ROS_ERROR("TEST_plan::main : error when updating hypotheses point cloud");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }
        else
        {
            // Update the hypotheses
            ROS_INFO("TEST_plan::main : updating hypotheses");
            if (!exp->update_hypotheses(in_cloud, in_image, in_transform))
            {
                ROS_ERROR("TEST_plan::main : error when updating hypotheses point cloud");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }

        if (do_visualize)
        {
            if (!exp->visualize())
            {
                ROS_ERROR("TEST_plan::main : error when visualizing the point cloud segments");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }

        // Plan
        // It will return the index for the next best view
        ROS_INFO("TEST_plan::main : planning");
        int next_best_index = start_index;
        if (sim == WORST_TO_BEST_ENTROPY || sim == BEST_TO_WORST_ENTROPY || sim == WORST_TO_BEST_PROB || sim == BEST_TO_WORST_PROB)
        {
            if (view_order.size() == 0)
            {
                ROS_ERROR("TEST_plan::main : no more available view points");
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
                ROS_ERROR("TEST_plan::main : error when planning");
                if (exp)
                    delete exp;
                return EXIT_FAILURE;
            }
        }
        if (next_best_index >= 0 && next_best_index <= transformed_locations.size())
        {
            // Get the new current location
            toROSMsg(clouds[next_best_index], in_cloud);
            in_transform = transforms[next_best_index];
            current_location = transformed_locations[next_best_index];
            in_indices.clear();
            if (indices.size() > 0)
                in_indices.push_back (indices[next_best_index]);
            // Remove element from the lists
            clouds.erase(clouds.begin() + next_best_index);
            transforms.erase(transforms.begin() + next_best_index);
            transformed_locations.erase(transformed_locations.begin() + next_best_index);
            if (indices.size() > 0)
                indices.erase(indices.begin() + next_best_index);
        }
        else
        {
            ROS_ERROR("TEST_plan::main : next best index %u is invalid", next_best_index);
            if (exp)
                delete exp;
            return EXIT_FAILURE;
        }

        ++count;
        if (count >= max_iters)
        {
            ROS_ERROR("TEST_plan::main : reached the maximum iterations");
            break;
        }

        return EXIT_SUCCESS;
    }

    if (exp)
        delete exp;

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

