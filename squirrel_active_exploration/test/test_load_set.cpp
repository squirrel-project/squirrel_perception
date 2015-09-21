#include "squirrel_active_exploration/active_exploration.h"
#include "squirrel_active_exploration/visualization_utils.h"

#define _ENTROPY_ORDER_FILE "entropy_order.txt"

using namespace std;
using namespace pcl;

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");
    int exit_code = EXIT_SUCCESS;

    // Seed the random number generator
    srand(time(NULL));

    bool save_pcds_and_exit = false;
    bool do_visualization = true;
    bool save_entropy_values = false;

    //string test_dir = "/home/tpat8946/Data/TUW/willow_dataset_training_models_gt/test_set/T_01_willow_dataset";
    //bool reverse_transform = true;
    //string test_dir = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set/set_00006";
    //bool reverse_transform = false;
    string test_dir = "/home/tpat8946/Data/TUW/willow_dataset_training_models_gt/test_set/T_01_willow_dataset";
    bool reverse_transform = true;
    bool load_segmentation_indices = true;
    int max_view_in_set = 16;  // 20 for asus_box, 16 for fruchtmolke

    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);
    exp->set_table_height_threshold(0.5);
    if (do_visualization)
        exp->turn_on_visualization();
    // Load test directory
    // Load test directory
    vector<Eigen::Vector4f> camera_poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<int> ix_order;
    vector<vector<vector<int> > > indices;
    if (load_segmentation_indices)
    {
        if (!load_test_directory_with_segment_indices(test_dir, reverse_transform, camera_poses, clouds, transforms, indices))
        {
            ROS_ERROR("TEST_plan::main : could not load test directory %s", test_dir.c_str());
            return EXIT_FAILURE;
        }
        // Remove some vector entries
        if (clouds.size() > max_view_in_set)
        {
            camera_poses.erase(camera_poses.begin() + max_view_in_set, camera_poses.end());
            clouds.erase(clouds.begin() + max_view_in_set, clouds.end());
            transforms.erase(transforms.begin() + max_view_in_set, transforms.end());
            indices.erase(indices.begin() + max_view_in_set, indices.end());
        }
    }
    else
    {
        indices.clear();
        if (!load_test_directory(test_dir, reverse_transform, camera_poses, clouds, transforms, ix_order))
        {
            ROS_ERROR("TEST_load_set::main : could not load test directory %s", test_dir.c_str());
            return EXIT_FAILURE;
        }
    }

    if (save_pcds_and_exit)
    {
        // View all clouds together
        vector<PointCloud<PointT>::Ptr> v_clouds;
        vector<PointCloud<PointT>::Ptr> p_clouds;
//        PointCloud<PointT> initial_cloud;
//        int initial_index = ix_order[0];
//        cout << "START INDEX " << initial_index << endl;
//        transformPointCloud(clouds[initial_index], initial_cloud, transforms[initial_index]);
//        PointCloud<PointT> initial_ground;
//        vector<int> initial_ground_ix;
//        PointCloud<PointT>::Ptr initial_nonground (new PointCloud<PointT>());
//        get_ground_plane(initial_cloud, initial_ground_ix, initial_ground, *initial_nonground);
//        string f = "/home/tpat8946/view_0.pcd";
//        io::savePCDFileBinary<PointT>(f, initial_cloud);
        string f;
        for (size_t i = 0; i < clouds.size(); ++i)
        {
            int ix = i;
            if (ix_order.size() > 0)
                ix = ix_order[i];
            //if (ix != initial_index)
            //{
                cout << "view " << ix << endl;
                //f = "/home/tpat8946/original_" + boost::lexical_cast<string>(i+1) + ".pcd";
                //io::savePCDFileBinary<PointT>(f,clouds[ix]);
                // Convert to pcl
                PointCloud<PointT>::Ptr t (new PointCloud<PointT>());
                // Transform and save the cloud
                transformPointCloud(clouds[ix], *t, transforms[ix]);
                f = "/home/tpat8946/view_" + boost::lexical_cast<string>(i+1) + ".pcd";
                io::savePCDFileBinary<PointT>(f,*t);
                // Create a cloud and transform
                PointCloud<PointT>::Ptr p (new PointCloud<PointT>());
                p->resize(1);
                p->points[0].x = camera_poses[ix][0];
                p->points[0].y = camera_poses[ix][1];
                p->points[0].z = camera_poses[ix][2];
                transformPointCloud(*p,*p,transforms[ix]);
                // Write to file
                v_clouds.push_back (t);
                v_clouds.push_back (p);
                p_clouds.push_back (p);
            //}
            //else
            //{
            //    cout << "Skipping index " << ix << endl;
            //}
        }
        visualization_u::visualize_point_cloud(p_clouds);
        //visualize_point_cloud(v_clouds);
        return EXIT_SUCCESS;
    }

    // ================================= //
    // PROCESS POINT CLOUDS
    // ================================= //

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

    // Write the directory name to the front of the entropy order file
    if (save_entropy_values)
    {
        // Save to file
        ofstream out;
        out.open(_ENTROPY_ORDER_FILE);
        // Write results
        if (out.is_open())
        {
            // Write the directory name
            out << test_dir << endl;
            out.close();
        }
        else
        {
            ROS_WARN("TEST_load_set::main : could not open file to save entropy to");
        }
    }

    // Loop through the point clouds in the directory
    int i = 0;
    while (ros::ok() && i < clouds.size())
    {
        // Process the point cloud
        sensor_msgs::PointCloud2 in_cloud;
        toROSMsg(clouds[i], in_cloud);
        exp->set_data(in_cloud, in_image, transforms[i]);
        // Initialize visualizer
        if (!exp->initialize_visualization())
        {
            ROS_ERROR("TEST_load_set::main : error in initializing visualization");
            return false;
        }
        if (load_segmentation_indices && indices.size() > 0)
        {
            ROS_INFO("TEST_load_set::main : updating hypotheses with segments");
            if (!exp->update_hypotheses(in_cloud, in_image, transforms[i], indices[i]))
            {
                ROS_ERROR("TEST_load_set::main : error when updating hypotheses point cloud");
                return EXIT_FAILURE;
            }
        }
        else
        {
            // Update the hypotheses
            ROS_INFO("TEST_load_set::main : updating hypotheses");
            if (!exp->update_hypotheses(in_cloud, in_image, transforms[i]))
            {
                ROS_ERROR("TEST_load_set::main : error when updating hypotheses point cloud");
                return EXIT_FAILURE;
            }
        }
        if (do_visualization)
        {
            if(!exp->visualize())
            {
                ROS_ERROR("TEST_load_set::main : error when visualizing the point cloud segments");
                return EXIT_FAILURE;
            }
        }
        if (save_entropy_values)
        {
            // Save to file
            ofstream out;
            out.open(_ENTROPY_ORDER_FILE, ios::app);
            // Write results
            if (out.is_open())
            {
                // Write the index i
                out << i;
                // Write the total entropy
                vector<double> ent = exp->get_entropies();
                double ent_sum = 0;
                for (vector<double>::size_type i = 0; i < ent.size(); ++i)
                    ent_sum += ent[i];
                out << " " << ent_sum << endl;
                out.close();
            }
            else
            {
                ROS_WARN("TEST_load_set::main : could not open file to save entropy to");
            }
            // Clear the data
            exp->clear();
        }
        ++i;
    }

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return exit_code;
}
