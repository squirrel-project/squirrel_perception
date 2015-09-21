#include "squirrel_active_exploration/io_utils.h"
#include "squirrel_active_exploration/ground_removal.h"
#include "squirrel_active_exploration/pc_utils.h"

#define _TRANSFORM_NAME "transformation_"

using namespace std;
using namespace pcl;

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");
    ros::init(argc, argv, "refine_transforms");
    ros::NodeHandle nh ("~");

    string test_dir = "/home/tpat8946/Data/TUW/TUW_GH30_dataset/set_001";
    nh.getParam("directory", test_dir);
    ROS_INFO("refine_transforms::main : aligning the clouds in directory %s", test_dir.c_str());
    bool reverse_transform = false;

    // Load test directory
    vector<Eigen::Vector4f> camera_poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<int> ix_order;
    if (!load_test_directory(test_dir, reverse_transform, camera_poses, clouds, transforms, ix_order))
    {
        ROS_ERROR("refine_transforms::main : could not load test directory %s", test_dir.c_str());
        return EXIT_FAILURE;
    }

    PointCloud<PointT> initial_cloud;
    int initial_index = ix_order[0];
    transformPointCloud(clouds[initial_index], initial_cloud, transforms[initial_index]);
    PointCloud<PointT> initial_ground;
    vector<int> initial_ground_ix;
    PointCloud<PointT>::Ptr initial_nonground (new PointCloud<PointT>());
    get_ground_plane(initial_cloud, initial_ground_ix, initial_ground, *initial_nonground);
    for (size_t i = 1; i < ix_order.size(); ++i)
    {
        int ix = i;
        if (ix_order.size() > 0)
            ix = ix_order[i];

        // Convert to pcl
        PointCloud<PointT> t;;
        // Transform and save the cloud
        transformPointCloud(clouds[ix], t, transforms[ix]);
        // If the index is larger than 0 then refine the alignment with icp
        PointCloud<PointT> current_ground;
        vector<int> current_ground_ix;
        PointCloud<PointT>::Ptr current_nonground (new PointCloud<PointT>());
        if (get_ground_plane(t, current_ground_ix, current_ground, *current_nonground))
        {
            Eigen::Matrix4f new_transform;
            double score;
            if (icp(current_nonground, initial_nonground, new_transform, score))
            {
                // Save the combined transform
                Eigen::Matrix4f total_tf = new_transform * transforms[ix];
                // Get the index
                string str_ix = boost::lexical_cast<string>(i+1);
                while (str_ix.size() < 3)
                    str_ix = "0" + str_ix;
                cout << "saving " << str_ix << endl;
                // Write the transform
                string filename = add_backslash(test_dir) + _TRANSFORM_NAME + str_ix + ".txt";
                ofstream myfile (filename.c_str());
                if (myfile.is_open())
                {
                    myfile << total_tf(0,0) << " " << total_tf(0,1) << " " << total_tf(0,2) << " " << total_tf(0,3) << " "
                           << total_tf(1,0) << " " << total_tf(1,1) << " " << total_tf(1,2) << " " << total_tf(1,3) << " "
                           << total_tf(2,0) << " " << total_tf(2,1) << " " << total_tf(2,2) << " " << total_tf(2,3) << " "
                           << total_tf(3,0) << " " << total_tf(3,1) << " " << total_tf(3,2) << " " << total_tf(3,3) << endl;
                    myfile.close();
                }
            }
        }
    }

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

