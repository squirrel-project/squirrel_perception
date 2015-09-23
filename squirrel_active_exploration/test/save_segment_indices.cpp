#include "squirrel_active_exploration/active_exploration.h"
#include "squirrel_active_exploration/visualization_utils.h"

#define _ENTROPY_ORDER_FILE "entropy_order.txt"

using namespace std;
using namespace pcl;

bool initialize_visualization(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, ros::ServiceClient &viz_init_client)
{
    ROS_INFO("save_segment_indices::initialize_visualization : starting");
    if (cloud.data.size() > 0 && image.data.size() > 0)
    {
        squirrel_object_perception_msgs::SegmentVisualizationInit viz_init_srv;
        viz_init_srv.request.cloud = cloud;
        viz_init_srv.request.saliency_map = image;
        if (!viz_init_client.call(viz_init_srv))
        {
            ROS_ERROR("save_segment_indices::initialize_visualization : could not initialize visualizer");
            return false;
        }
    }
    else
    {
        ROS_WARN("save_segment_indices::initialize_visualization : skipping visualization initialization, cloud has %lu points and image has %lu pixels",
                 cloud.data.size(), image.data.size());
        return false;
    }

    ROS_INFO("save_segment_indices::initialize_visualization : finished");
    return true;
}

bool initialize_visualization(const PointCloud<PointT> &cloud, const sensor_msgs::Image &image, ros::ServiceClient &viz_init_client)
{
    sensor_msgs::PointCloud2 in_cloud;
    toROSMsg(cloud, in_cloud);
    return initialize_visualization(in_cloud, image, viz_init_client);
}

bool visualize_segmentation(const string &dir, const ActiveExploration *exp, const int &index,
                            ros::ServiceClient &viz_client, ros::ServiceClient &viz_init_client)
{
    ROS_INFO("save_segment_indices::visualize_segmentation : starting for index %u", index);
    vector<vector<int> > segments = exp->get_segments();
    squirrel_object_perception_msgs::SegmentVisualizationOnce viz_srv;
    int save_count = 0;
    string index_str = boost::lexical_cast<string>(index);
    while (index_str.size() != 10)
        index_str = "0" + index_str;
    string input;
    string filename;

    // Go through segments in order of size
    vector<pair<int,int> > num_els;
    for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        num_els.push_back(make_pair(i,segments[i].size()));
    sort(num_els.begin(), num_els.end(), compare<int>);
    reverse(num_els.begin(), num_els.end());
    vector<int> seg_order;
    for (vector<pair<int,int> >::size_type i = 0; i < num_els.size(); ++i)
        seg_order.push_back(num_els[i].first);

    vector<std_msgs::Int32MultiArray> saved_segs;

    for (vector<int>::size_type i = 0; i < seg_order.size(); ++i)
    {
        if (saved_segs.size() > 0)
        {
            for (size_t j = 0; j < saved_segs.size(); ++j)
            {
                vector<std_msgs::Int32MultiArray> cluster_indices;
                cluster_indices.push_back(saved_segs[j]);
                viz_srv.request.clusters_indices = cluster_indices;
                viz_client.call(viz_srv);
            }
        }
        bool clear_vis = true;
        // Wait for input
        cout << "Display segment ";
        cin.ignore();
        int ix = seg_order[i];
        std_msgs::Int32MultiArray seg;
        if (segments[ix].size() > 0)
        {
            seg.data = segments[ix];
            vector<std_msgs::Int32MultiArray> cluster_indices;
            cluster_indices.push_back (seg);
            viz_srv.request.clusters_indices = cluster_indices;
            if (viz_client.call(viz_srv))
            {
                // Print the size of the segment
                ROS_INFO("Segment %u : %lu", ix, segments[ix].size());
                cout << "Save segment?" << endl;
                cin >> input;
                // If "y" input
                if (input.compare("y") == 0 || input.compare("yes") == 0 ||
                    input.compare("Y") == 0 || input.compare("YES") == 0)
                {
                    string save_count_str = boost::lexical_cast<string>(save_count);
                    while (save_count_str.size() != 2)
                        save_count_str = "0" + save_count_str;
                    filename = add_backslash(dir) + _INDICES_PREFIX + save_count_str + "_" + index_str + ".pcd";
                    PointCloud<IndexPoint> cloud;
                    cloud.resize(segments[ix].size());
                    for (vector<int>::size_type j = 0; j < segments[ix].size(); ++j)
                        cloud.points[j].idx = segments[ix][j];
                    if (io::savePCDFileBinary<IndexPoint>(filename.c_str(), cloud) == -1)
                    {
                        ROS_WARN("save_segment_indices::visualize_segmentation : could not write index file");
                    }
                    ++save_count;
                    saved_segs.push_back(seg);
                    clear_vis = false;
                }
                else if (input.compare("q") == 0 || input.compare("quit") == 0 ||
                         input.compare("Q") == 0 || input.compare("QUIT") == 0)
                {
                    break;
                }
            }
            else
            {
                ROS_INFO("save_segment_indices::visualize_segmentation : could not visualize segment %lu", i);
                return false;
            }
        }
        else
        {
            ROS_WARN("save_segment_indices::visualize_segmentation : error with segment");
        }
        // Clear the visualization
        if (clear_vis)
            initialize_visualization(exp->get_cloud(), exp->get_image(), viz_init_client);
    }
    cout << "Viewing all segment saved to file" << endl;
    cin.ignore();
    // Clear the visualization
    initialize_visualization(exp->get_cloud(), exp->get_image(), viz_init_client);
    // Load the directory and the segments
    vector<Eigen::Vector4f> camera_poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<vector<vector<int> > > segment_indices;
    bool reverse_transform = true;
    if (!load_test_directory_with_segment_indices(dir, reverse_transform, camera_poses, clouds, transforms, segment_indices, index))
    {
        ROS_ERROR("save_segment_indices::visualize_segmentation : could not load test directory %s", dir.c_str());
        return false;
    }
    if (segment_indices.size() == 0)
    {
        ROS_ERROR("save_segment_indices::visualize_segmentation : segment indices is empty");
        return false;
    }
    for (vector<vector<int> >::size_type i = 0; i < segment_indices[0].size(); ++i)
    {
        std_msgs::Int32MultiArray seg;
        if (segment_indices[0][i].size() > 0)
        {
            seg.data = segment_indices[0][i];
            vector<std_msgs::Int32MultiArray> cluster_indices;
            cluster_indices.push_back (seg);
            viz_srv.request.clusters_indices = cluster_indices;
            if (viz_client.call(viz_srv))
            {
                // Print the size of the segment
                ROS_INFO("Segment %lu : %lu", i, segment_indices[0][i].size());
            }
            else
            {
                ROS_INFO("save_segment_indices::visualize_segmentation : could not visualize segment %lu", i);
                return false;
            }
        }
        else
        {
            ROS_WARN("save_segment_indices::visualize_segmentation : error with segment");
        }
    }
    cout << "Continue with processing" << endl;
    cin.ignore();

    ROS_INFO("save_segment_indices::visualize_segmentation : finished");
    return true;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING SAVE SEGMENT INDICES ***");

    // Seed the random number generator
    srand(time(NULL));

    bool do_visualization = true;
    bool reverse_transform = true;

    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);
    exp->set_table_height_threshold(0.5);
    if (do_visualization)
        exp->turn_on_visualization();

    ros::NodeHandle *n = exp->get_ros_node_handle();

    ros::ServiceClient viz_init_client = n->serviceClient<squirrel_object_perception_msgs::SegmentVisualizationInit>("/squirrel_segmentation_visualization_init");
    ros::ServiceClient viz_client = n->serviceClient<squirrel_object_perception_msgs::SegmentVisualizationOnce>("/squirrel_segmentation_visualization_once");

    // Get the directory
    string test_dir;
    if (!n->getParam("directory", test_dir))
    {
        ROS_ERROR("save_segment_indices::main : You must enter a directory");
        if (exp)
            delete exp;
        return EXIT_FAILURE;
    }
    // Get a fake image file for visualization
    string image_file;
    if (!n->getParam("image_file", umage_file))
    {
        ROS_ERROR("save_segment_indices::main : You must enter an image file for visualization");
        if (exp)
            delete exp;
        return EXIT_FAILURE;
    }

    // Load test directory
    vector<Eigen::Vector4f> camera_poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<int> ix_order;
    if (!load_test_directory(test_dir, reverse_transform, camera_poses, clouds, transforms, ix_order))
    {
        ROS_ERROR("save_segment_indices::main : could not load test directory %s", test_dir.c_str());
        return EXIT_FAILURE;
    }

    // Load the image
    cv::Mat image = cv::imread(image_file,-1);
    cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // Convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "saliency_map";
    cv_ptr->encoding = "mono8";
    cv_ptr->image = image;
    sensor_msgs::Image in_image;
    cv_ptr->toImageMsg(in_image);

    // Loop through the point clouds in the directory
    int i = 0;
    while (ros::ok() && i < clouds.size())
    {
        // Process the point cloud
        sensor_msgs::PointCloud2 in_cloud;
        toROSMsg(clouds[ix_order[i]], in_cloud);
        exp->set_data(in_cloud, in_image, transforms[ix_order[i]]);
        // Initialize visualizer
        if (!initialize_visualization(in_cloud, in_image, viz_init_client))
        {
            ROS_ERROR("save_segment_indices::main : error in initializing visualization");
            return EXIT_FAILURE;
        }
        // Segment the point cloud
        if (!exp->segment())
        {
            ROS_ERROR("save_segment_indices::main : error in segmentation");
            return EXIT_FAILURE;
        }
        if (!visualize_segmentation(test_dir, exp, i, viz_client, viz_init_client))
        {
            ROS_ERROR("save_segment_indices::main : error when visualizing the point cloud segments");
            return EXIT_FAILURE;
        }
        // Clear the data
        exp->clear();
        ++i;
    }

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

