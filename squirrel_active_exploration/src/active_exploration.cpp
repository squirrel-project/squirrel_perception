#include "squirrel_active_exploration/active_exploration.h"

using namespace std;
using namespace pcl;
using namespace squirrel_object_perception_msgs;
using namespace octomap;

/* === CONSTRUCTORS, DESTRUCTORS AND INITIALIZATION === */

ActiveExploration::ActiveExploration()
    : _transform (Eigen::Matrix4f::Identity()),
      _current_transform (Eigen::Matrix4f::Identity()),
      _received_cloud (false),
      _received_image (false),
      _received_transform (false),
      _num_clouds (0),
      _save_dir (_SAVE_DIRECTORY),
      _max_object_distance (_MAX_OBJECT_DISTANCE),
      _min_object_height (_MIN_OBJECT_HEIGHT),
      _max_object_height (_MAX_OBJECT_HEIGHT),
      _min_object_length (_MIN_OBJECT_LENGTH),
      _max_object_length (_MAX_OBJECT_LENGTH),
      _table_height_threshold (-_TABLE_HEIGHT_THRESHOLD),
      _tree (_DEFAULT_TREE_RESOLUTION),
      _visualization_on(false),
      _saving_on(false),
      _expected_num_objects(-1),
      _expected_num_classes(-1),
      _voxel_overlap_threshold (_VOXEL_OVERLAP_THRESHOLD)
{}

ActiveExploration::ActiveExploration(const ActiveExploration &that)
    : _position (that.get_position()),
      _cloud (that.get_cloud()),
      _transformed_cloud (that.get_transformed_cloud()),
      _current_cloud (that.get_current_cloud()),
      _image (that.get_image()),
      _current_image (that.get_current_image()),
      _saliency_map (that.get_saliency_map()),
      _current_saliency_map (that.get_current_saliency_map()),
      _transform (that.get_transform()),
      _current_transform (that.get_current_transform()),
      _received_cloud (that.get_flag_received_cloud()),
      _received_image (that.get_flag_received_image()),
      _received_transform (that.get_flag_received_transform()),
      _segments (that.get_segments()),
      _segment_octree_keys (that.get_segment_octree_keys()),
      _poses (that.get_poses()),
      _class_estimates (that.get_class_estimates()),
      _instance_directories (that.get_instance_directories()),
      _instances_to_map_tfs(that.get_instances_to_map_tfs()),
      _emaps(that.get_emaps()),
      _entropies (that.get_entropies()),
      _entropy_ranking (that.get_entropy_ranking()),
      _num_clouds (that.get_num_clouds()),
      _save_dir (that.get_save_directory()),
      _max_object_distance (that.get_max_object_distance()),
      _min_object_height (that.get_min_object_height()),
      _max_object_height (that.get_max_object_height()),
      _min_object_length (that.get_min_object_length()),
      _max_object_length (that.get_max_object_length()),
      _table_height_threshold (that.get_table_height_threshold()),
      _tree (that.get_octree()),
      _visualization_on (false),
      _saving_on (false),
      _expected_num_objects (that.get_expected_num_objects()),
      _expected_num_classes (that.get_expected_num_classes()),
      _voxel_overlap_threshold (that.get_voxel_overlap_threshold())
{
    //initialize(that.get_ros_node_handle());
}

ActiveExploration::~ActiveExploration()
{
    // Delete the ros node handle
    if (_n)
        delete _n;
    // Clear all data
    clear();
}

ActiveExploration& ActiveExploration::operator=(const ActiveExploration &that)
{
    _position = that.get_position();
    _cloud = that.get_cloud();
    _transformed_cloud = that.get_transformed_cloud();
    _current_cloud = that.get_current_cloud();
    _image = that.get_image();
    _current_image = that.get_current_image();
    _saliency_map = that.get_saliency_map();
    _current_saliency_map = that.get_current_saliency_map();
    _transform = that.get_transform();
    _current_transform = that.get_current_transform();
    _received_cloud = that.get_flag_received_cloud();
    _received_image = that.get_flag_received_image();
    _received_transform = that.get_flag_received_transform();
    _segments = that.get_segments();
    _segment_octree_keys = that.get_segment_octree_keys();
    _poses = that.get_poses();
    _class_estimates = that.get_class_estimates();
    _instance_directories = that.get_instance_directories();
    _instances_to_map_tfs = that.get_instances_to_map_tfs();
    _emaps = that.get_emaps();
    _entropies = that.get_entropies();
    _entropy_ranking = that.get_entropy_ranking();
    _num_clouds = that.get_num_clouds();
    _save_dir = that.get_save_directory();
    _max_object_distance = that.get_max_object_distance();
    _min_object_height = that.get_min_object_height();
    _max_object_height = that.get_max_object_height();
    _min_object_length = that.get_min_object_length();
    _max_object_length = that.get_max_object_length();
    _table_height_threshold = that.get_table_height_threshold();
    _visualization_on = false;
    _saving_on = false;
    _expected_num_objects = that.get_expected_num_objects();
    _expected_num_classes = that.get_expected_num_classes();
    _voxel_overlap_threshold = that.get_voxel_overlap_threshold();
    //_tree = that.get_octree();
    //initialize(that.get_ros_node_handle());
}

bool ActiveExploration::initialize(int argc, char **argv)
{
    ROS_INFO("ActiveExploration::initialize : starting");
    ros::init(argc, argv, "active_exploration");
    ros::NodeHandle *node (new ros::NodeHandle("~"));
    clear();
    return initialize (node);
}

bool ActiveExploration::initialize(ros::NodeHandle *node)
{
    // Assign the ros node handle
    _n = node;

    // Set up the subscribers
    _cloud_sub = _n->subscribe("/kinect/depth_registered/points", 1, &ActiveExploration::point_cloud_callback, this);
    _image_sub = _n->subscribe("/kinect/rgb/image_raw", 5, &ActiveExploration::image_callback, this);

    // Set up the publishers

    // Set up the service clients
    _seg_client = _n->serviceClient<Segment>("/squirrel_segmentation");
    _viz_init_client = _n->serviceClient<SegmentVisualizationInit>("/squirrel_segmentation_visualization_init");
    _viz_client = _n->serviceClient<SegmentVisualizationOnce>("/squirrel_segmentation_visualization_once");
    _class_client = _n->serviceClient<Classify>("/squirrel_classify");
    _em_client = _n->serviceClient<squirrel_object_perception_msgs::EntropyMap>("/squirrel_entropy_map");
    _em_viz_client = _n->serviceClient<squirrel_object_perception_msgs::EntropyMapViz>("/squirrel_entropy_map_visualize");

    // Read the input if it exists
    string cloud_name = "";
    _n->getParam ("cloud_name", cloud_name);
    string saliency_name = "";
    _n->getParam ("saliency_name", saliency_name);
    string transform_name = "";
    _n->getParam ("transform_name", transform_name);
    _save_dir = _SAVE_DIRECTORY;
    _n->getParam ("save_directory", _save_dir);
    _save_dir = add_backslash(_save_dir);
    string data_set_name = "";
    _n->getParam("data_directory", data_set_name);
    // Extract the last part of the file
    if (data_set_name.size() > 0)
    {
        // Remove the last '/'
        data_set_name = rem_backslash(data_set_name);
        size_t found = data_set_name.find_last_of('/');
        if (found != string::npos)
        {
            string dname = data_set_name.substr(found+1);
            if (dname.size() > 0)
                data_set_name = dname;
            if (data_set_name.size() > 0)
            {
                _save_dir = _save_dir + data_set_name;
                _save_dir = add_backslash(_save_dir);
            }
        }
    }
    // Extract the start index
    int start_index = -10;
    string start_subdir = "";
    if (_n->getParam ("start_index", start_index))
    {
        // Convert start_index int to a string
        if (start_index == -1)
        {
            start_subdir = "index_min_ent";
        }
        else
        {
            stringstream ss;
            ss << start_index;
            start_subdir = ss.str();
        }
    }
    if (start_subdir.size() > 0)
    {
        _save_dir = _save_dir + start_subdir;
        _save_dir = add_backslash(_save_dir);
    }
    string plan_type = "";
    _n->getParam ("plan_type", plan_type);
    if (plan_type.size() > 0)
    {
        _save_dir = _save_dir + plan_type;
        _save_dir = add_backslash(_save_dir);
    }
    ROS_INFO("ActiveExploration::initialize : saving data to");
    printf("%s\n", _save_dir.c_str());
    _max_object_distance = _MAX_OBJECT_DISTANCE;
    _n->getParam ("max_object_distance", _max_object_distance);
    _min_object_height = _MIN_OBJECT_HEIGHT;
    _n->getParam ("min_object_height", _min_object_height);
    _max_object_height = _MAX_OBJECT_HEIGHT;
    _n->getParam ("max_object_height", _max_object_height);
    _min_object_length = _MIN_OBJECT_LENGTH;
    _n->getParam ("min_object_length", _min_object_length);
    _max_object_length = _MAX_OBJECT_LENGTH;
    _n->getParam ("max_object_length", _max_object_length);
    _table_height_threshold = _TABLE_HEIGHT_THRESHOLD;
    _n->getParam ("table_height_threshold", _table_height_threshold);
    _voxel_overlap_threshold = _VOXEL_OVERLAP_THRESHOLD;
    _n->getParam ("voxel_overlap", _voxel_overlap_threshold);

    ROS_INFO("ActiveExploration::initialize : finihsed");
    return read_input(cloud_name, transform_name, saliency_name);
}

void ActiveExploration::clear()
{
    _cloud.clear();
    _transformed_cloud.clear();
    _transform = Eigen::Matrix4f::Identity();
    _image.data.clear();
    _saliency_map.data.clear();
    clear_sensor_input();
    clear_hypotheses();
    _num_clouds = 0;
    _tree.clear();
}

void ActiveExploration::clear_sensor_input()
{
    _received_cloud = false;
    _received_image = false;
    _received_transform = false;
    _current_cloud.data.clear();
    _current_image.data.clear();
    _current_saliency_map.data.clear();
    _current_transform = Eigen::Matrix4f::Identity();
}

void ActiveExploration::clear_hypotheses()
{
    _segments.clear();
    _segment_octree_keys.clear();
    _poses.clear();
    _class_estimates.clear();
    _instance_directories.clear();
    _instances_to_map_tfs.clear();
    _emaps.clear();
    _entropies.clear();
    _entropy_ranking.clear();
}

/* === CALLBACKS === */

void ActiveExploration::point_cloud_callback(const sensor_msgs::PointCloud2Ptr &msg)
{
    ROS_INFO("ActiveExploration::point_cloud_callback : point cloud callback");
    // Hack to organise the point cloud
    PointCloud<PointT>::Ptr frame (new PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *frame);
    if (frame->height == 1)
    {
        if (frame->points.size() == 640*480)
        {
            frame->height = 480;
            frame->width = 640;
        }
    }
    // Convert to ROS sensor_msgs::PointCloud2Ptr
    toROSMsg(*frame,_current_cloud);
    _received_cloud = true;
}

void ActiveExploration::image_callback(const sensor_msgs::ImagePtr &msg)
{
    ROS_INFO("ActiveExploration::image_callback : image callback");
    _current_image = *msg;
    _received_image = true;
}

/* === IO === */

bool ActiveExploration::read_input(const string &cloud_name, const string &transform_name, const string &saliency_name)
{
    _received_cloud = false;
    _received_image = false;
    _received_transform = false;

    // Load the file
    if (io::loadPCDFile<PointT> (cloud_name.c_str(), _cloud) == -1)
        ROS_WARN("ActiveExploration::read_input : could not read input cloud filename %s", cloud_name.c_str());
    else
        ++_num_clouds;
    ROS_INFO("ActiveExploration::read_input : read %lu points", _cloud.size());
    // Load the transform file
    Eigen::Matrix4f in_transform;
    if (!read_tf_file(transform_name, in_transform))
    {
        ROS_WARN("ActiveExploration::read_input : could not read input transform filename %s", transform_name.c_str());
        _transform = Eigen::Matrix4f::Identity();
    }
    else
    {
        _transform = in_transform.inverse();
    }
    transformPointCloud(_cloud, _transformed_cloud, _transform);
    compute_position();
    if (!add_to_octree())
        ROS_WARN("ActiveExploration::read_input : could not add point cloud to octree");

    // Read the saliency map
    cv::Mat saliency = cv::imread(saliency_name,-1);
    cv_bridge::CvImagePtr cv_ptr (new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // Convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "saliency_map";
    cv_ptr->encoding = "mono8";
    cv_ptr->image = saliency;
    cv_ptr->toImageMsg(_saliency_map);
    ROS_INFO("ActiveExploration::read_input : read %lu pixels", _saliency_map.data.size());
    // Assign this as the current image
    _image = _saliency_map;

    // Initialize visualizer
    if (!initialize_visualization())
    {
        ROS_ERROR("ActiveExploration::read_input : could not initialize visualizer");
        return false;
    }

    _received_cloud = true;
    _received_image = true;

    return true;
}

bool ActiveExploration::data_from_sensor(const Eigen::Matrix4f &transform)
{
    // Read the input from the sensor by litening to the publishers
    int count = 0;  // count the number of times the callbacks will be processed
    ros::Rate r(_R_RATE);  // rate in Hz
    // Loop until valid input from the sensor
    while (!valid_input())
    {
        // Process all callbacks
        ros::spinOnce();
        // Sleep
        r.sleep();
        // Increment counter
        ++count;
        // If reached the maximum allowed number of time outs
        if (count >= _N_TIMEOUT)
        {
            // Unsuccessfully received data from sensor
            ROS_ERROR("ActiveExploration::data_from_sensor : could not read input)");
            return false;
        }
    }
    // Successfully received data from the sensor
    ROS_INFO("ActiveExploration::data_from_sensor : successfully received data from sensor");
    ROS_INFO("ActiveExploration::data_from_sensor : %lu points in cloud", _current_cloud.data.size());
    ROS_INFO("ActiveExploration::data_from_sensor : %lu pixels in image", _current_image.data.size());

    _transform = transform;
    _received_transform = true;

    pcl::fromROSMsg(_current_cloud, _cloud);
    transformPointCloud(_cloud, _transformed_cloud, _transform);
    compute_position();
    if (!add_to_octree())
        ROS_WARN("ActiveExploration::data_from_sensor : could not add point cloud to octree");
    _image = _current_image;

    ++_num_clouds;

    // Initialize visualizer
    if (!initialize_visualization())
    {
        ROS_ERROR("ActiveExploration::data_from_sensor : could not initialize visualizer");
        return false;
    }
}

bool ActiveExploration::valid_input()
{
    return (valid_cloud() && valid_image());
}

bool ActiveExploration::valid_cloud()
{
    return (_received_cloud && _current_cloud.data.size() > 0);
}

bool ActiveExploration::valid_image()
{
    return (_received_image && _current_image.data.size() > 0);
}

/* === PROCESS === */

bool ActiveExploration::process()
{
    ROS_INFO("ActiveExploration::process : starting");
    // Clear the hypotheses
    clear_hypotheses();
    // Segment
    if (!segment())
    {
        ROS_ERROR("ActiveExploration::process : error in segmentation");
        return false;
    }
    // Filter out ground, table and objects far away
    if (!filter_segments())
    {
        ROS_ERROR("ActiveExploration::process : error in filtering segmentation");
        return false;
    }
    // Get the keys in the tree for each each segment
    if (!extract_segment_octree_keys())
    {
        ROS_ERROR("ActiveExploration::process : error in extracting segment tree keys");
        return false;
    }
    // Estimate poses
    if (!estimate_pose())
    {
        ROS_ERROR("ActiveExploration::process : error in estimating poses");
        return false;
    }
    // Classify
    if (!classify())
    {
        ROS_ERROR("ActiveExploration::process : error in classification");
        return false;
    }
    // Get the entropy maps for the segments and instance directories
    if (!retrieve_entropy_maps())
    {
        ROS_ERROR("ActiveExploration::process : error in retrieving the entropy maps");
        return false;
    }
    // Compute entropy
    if (!compute_entropy())
    {
        ROS_ERROR("ActiveExploration::process : error in computing entropy");
        return false;
    }
    // Rank entropies
    if (!rank_entropy())
    {
        ROS_ERROR("ActiveExploration::process : error in ranking entropy");
        return false;
    }
    // Save data to file
    if (_saving_on)
    {
        if (!save_results())
        {
            ROS_ERROR("ActiveExploration::process : error in saving results");
            return false;
        }
//        if (!save_octree())
//        {
//            ROS_ERROR("ActiveExploration::process : error in saving octree");
//            return false;
//        }
        if (!save_object_details())
        {
            ROS_ERROR("ActiveExploration::process : error in saving object details");
            return false;
        }
    }
    // Successfully processed input
    ROS_INFO("ActiveExploration::process : finihsed");
    return true;
}

bool ActiveExploration::process(const vector<vector<int> > &segment_indices)
{
    ROS_INFO("ActiveExploration::process : starting");
    // Clear the hypotheses
    clear_hypotheses();
    // Segment
    if (segment_indices.size() > 0)
    {
        if (!segment(segment_indices))
        {
            ROS_ERROR("ActiveExploration::process : error in segmentation");
            return false;
        }
    }
    else
    {
        if (!segment())
        {
            ROS_ERROR("ActiveExploration::process : error in segmentation");
            return false;
        }
        // Filter out ground, table and objects far away
        if (!filter_segments())
        {
            ROS_ERROR("ActiveExploration::process : error in filtering segmentation");
            return false;
        }
    }
    // Get the keys in the tree for each each segment
    if (!extract_segment_octree_keys())
    {
        ROS_ERROR("ActiveExploration::process : error in extracting segment tree keys");
        return false;
    }
    // Estimate poses
    if (!estimate_pose())
    {
        ROS_ERROR("ActiveExploration::process : error in estimating poses");
        return false;
    }
    // Classify
    if (!classify())
    {
        ROS_ERROR("ActiveExploration::process : error in classification");
        return false;
    }
    // Get entropy maps
    if (!retrieve_entropy_maps())
    {
        ROS_ERROR("ActiveExploration::process : error in retrieving the entropy maps");
        return false;
    }
    // Compute entropy
    if (!compute_entropy())
    {
        ROS_ERROR("ActiveExploration::process : error in computing entropy");
        return false;
    }
    // Rank entropies
    if (!rank_entropy())
    {
        ROS_ERROR("ActiveExploration::process : error in ranking entropy");
        return false;
    }
    // Save data to file
    if (_saving_on)
    {
        if (!save_results())
        {
            ROS_ERROR("ActiveExploration::process : error in saving results");
            return false;
        }
//        if (!save_octree())
//        {
//            ROS_ERROR("ActiveExploration::process : error in saving octree");
//            return false;
//        }
        if (!save_object_details())
        {
            ROS_ERROR("ActiveExploration::process : error in saving object details");
            return false;
        }
    }
    // Successfully processed input
    ROS_INFO("ActiveExploration::process : finihsed");
    return true;
}

bool ActiveExploration::process(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const Eigen::Matrix4f &transform,
                                const std::vector<std::vector<int> > &segment_indices)
{
    // Set the data
    set_data(cloud, image, transform);
    // Initialize visualizer
    if (!initialize_visualization())
    {
        ROS_ERROR("ActiveExploration::process : could not initialize visualizer");
        return false;
    }
    // Process the cloud
    if (segment_indices.size() == 0)
        return process();
    else
        return process(segment_indices);
}

bool ActiveExploration::process(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const vector<vector<int> > &segment_indices)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    return process (cloud, image, transform, segment_indices);
}

bool ActiveExploration::process(const ros_CloudTX &ctx, const sensor_msgs::Image &image, const vector<vector<int> > &segment_indices)
{
    return process (ctx._cloud, image, ctx._transform, segment_indices);
}

bool ActiveExploration::process_current_cloud()
{
    // Get the current input
    clear_sensor_input();
    // Process the cloud
    return process();
}

/* === SEGMENTATION === */

bool ActiveExploration::segment()
{
    ROS_INFO("ActiveExploration::segment : starting");
    // If the cloud has point
    if (_cloud.size() > 0)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(_cloud, msg);
        // Call the segmentation service
        _seg_srv.request.cloud = msg;
        if (!_seg_client.call(_seg_srv))
        {
            ROS_ERROR("ActiveExploration::segment : could not call the segmentation service");
            return false;
        }
        _segments.clear();
        _segments.resize(_seg_srv.response.clusters_indices.size());
        for (size_t i = 0; i < _seg_srv.response.clusters_indices.size(); ++i)
            _segments[i] = _seg_srv.response.clusters_indices[i].data;
        // If successful segmentation
        if (_segments.size() == 0)
        {
            ROS_ERROR("ActiveExploration::segment : cloud has zero segments");
            return false;
        }
    }
    // Otherwise cannot segment an empty point cloud
    else
    {
        ROS_ERROR("ActiveExploration::segment : cloud is empty");
        return false;
    }

//    if (!active_exploration_utils::segment(_cloud, _seg_client, _segments))
//    {
//        ROS_ERROR("ActiveExploration::segment : could not segment the point cloud");
//        return false;
//    }

    ROS_INFO("ActiveExploration::segment : successfully segmented point cloud into %lu segments", _segments.size());
    ROS_INFO("ActiveExploration::segment : finished");
    return true;
}

bool ActiveExploration::segment(const vector<vector<int> > &segment_indices)
{
    ROS_INFO("ActiveExploration::segment : starting");
//    // If the cloud has point
//    int cloud_size = _cloud.size();
//    if (cloud_size > 0)
//    {
//        _segments.clear();
//        for (vector<vector<int> >::const_iterator it = segment_indices.begin(); it != segment_indices.end(); ++it)
//        {
//            std_msgs::Int32MultiArray seg;
//            seg.data = *it;
//            // Verify that each index is valid in the point cloud
//            bool valid = true;
//            for (vector<int>::size_type i = 0; i < seg.data.size(); ++i)
//            {
//                if (seg.data[i] > cloud_size)
//                {
//                    ROS_WARN("ActiveExploration::segment : segment %lu specifies point %u which is larger than the point cloud %u",
//                             i, seg.data[i], cloud_size);
//                    valid = false;
//                    break;
//                }
//            }
//            if (valid)
//                _segments.push_back(*it);
//        }
//        // If successful segmentation
//        if (_segments.size() == 0)
//        {
//            ROS_ERROR("ActiveExploration::segment : cloud has zero segments");
//            return false;
//        }
//    }
//    // Otherwise cannot segment an empty point cloud
//    else
//    {
//        ROS_ERROR("ActiveExploration::segment : cloud is empty");
//        return false;
//    }

    if (!active_exploration_utils::segment(_cloud, segment_indices, _segments))
    {
        ROS_ERROR("ActiveExploration::segment : could not segment the point cloud");
        return false;
    }

    ROS_INFO("ActiveExploration::segment : successfully segmented point cloud into %lu segments", _segments.size());
    ROS_INFO("ActiveExploration::segment : finished");
    return true;
}

bool ActiveExploration::filter_segments()
{
    ROS_INFO("ActiveExploration::filter_segments : starting");
//    // Check that a point cloud exists
//    if (_cloud.size() == 0)
//    {
//        ROS_ERROR("ActiveExploration::filter_segments : input cloud is empty");
//        return false;
//    }
//    // Check that segments exist
//    if (_segments.size() == 0)
//    {
//        ROS_ERROR("ActiveExploration::filter_segments : _segments is empty");
//        return false;
//    }
//    // Get the planes
//    vector<int> plane_indices;
//    if (!find_planes(_cloud, plane_indices))
//    {
//        ROS_ERROR("ActiveExploration::filter_segments : could not find planes");
//        return false;
//    }
//    if (plane_indices.size() == 0)
//        ROS_WARN("ActiveExploration::filter_segments : no planes were detected");
//    else
//        ROS_INFO("ActiveExploration::filter_segments : found %lu planes", plane_indices.size());

//    // Keep track of the current ground
//    _current_ground_indices = plane_indices;
//    for (vector<int>::size_type i = 0; i < _current_ground_indices.size(); ++i)
//    {
//        // Get the key
//        OcTreeKey k;
//        point3d p (_transformed_cloud.points[_current_ground_indices[i]].x,
//                   _transformed_cloud.points[_current_ground_indices[i]].y,
//                   _transformed_cloud.points[_current_ground_indices[i]].z);
//        if (_tree.coordToKeyChecked(p,k))
//        {
//            OcTreeNode *node = _tree.search(k);
//            if (node)
//            {
//                if (node->getOccupancy() > 0)
//                    _current_ground_keys.push_back(k);
//            }
//        }
//    }
//    sort(_current_ground_keys.begin(), _current_ground_keys.end(), compare_octree_key);
//    _current_ground_keys.erase(unique(_current_ground_keys.begin(), _current_ground_keys.end(), equal_octree_key), _current_ground_keys.end());

//    PointT grd_min_pt, grd_max_pt;
//    PointCloud<PointT> ground_cloud;
//    copyPointCloud(_cloud, _current_ground_indices, ground_cloud);
//    getMinMax3D (ground_cloud, grd_min_pt, grd_max_pt);
//    // Verify each segment
//    vector<int> plane_segs;
//    vector<int> invalid_segs;
//    bool done_transform = true;
//    for (vector<vector<int> >::size_type i = 0; i != _segments.size(); ++i)
//    {
//        bool valid = true;
//        PointCloud<PointT> temp;
//        copyPointCloud(_cloud, _segments[i], temp);
//        PointT min_pt, max_pt;
//        getMinMax3D (temp, min_pt, max_pt);
//        double z_height = fabs(max_pt.data[2] - min_pt.data[2]);
//        double height_above_ground = min_pt.data[0] - grd_min_pt.data[0];
//        vector<int> v (_segments[i].size() + plane_indices.size());
//        vector<int>::iterator iter = set_intersection (_segments[i].begin(), _segments[i].end(),
//                                                       plane_indices.begin(), plane_indices.end(), v.begin());
//        v.resize(iter-v.begin());
//        double percentage_overlap = (double)v.size() / (double)_segments[i].size();
//        // If more than 50% of the points overlap with a plane then this segment is a plane
//        if (percentage_overlap >= 0.5)
//        {
//            //ROS_INFO("ActiveExploration::filter_segments : segment %lu has a %.2f overlap with a plane", i, percentage_overlap);
//            valid = false;
//            plane_segs.push_back(i);
////            // Double check the z height of this segment
////            if (_received_transform)
////            {
////                if (z_height < 0.25)
////                {
////                    ROS_INFO("ActiveExploration::filter_segments : rejecting segment %lu which has height is %.2f", i, z_height);
////                    valid = false;
////                    plane_segs.push_back(i);
////                }
////                else
////                {
////                    ROS_INFO("ActiveExploration::filter_segments : segment %lu has height %.2f, not small enough for ground", i, z_height);
////                }
////            }
////            else
////            {
////                valid = false;
////                plane_segs.push_back(i);
////            }
//        }
//        // Reject the segment if it is far away
//        if (valid)
//        {
//            Eigen::Vector4f centroid;
//            compute3DCentroid (_cloud, _segments[i], centroid);
//            float d = distance3D(0,centroid[0],0,centroid[1],0,centroid[2]);
//            if (d > _max_object_distance)
//            {
//                ROS_INFO("ActiveExploration::filter_segments : rejecting segment %lu which has distance is %.2f", i, d);
//                valid = false;
//                invalid_segs.push_back(i);
//            }
//        }
//        // Reject objects that are on the floor (only want objects on table)
//        if (valid)
//        {
//            if (_table_height_threshold > 0)  // negative implies don't use table height threshold
//            {
//                if (height_above_ground < _table_height_threshold) // should also check the minimum z point, but that may be too strong
//                {
//                    ROS_INFO("ActiveExploration::filter_segments : rejecting segment %lu which is above ground by %.2f",
//                             i, height_above_ground);
//                    invalid_segs.push_back(i);
//                    valid = false;
//                }
//            }
//        }
//        // If this has not yet been labelled as ground, check that it is valid
//        if (valid)
//        {
//            if (!is_valid_segment(temp, _transform, i))
//            {
//                valid = false;
//                invalid_segs.push_back(i);
//            }
//        }
//        // If the segment is still valid then add it to the segments list
//        if (valid)
//        {
//            //ROS_INFO("ActiveExploration::filter_segments : segment %lu is valid", i);
//        }
//    }

////    // Visualize the planes and invalid segments
////    if (!visualize_background(plane_segs, invalid_segs, grd_min_pt.data[2]))
////        ROS_ERROR("ActiveExploration::filter_segments : could not visualize the background");

//    // Remove the segments
//    invalid_segs.insert (invalid_segs.end(), plane_segs.begin(), plane_segs.end());
//    sort(invalid_segs.begin(), invalid_segs.end());  // sort
//    invalid_segs.erase(unique(invalid_segs.begin(), invalid_segs.end()), invalid_segs.end()); // remove duplicates
//    vector<vector<int> > previous_segments = _segments;
//    _segments.clear();
//    for (size_t i = 0; i < previous_segments.size(); ++i)
//    {
//        if (find(invalid_segs.begin(),invalid_segs.end(),i) == invalid_segs.end())
//            _segments.push_back (previous_segments[i]);
//    }

    if (!active_exploration_utils::filter_segments(_tree, _transformed_cloud, _position, _max_object_distance, _min_object_height, _max_object_height,
                                                  _min_object_length, _max_object_length, _segments, _current_ground_indices, _current_ground_keys))
    {
        ROS_ERROR("ActiveExploration::filter_segments : could not filter the segments");
        return false;
    }

    ROS_INFO("ActiveExploration::filter_segments : finished");

    return true;
}

bool ActiveExploration::extract_segment_octree_keys()
{
    ROS_INFO("ActiveExploration::extract_segment_octree_keys : starting");
//    // If there are no segments
//    if (_segments.size() == 0)
//    {
//        ROS_ERROR("ActiveExploration::extract_segment_octree_keys : segments is empty");
//        return false;
//    }
//    // Otherwise find the voxels
//    _segment_octree_keys.clear();
//    _segment_octree_keys.resize(_segments.size());
//    for (vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
//    {
//        vector<OcTreeKey> seg_keys;
//        for (vector<int>::size_type j = 0; j < _segments[i].size(); ++j)
//        {
//            // Get the key
//            OcTreeKey k;
//            point3d p (_transformed_cloud.points[_segments[i][j]].x,
//                       _transformed_cloud.points[_segments[i][j]].y,
//                       _transformed_cloud.points[_segments[i][j]].z);
//            if (_tree.coordToKeyChecked(p, k))
//                seg_keys.push_back(k);
//            else
//                ROS_WARN("ActiveExploration::extract_segment_octree_keys : could not find point (%.2f,%.2f,%.2f)", p.x(), p.y(), p.z());
//        }
//        sort(seg_keys.begin(), seg_keys.end(), compare_octree_key);
//        seg_keys.erase(unique(seg_keys.begin(), seg_keys.end(), equal_octree_key), seg_keys.end());
//        _segment_octree_keys[i] = seg_keys;
//    }
////    // Visualize the segments
////    octree_visualize_segments(_tree, 0.2, _segment_octree_keys);

    if (!active_exploration_utils::extract_segment_octree_keys(_tree, _transformed_cloud, _segments, _segment_octree_keys))
    {
        ROS_ERROR("ActiveExploration::extract_segment_octree_keys : Cloud not extract segment octree keys");
        return false;
    }

    ROS_INFO("ActiveExploration::extract_segment_octree_keys : finished");

    return true;
}

/* === POSE ESTIMATION / OBJECT TRACKING === */

bool ActiveExploration::estimate_pose()
{
    ROS_INFO("ActiveExploration::estimate_pose : starting");

//    // If have already segmented the point cloud
//    if (_segments.size() > 0)
//    {
//        // Clear
//        _poses.clear();
//        // Convert each segment to a point cloud
//        for (vector<vector<int> >::const_iterator it = _segments.begin(); it != _segments.end(); ++it)
//        {
//            // Extract the point cloud from the indices
//            PointCloud<PointT> seg;
//            copyPointCloud(_transformed_cloud, *it, seg);
//            // Compute the centroid
//            Eigen::Vector4f centroid;
//            compute3DCentroid (seg, centroid);
//            // Compute the bounding box limits
//            Eigen::Vector4f bb_min, bb_max;
//            getMinMax3D (_transformed_cloud, *it, bb_min, bb_max);
//            // Add to _poses vector
//            _poses.push_back (Pose (centroid, bb_min, bb_max));
//        }
//    }
//    // Otherwise, no segments exists
//    else
//    {
//        ROS_WARN("ActiveExploration::estimate_pose : have not segmented the point cloud");
//        if (!segment())
//        {
//            ROS_ERROR("ActiveExploration::estimate_pose : could not segment the point cloud");
//            return false;
//        }
//        // Call the estimate pose function
//        return estimate_pose();
//    }

    if (!active_exploration_utils::estimate_pose(_transformed_cloud, _segments, _poses))
    {
        ROS_ERROR("ActiveExploration::estimate_pose : Could not estimate the poses");
        return false;
    }

    ROS_INFO("ActiveExploration::estimate_pose : finished");
    return true;
}

/* === CLASSIFICATION === */

bool ActiveExploration::classify()
{
    ROS_INFO("ActiveExploration::classify : starting");
//    // If have already segmented the point cloud
//    if (_segments.size() > 0)
//    {
//        // Clear
//        _class_estimates.clear();
//        // Pass the data to the message
//        sensor_msgs::PointCloud2 cloud_msg;
//        pcl::toROSMsg(_cloud, cloud_msg);

//        vector<std_msgs::Int32MultiArray> seg_msg;
//        seg_msg.resize(_segments.size());
//        for(vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
//        {
//            std_msgs::Int32MultiArray seg;
//            seg.data = _segments[i];
//            seg_msg[i] = seg;
//        }
//        _class_srv.request.cloud = cloud_msg;
//        _class_srv.request.clusters_indices = seg_msg;
//        // Call the service
//        if (!_class_client.call(_class_srv))
//        {
//            ROS_ERROR("ActiveExploration::classify : could not call the classification service");
//            return false;
//        }
//        ROS_INFO("ActiveExploration::classify : successfully classified the segments");
//        _class_estimates = _class_srv.response.class_results;
//        // Replace the double back slashes in the file paths
//        fix_path_names();
//        // Read the best instance and extract the pose transform file for each of the object class result
//        if (!extract_instance_directories())
//        {
//            ROS_ERROR("ActiveExploration::classify : could not extract the instance directories");
//            return false;
//        }
//        ROS_INFO("ActiveExploration::classify : successfully extracted instance directories");
//        // Get the transformations of the instances to the maps
//        if (!transform_instances_to_map())
//        {
//            ROS_ERROR("ActiveExploration::classify : could not compute the transforms to the map");
//            return false;
//        }
//    }
//    // Otherwise, no segments exists
//    else
//    {
//        ROS_WARN("ActiveExploration::classify : have not segmented the point cloud");
//        if (!segment())
//        {
//            ROS_ERROR("ActiveExploration::classify : could not segment the point cloud");
//            return false;
//        }
//        // Call the classification function
//        return classify();
//    }

//    ROS_INFO("ActiveExploration::classify : class estimates");
//    for (size_t i = 0; i < _class_estimates.size(); ++i)
//    {
//        ROS_INFO("Segment %lu -", i);
//        for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
//            ROS_INFO("  %-15s %.2f", _class_estimates[i].class_type[j].data.c_str(), _class_estimates[i].confidence[j]);
//    }

    if (!active_exploration_utils::classify(_transformed_cloud, _segments, _class_client, _class_estimates, _instance_directories, _instances_to_map_tfs))
    {
        ROS_ERROR("ActiveExploration::classify : Could not classify segments");
        return false;
    }

    ROS_INFO("ActiveExploration::classify : finished");
    return true;
}

/* === GET ENTROPY MAPS === */

bool ActiveExploration::retrieve_entropy_maps(vector<vector<EntMap> > &entropy_maps)
{
    ROS_INFO("ActiveExploration::retrieve_entropy_maps : starting");
//    entropy_maps.clear();
//    // Check if there are segments and class estimates available with to do the planning
//    if (_segments.size() == 0)
//    {
//        ROS_ERROR("ActiveExploration::retrieve_entropy_maps : no segments available");
//        return false;
//    }
//    if (_instance_directories.size() == 0)
//    {
//        ROS_ERROR("ActiveExploration::retrieve_entropy_maps : no instance directories available");
//        return false;
//    }
//    if (_segments.size() != _instance_directories.size())
//    {
//        ROS_ERROR("ActiveExploration::retrieve_entropy_maps : segments size %lu does not match instance directories size %lu",
//                  _segments.size(), _instance_directories.size());
//        return false;
//    }
//    // For each segment call the entropy map service
//    entropy_maps.resize(_instance_directories.size());
//    for (size_t i = 0; i < _instance_directories.size(); ++i)
//    {
//        // For each class estimate
//        for (size_t j = 0; j < _instance_directories[i].size(); ++j)
//        {
//            // Get the instance entropy map
//            // Call the service to get the instance entropy map
//            _em_srv.request.class_type.data = _instance_directories[i][j]._class_type;
//            _em_srv.request.instance_name.data = _instance_directories[i][j]._instance_name;
//            if (!_em_client.call(_em_srv))
//            {
//                ROS_ERROR("ActiveExploration::retrieve_entropy_maps : could not call the entropy map service");
//                return false;
//            }
////            ROS_INFO("ActiveExploration::retrieve_entropy_maps : extracted entropy map with %lu elements and instance cloud size %lu",
////                      _em_srv.response.entropy_map.size(), _em_srv.response.cloud.data.size());
//            // Convert to local structure
//            EntMap emap;
//            if (ActiveExploration::fromROSMsg(_em_srv, emap))
//            {
//                emap._valid = true;
//                entropy_maps[i].push_back (emap);
//            }
//            else
//            {
//                ROS_ERROR("ActiveExploration::retrieve_entropy_maps : could not convert the entropy map for %s %s",
//                          _em_srv.request.class_type.data.c_str(), _em_srv.request.instance_name.data.c_str());
//                emap._valid = false;
//                entropy_maps[i].push_back (emap);
//            }
//        }
//    }
//    ROS_INFO("ActiveExploration::retrieve_entropy_maps : retrieved %lu entropy maps", entropy_maps.size());
//    if (entropy_maps.size() != _segments.size())
//        ROS_WARN("ActiveExploration::retrieve_entropy_maps : retrieved %lu entropy maps, should be %lu",
//                 entropy_maps.size(), _segments.size());

    if (!active_exploration_utils::retrieve_entropy_maps(_segments, _instance_directories, _em_client, entropy_maps))
    {
        ROS_ERROR("ActiveExploration::retrieve_entropy_maps : Could not retrive entropy maps");
        return false;
    }

    ROS_INFO("ActiveExploration::retrieve_entropy_maps : finished");
    return true;
}

bool ActiveExploration::retrieve_entropy_maps()
{
    return retrieve_entropy_maps(_emaps);
}

/* === COMPUTE ENTROPY === */

bool ActiveExploration::compute_entropy()
{
    ROS_INFO("ActiveExploration::compute_entropy : starting");
//    // Check that classification results exist
//    if (_class_estimates.size() > 0)
//    {
//        // Clear
//        _entropies.clear();
//        // Compute the entropies
//        for (vector<Classification>::const_iterator it = _class_estimates.begin(); it != _class_estimates.end(); ++it)
//            _entropies.push_back ((double)entropy(it->confidence));
//    }
//    // Otherwise, must do classification
//    else
//    {
//        ROS_WARN("ActiveExploration::compute_entropy : have not classified the point cloud");
//        if (!classify())
//        {
//            ROS_ERROR("ActiveExploration::compute_entropy : could not classify the point cloud");
//            return false;
//        }
//        // Call the compute entropy function
//        return compute_entropy();
//    }

    if (!active_exploration_utils::compute_entropy(_class_estimates, _entropies))
    {
        ROS_ERROR("ActiveExploration::compute_entropy : Could not compute the entropies");
        return false;
    }

    ROS_INFO("ActiveExploration::compute_entropy : finished");
    return true;
}

bool ActiveExploration::rank_entropy()
{
    ROS_INFO("ActiveExploration::rank_entropy : starting");
//    // Check that entropies have been calculated
//    if (_entropies.size() > 0)
//    {
//        // Clear
//        _entropy_ranking.clear();
//        // Compute the ranking
//        _entropy_ranking = rank(_entropies);
//        reverse (_entropy_ranking.begin(), _entropy_ranking.end());
//    }
//    // Otherwise, must compute the entropies
//    else
//    {
//        ROS_WARN("ActiveExploration::rank_entropy : have not computed the entropies of the segments");
//        if (!classify())
//        {
//            ROS_ERROR("ActiveExploration::rank_entropy : could not compute the entropies");
//            return false;
//        }
//        // Call the rank entropy function
//        return rank_entropy();
//    }

    if (!active_exploration_utils::rank_entropy(_entropies, _entropy_ranking))
    {
        ROS_ERROR("ActiveExploration::rank_entropy : Could not rank the entropies");
        return false;
    }

    ROS_INFO("ActiveExploration::rank_entropy : finished");
    return true;
}

/* === UPDATE HYPOTHESES === */

bool ActiveExploration::update_hypotheses()
{
    // Copy the current values
    PointCloud<PointT> pre_cloud = _transformed_cloud;
    vector<vector<int> > pre_segs = _segments;
    vector<vector<OcTreeKey> > pre_keys = _segment_octree_keys;
    vector<Classification> pre_ests = _class_estimates;
    vector<Pose> pre_poses = _poses;
    vector<vector<InstLookUp> > pre_inst_dirs = _instance_directories;
    vector<vector<InstToMapTF> > pre_trans = _instances_to_map_tfs;
    // Get input from the sensor
    clear_sensor_input();
    clear_hypotheses();
    if (!data_from_sensor())
    {
        ROS_ERROR("ActiveExploration::update_hypotheses : could not get data from the sensor");
        return false;
    }
    // Initialize visualizer
    if (!initialize_visualization())
    {
        ROS_ERROR("ActiveExploration::update_hypotheses : could not initialize visualizer");
        return false;
    }

    // Update
    vector<vector<int> > segment_indices;
    vector<vector<int> > overlaps;
    if (!update_hypotheses(pre_keys, pre_ests, pre_poses, pre_inst_dirs, pre_trans, segment_indices, overlaps))
    {
        ROS_ERROR("ActiveExploration::update_hypotheses : error updating hypotheses");
        return false;
    }

    if (_visualization_on)
        visualize_segment_overlap(pre_cloud, pre_segs, pre_poses, pre_keys, overlaps);

    return true;
}

bool ActiveExploration::update_hypotheses(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const Eigen::Matrix4f transform,
                                          const vector<vector<int> > &segment_indices)
{
    // Copy the current values
    PointCloud<PointT> pre_cloud = _transformed_cloud;
    vector<vector<int> > pre_segs = _segments;
    vector<vector<OcTreeKey> > pre_keys = _segment_octree_keys;
    vector<Classification> pre_ests = _class_estimates;
    vector<Pose> pre_poses = _poses;
    vector<vector<InstLookUp> > pre_inst_dirs = _instance_directories;
    vector<vector<InstToMapTF> > pre_trans = _instances_to_map_tfs;

    // Set the cloud and image
    clear_hypotheses();
    set_data (cloud, image, transform);
    // Initialize visualizer
    if (!initialize_visualization())
    {
        ROS_ERROR("ActiveExploration::update_hypotheses : could not initialize visualizer");
        return false;
    }
    // Update
    vector<vector<int> > overlaps;
    if (!update_hypotheses (pre_keys, pre_ests, pre_poses, pre_inst_dirs, pre_trans, segment_indices, overlaps))
    {
        ROS_ERROR("ActiveExploration::update_hypotheses : error updating hypotheses");
        return false;
    }

    if (_visualization_on)
        visualize_segment_overlap(pre_cloud, pre_segs, pre_poses, pre_keys, overlaps);

    return true;
}

bool ActiveExploration::update_hypotheses(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const vector<vector<int> > &segment_indices)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    return update_hypotheses (cloud, image, transform, segment_indices);
}

bool ActiveExploration::update_hypotheses(const ros_CloudTX &ctx, const sensor_msgs::Image &image, const vector<vector<int> > &segment_indices)
{
    return update_hypotheses (ctx._cloud, image, ctx._transform, segment_indices);
}

/* === PLANNING === */

bool ActiveExploration::plan(int &next_best_index, const SIM_TYPE &sim, const double &variance, const vector<Eigen::Vector4f> &map_locations)
{
    ROS_INFO("ActiveExploration::plan : starting");
    // Check if there are segments and class estimates available with to do the planning
    if (_segments.size() == 0)
    {
        ROS_ERROR("ActiveExploration::plan : no segments available");
        return false;
    }
    if (_instance_directories.size() == 0)
    {
        ROS_ERROR("ActiveExploration::plan : no instances available");
        return false;
    }
    if (_segments.size() != _instance_directories.size())
    {
        ROS_ERROR("ActiveExploration::plan : segments size %lu does not match instances size %lu",
                  _segments.size(), _instance_directories.size());
        return false;
    }

    // Compute the utility value for each map location
    if (map_locations.size() == 0)
    {
        ROS_ERROR("ActiveExploration::plan : no locations to do planning");
        return false;
    }

//    // Extract the transforms out of the itfs
//    if (!next_best_view(next_best_index, sim, map_locations, variance))
//    {
//        ROS_ERROR("ActiveExploration::plan : could not determine the next best view");
//        return false;
//    }

    Hypothesis hyp;
    hyp._segments = _segments;
    hyp._octree_keys = _segment_octree_keys;
    hyp._class_estimates = _class_estimates;
    hyp._poses = _poses;
    hyp._instance_directories = _instance_directories;
    hyp._transforms = _instances_to_map_tfs;
    hyp._emaps = _emaps;
    hyp._entropies = _entropies;
    hyp._entropy_ranking = _entropy_ranking;

    if (!active_exploration_utils::next_best_view(next_best_index, _tree, hyp, sim, map_locations, variance, _visualization_on))
    {
        ROS_ERROR("ActiveExploration::plan : Could not find next best view");
        return false;
    }

    // Visualize the instances in the map frame
    if (_visualization_on)
        visualize_planning_world (map_locations[next_best_index], map_locations, sim);

    ROS_INFO("ActiveExploration::plan : finihsed");
    return true;
}

/* === VISUALIZATION === */

bool ActiveExploration::initialize_visualization()
{
    ROS_INFO("ActiveExploration::initialize_visualization : starting");
    if (_cloud.size() > 0 && _image.data.size() > 0)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(_cloud, msg);
        _viz_init_srv.request.cloud = msg;
        _viz_init_srv.request.saliency_map = _image;
        if (!_viz_init_client.call(_viz_init_srv))
        {
            ROS_ERROR("ActiveExploration::initialize_visualization : could not initialize visualizer");
            return false;
        }
    }
    else
    {
        ROS_WARN("ActiveExploration::initialize_visualization : skipping visualization initialization, cloud has %lu points and image has %lu pixels",
                 _cloud.size(), _image.data.size());
    }

    ROS_INFO("ActiveExploration::initialize_visualization : finished");
    return true;
}

bool ActiveExploration::visualize()
{
    ROS_INFO("ActiveExploration::visualize : starting");
    PointT grd_min_pt, grd_max_pt;
    PointCloud<PointT> ground_cloud;
    copyPointCloud(_cloud, _current_ground_indices, ground_cloud);
    getMinMax3D (ground_cloud, grd_min_pt, grd_max_pt);
    for (vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
    {
        // Wait for input
        bool false_points = false;
        cout << "Display segment ";
        cin.ignore();
        std_msgs::Int32MultiArray seg;
        if (_segments[i].size() > 0)
        {
            seg.data = _segments[i];
        }
        else
        {
            false_points = true;
            // Get the points that are located in the octomap
            vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(_tree, _segment_octree_keys[i], _map_key_to_points,
                                                                                         _current_ground_keys);
            seg.data = unknown_seg_ix;
        }
        vector<std_msgs::Int32MultiArray> cluster_indices;
        cluster_indices.push_back (seg);
        _viz_srv.request.clusters_indices = cluster_indices;
        if (_viz_client.call(_viz_srv))
        {
            // Print the size of the segment
            ROS_INFO("Segment %lu : %lu", i, _segments[i].size());
            if (false_points)
                ROS_WARN("points are false");
            // Print the pose
            if (_poses.size() > 0 && i < _poses.size())
            {
                ROS_INFO("Pose :");
                Eigen::Vector4f centroid = _poses[i].get_centroid();
                ROS_INFO("  centroid %.2f %.2f %.2f", centroid[0], centroid[1], centroid[2]);
                Eigen::Vector4f bb_min = _poses[i].get_bb_min();
                ROS_INFO("  min      %.2f %.2f %.2f", bb_min[0], bb_min[1], bb_min[2]);
                Eigen::Vector4f bb_max = _poses[i].get_bb_max();
                ROS_INFO("  max      %.2f %.2f %.2f", bb_max[0], bb_max[1], bb_max[2]);
            }
            // Print the number of octree voxels for this segment
            if (_segment_octree_keys.size() > 0 && i < _segment_octree_keys.size())
            {
                ROS_INFO("Octree keys :");
                ROS_INFO("   num %lu", _segment_octree_keys[i].size());
            }
            // Print the classification result
            if (_class_estimates.size() > 0 && i < _class_estimates.size())
            {
                ROS_INFO("Classification :");
                for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                {
                    ROS_INFO("  %-15s %.2f", _class_estimates[i].class_type[j].data.c_str(),
                                             _class_estimates[i].confidence[j]);
                    ROS_INFO("     pose file %s", _class_estimates[i].pose[j].data.c_str());
                }
            }
            // Print the entropy
            if (_entropies.size() > 0 &&  i < _entropies.size())
            {
                ROS_INFO("Entropy :");
                ROS_INFO("  entropy = %.2f", _entropies[i]);
            }
            // Print the size
            if (!false_points)
            {
                PointCloud<PointT> seg_cloud;
                copyPointCloud(_cloud, _segments[i], seg_cloud);
                PointT min_pt, max_pt;
                getMinMax3D (seg_cloud, min_pt, max_pt);
                double x_length = fabs(max_pt.data[0] - min_pt.data[0]);
                double y_length = fabs(max_pt.data[1] - min_pt.data[1]);
                double z_height = fabs(max_pt.data[2] - min_pt.data[2]);
                ROS_INFO("Dimensions :");
                ROS_INFO("  x length %.2f", x_length);
                ROS_INFO("  y length %.2f", y_length);
                ROS_INFO("  z length %.2f", z_height);
                ROS_INFO("  ground = %.2f %.2f %.2f", grd_min_pt.data[0], grd_min_pt.data[1], grd_min_pt.data[2]);
                ROS_INFO("  dims   = %.2f %.2f %.2f", min_pt.data[0], min_pt.data[1], min_pt.data[2]);
                double xh = min_pt.data[0] - grd_min_pt.data[0];
                double yh = min_pt.data[1] - grd_min_pt.data[1];
                double zh = min_pt.data[2] - grd_min_pt.data[2];
                ROS_INFO("  height = %.2f %.2f %.2f", xh, yh, zh);
            }
        }
        else
        {
            ROS_INFO("ActiveExploration::visualize : could not visualize segment %lu", i);
            return false;
        }
    }
    cout << "Continue with processing " << endl;
    cin.ignore();
    // Print the entropy rankings of the segments
    ROS_INFO("The 10 highest entropies:");
    if (_entropy_ranking.size() > 0)
    {
        for (int i = 0; i < 10; ++i)
        {
            // Check this is a valid index
            if (i <= (_entropy_ranking.size()-1) &&
                i <= (_entropies.size()-1) &&
                i <= (_class_estimates.size()-1) &&
                i <= (_segments.size()-1))
            {
                int ix = _entropy_ranking[i];
                printf("  segment %d - points %lu, class %s, confidence %.2f, entropy %.2f\n",
                         ix, _segments[ix].size(), _class_estimates[ix].class_type[0].data.c_str(),
                         _class_estimates[ix].confidence[0], _entropies[ix]);
            }
        }
    }

    ROS_INFO("ActiveExploration::visualize : finished");
    return true;
}

bool ActiveExploration::visualize_background(const vector<int> &planes, const vector<int> &invalids, const float &ground_height)
{
    ROS_INFO("ActiveExploration::visualize_background : starting");
    vector<int> non_segments;
    // Visualize planes
    printf(ANSI_COLOR_GREEN  "PLANES \n"  ANSI_COLOR_RESET);
    cin.ignore();
    for (vector<int>::size_type i = 0; i < planes.size(); ++i)
    {
        non_segments.push_back (planes[i]);
        std_msgs::Int32MultiArray seg;
        seg.data = _segments[planes[i]];
        vector<std_msgs::Int32MultiArray> cluster_indices;
        cluster_indices.push_back (seg);
        _viz_srv.request.clusters_indices = cluster_indices;
        if (_viz_client.call(_viz_srv))
        {
            // Print the size of the segment
            cout << "(segment " << planes[i] << ")" << endl;
            ROS_INFO("plane %lu : %lu", i, _segments[planes[i]].size());
            Eigen::Vector4f centroid;
            compute3DCentroid (_cloud, _segments[planes[i]], centroid);
            cout << "distance from sensor " << distance3D(0,centroid[0],0,centroid[1],0,centroid[2]) << endl;
            PointCloud<PointT> temp;
            copyPointCloud(_cloud, _segments[planes[i]], temp);
            PointT min_pt, max_pt;
            getMinMax3D (temp, min_pt, max_pt);
            if (ground_height != numeric_limits<float>::infinity())
                //cout << "height from ground " << max_pt.data[2] - ground_height << endl;
            copyPointCloud(_transformed_cloud, _segments[planes[i]], temp);
            getMinMax3D (temp, min_pt, max_pt);
            cout << "dimensions " << fabs(max_pt.data[0] - min_pt.data[0]) << " "
                                  << fabs(max_pt.data[1] - min_pt.data[1]) << " "
                                  << fabs(max_pt.data[2] - min_pt.data[2]) << endl;
            // Wait for input
            cin.ignore();
        }
        else
        {
            ROS_INFO("ActiveExploration::visualize_background : could not visualize plane %lu", i);
            return false;
        }
    }
    // Visualize invalids
    printf(ANSI_COLOR_YELLOW  "INVALIDS \n"  ANSI_COLOR_RESET);
    cin.ignore();
    for (vector<int>::size_type i = 0; i < invalids.size(); ++i)
    {
        non_segments.push_back (invalids[i]);
        std_msgs::Int32MultiArray seg;
        seg.data = _segments[invalids[i]];
        vector<std_msgs::Int32MultiArray> cluster_indices;
        cluster_indices.push_back (seg);
        _viz_srv.request.clusters_indices = cluster_indices;
        if (_viz_client.call(_viz_srv))
        {
            // Print the size of the segment
            cout << "(segment " << invalids[i] << ")" << endl;
            ROS_INFO("invalid %lu : %lu", i, _segments[invalids[i]].size());
            Eigen::Vector4f centroid;
            compute3DCentroid (_cloud, _segments[invalids[i]], centroid);
            cout << "distance from sensor " << distance3D(0,centroid[0],0,centroid[1],0,centroid[2]) << endl;
            PointCloud<PointT> temp;
            copyPointCloud(_cloud, _segments[invalids[i]], temp);
            PointT min_pt, max_pt;
            getMinMax3D (temp, min_pt, max_pt);
            if (ground_height != numeric_limits<float>::infinity())
                cout << "height from ground " << max_pt.data[2] - ground_height << endl;
            copyPointCloud(_cloud, _segments[invalids[i]], temp);
            getMinMax3D (temp, min_pt, max_pt);
            cout << "dimensions " << fabs(max_pt.data[0] - min_pt.data[0]) << " "
                                  << fabs(max_pt.data[1] - min_pt.data[1]) << " "
                                  << fabs(max_pt.data[2] - min_pt.data[2]) << endl;
            // Wait for input
            cin.ignore();
        }
        else
        {
            ROS_INFO("ActiveExploration::visualize_background : could not visualize invalid %lu", i);
            return false;
        }
    }
    // Visualize normal segments
    sort(non_segments.begin(), non_segments.end());
    non_segments.erase(unique(non_segments.begin(), non_segments.end()), non_segments.end());
    printf(ANSI_COLOR_CYAN  "SEGMENTS"  ANSI_COLOR_RESET);
    cin.ignore();
    for (vector<std_msgs::Int32MultiArray>::size_type i = 0; i < _segments.size(); ++i)
    {
        if (find(non_segments.begin(), non_segments.end(), i) == non_segments.end())
        {
            std_msgs::Int32MultiArray seg;
            seg.data = _segments[i];
            vector<std_msgs::Int32MultiArray> cluster_indices;
            cluster_indices.push_back (seg);
            _viz_srv.request.clusters_indices = cluster_indices;
            if (_viz_client.call(_viz_srv))
            {
                // Print the size of the segment
                ROS_INFO("segment %lu : %lu", i, _segments[i].size());
                Eigen::Vector4f centroid;
                compute3DCentroid (_cloud, _segments[i], centroid);
                cout << "distance from sensor " << distance3D(0,centroid[0],0,centroid[1],0,centroid[2]) << endl;
                PointCloud<PointT> temp;
                PointT min_pt, max_pt;
//                copyPointCloud(_cloud, _segments[i], temp);
//                getMinMax3D (temp, min_pt, max_pt);
//                if (ground_height != numeric_limits<float>::infinity())
//                    cout << "height from ground " << max_pt.data[2] - ground_height << endl;
                copyPointCloud(_transformed_cloud, _segments[i], temp);
                getMinMax3D (temp, min_pt, max_pt);
                cout << "dimensions " << fabs(max_pt.data[0] - min_pt.data[0]) << " "
                                      << fabs(max_pt.data[1] - min_pt.data[1]) << " "
                                      << fabs(max_pt.data[2] - min_pt.data[2]) << endl;
                // Wait for input
                cin.ignore();
            }
            else
            {
                ROS_INFO("ActiveExploration::visualize_background : could not visualize segment %lu", i);
                return false;
            }
        }
        else
        {
            ROS_INFO("ActiveExploration::visualize_background : segment %lu is not a valid segment", i);
        }
    }

    // Re-initialize the visualization
    if (!initialize_visualization())
    {
        ROS_ERROR("ActiveExploration::visualize_background : could not initialize visualizer");
        return false;
    }

    ROS_INFO("ActiveExploration::visualize_background : finished");
    return true;
}

bool ActiveExploration::visualize_planning_world(const Eigen::Vector4f &best_location, const vector<Eigen::Vector4f> &map_locations,
                                                 const SIM_TYPE &sim)
{
    ROS_INFO("ActiveExploration::visualize_planning_world : starting");

    // Start the visualization
    ROS_INFO("ActiveExploration::visualize_planning_world : viewing the planning world");
    visualization::PCLVisualizer *viewer  = new visualization::PCLVisualizer("Scene");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    // Get the camera position
    Eigen::Vector4f camera_pose = extract_camera_position (_cloud);
    Eigen::Vector4f cam_transformed = transform_eigvec(camera_pose, _transform);
    PointCloud<PointT> camera_pose_pt;
    camera_pose_pt.resize(1);
    camera_pose_pt.points[0].x = cam_transformed[0];
    camera_pose_pt.points[0].y = cam_transformed[1];
    camera_pose_pt.points[0].z = cam_transformed[2];
    viewer->addSphere (camera_pose_pt[0], 0.06, 1.0, 1.0, 1.0, "camera position");  // White

    // Add the map locations to the viewer
    string f;
    PointCloud<PointT> location_cloud;
    location_cloud.resize(1);
    for (vector<Eigen::Vector4f>::size_type i = 0; i < map_locations.size(); ++i)
    {
        f = "map position " + boost::lexical_cast<string>(i);
        location_cloud.points[0].x = map_locations[i][0];
        location_cloud.points[0].y = map_locations[i][1];
        location_cloud.points[0].z = map_locations[i][2];
        viewer->addSphere (location_cloud.points[0], 0.04, 0.5, 0.5, 0.5, f);  // Grey
    }

    // Show the next best location
    PointCloud<PointT> best_location_pt;
    best_location_pt.resize(1);
    best_location_pt.points[0].x = best_location[0];
    best_location_pt.points[0].y = best_location[1];
    best_location_pt.points[0].z = best_location[2];
    viewer->addSphere (best_location_pt[0], 0.06, 1.0, 0.3, 0.6, "next best view");  // Pink

    // Unwrap the the transforms from the _instances_to_map_tfs vector
    vector<vector<Eigen::Matrix4f> > itfs;
    itfs.resize(_instances_to_map_tfs.size());
    for (vector<vector<InstToMapTF> >::size_type i = 0; i < _instances_to_map_tfs.size(); ++i)
    {
        itfs[i].resize(_instances_to_map_tfs[i].size());
        for (vector<InstToMapTF>::size_type j = 0; j < _instances_to_map_tfs[i].size(); ++j)
            itfs[i][j] = _instances_to_map_tfs[i][j]._transform;
    }
    // Threads to simultaneously 1) keep viewer open and 2) display clouds and wait for next input to update the clouds
    bool cloud_update_thread_success_val = true;
    bool *cloud_update_thread_success = &cloud_update_thread_success_val;
    boost::thread view_thread = boost::thread(&active_exploration_utils::run_viewer, viewer);
    // View each segment with its overlapping recognised instance and potenttial views
    boost::thread cloud_update_thread = boost::thread(&ActiveExploration::run_view_segment, this, viewer, _emaps, itfs, sim,
                                                      cloud_update_thread_success);
    // Join the threads
    view_thread.join();
    cloud_update_thread.join();
    // Finished, wait for thread to exit
    ROS_INFO("ActiveExploration::visualize_planning_world : finished viewing all point clouds");
    if (!*cloud_update_thread_success)
        ROS_WARN("ActiveExploration::visualize_planning_world : cloud update thread exited with FALSE");
    // Close the window
    if (viewer)
        delete viewer;
    if (!*cloud_update_thread_success)
        return false;

    ROS_INFO("ActiveExploration::visualize_planning_world : finished");
    return true;
}

bool ActiveExploration::visualize_in_model_frame(const Eigen::Vector4f &location, const EntMap &emap,
                                                 const Eigen::Matrix4f &instance_to_map_tf, const Eigen::Matrix4f &instance_to_model_tf)
{
    // Visualize this location in the entropy map
    visualization::PCLVisualizer viewer("Point clouds");
    int v1 = 0;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, v1); // Setting background to a dark grey
    int v2 = 1;
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, v2); // Setting background to a dark grey

    PointCloud<PointT>::Ptr transformed_cloud_ptr (new PointCloud<PointT>(_transformed_cloud));
    visualization::PointCloudColorHandlerCustom<PointT> cloud_handler (transformed_cloud_ptr, 255, 255, 255);  // White
    viewer.addPointCloud (transformed_cloud_ptr, cloud_handler, "transformed_cloud", v1);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud", v1);

    Eigen::Vector4f camera_pose = extract_camera_position (_cloud);
    PointCloud<PointT>::Ptr camera_pose_pt (new PointCloud<PointT>());
    camera_pose_pt->resize(1);
    camera_pose_pt->points[0].x = camera_pose[0];
    camera_pose_pt->points[0].y = camera_pose[1];
    camera_pose_pt->points[0].z = camera_pose[2];
    transformPointCloud(*camera_pose_pt, *camera_pose_pt, _transform);
    visualization::PointCloudColorHandlerCustom<PointT> camera_pt_handler (camera_pose_pt, 255, 0, 0);  // Red
    viewer.addPointCloud (camera_pose_pt, camera_pt_handler, "camera_pt", v1);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "camera_pt", v1);

    PointCloud<PointT>::Ptr location_cloud (new PointCloud<PointT>());
    location_cloud->resize(1);
    location_cloud->points[0].x = location[0];
    location_cloud->points[0].y = location[1];
    location_cloud->points[0].z = location[2];
    visualization::PointCloudColorHandlerCustom<PointT> location_pt_handler (location_cloud, 0, 255, 0);  // Green
    viewer.addPointCloud (location_cloud, location_pt_handler, "location_pt", v1);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "location_pt", v1);

    Eigen::Matrix4f i_transform = instance_to_map_tf * instance_to_model_tf.inverse();  // CHANGED
    PointCloud<PointT>::Ptr model_cloud (new PointCloud<PointT>());
    transformPointCloud(emap._instance_cloud, *model_cloud, i_transform);
    visualization::PointCloudColorHandlerCustom<PointT> model_handler (model_cloud, 255, 0, 0);  // Red
    viewer.addPointCloud (model_cloud, model_handler, "model_pt", v1);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model_pt", v1);

    PointCloud<PointT>::Ptr view_pts_cloud (new PointCloud<PointT>());
    view_pts_cloud->resize(emap._camera_poses.size());
    for (int c = 0; c < emap._camera_poses.size(); ++c)
    {
        PointCloud<PointT> v_pt;
        v_pt.resize(1);
        v_pt.points[0].x = emap._camera_poses[c][0];
        v_pt.points[0].y = emap._camera_poses[c][1];
        v_pt.points[0].z = emap._camera_poses[c][2];
        transformPointCloud(v_pt, v_pt, emap._transforms[c]);
        view_pts_cloud->points[c] = v_pt.points[0];
    }
    transformPointCloud(*view_pts_cloud, *view_pts_cloud, i_transform);
    visualization::PointCloudColorHandlerCustom<PointT> views_handler (view_pts_cloud, 0, 0, 255);  // Blue
    viewer.addPointCloud (view_pts_cloud, views_handler, "views_pt", v1);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "views_pt", v1);

    // Visualize the location in the model frame
    PointCloud<PointT>::Ptr model_cloud_orig (new PointCloud<PointT>(emap._instance_cloud));
    visualization::PointCloudColorHandlerCustom<PointT> model_handler_orig (model_cloud_orig, 255, 0, 0);  // Red
    viewer.addPointCloud (model_cloud_orig, model_handler_orig, "model_pt_orig", v2);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model_pt_orig", v2);

    PointCloud<PointT>::Ptr view_pts_cloud_orig (new PointCloud<PointT>());
    view_pts_cloud_orig->resize(emap._camera_poses.size());
    for (int c = 0; c < emap._camera_poses.size(); ++c)
    {
        PointCloud<PointT> v_pt;
        v_pt.resize(1);
        v_pt.points[0].x = emap._camera_poses[c][0];
        v_pt.points[0].y = emap._camera_poses[c][1];
        v_pt.points[0].z = emap._camera_poses[c][2];
        transformPointCloud(v_pt, v_pt, emap._transforms[c]);
        view_pts_cloud_orig->points[c] = v_pt.points[0];
    }
    visualization::PointCloudColorHandlerCustom<PointT> views_handler_orig (view_pts_cloud_orig, 0, 0, 255);  // Blue
    viewer.addPointCloud (view_pts_cloud_orig, views_handler_orig, "views_pt_orig", v2);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "views_pt_orig", v2);

    // Transform the location into the model frame

    //Eigen::Matrix4f i_transform = itfs[seg_ix][k] * _transforms_to_instances[seg_ix][k].inverse();
    // Transform model to map: x_map = (A*B^-1)x
    //                               = A(B^-1x)
    // Transform map to model:     A^-1x_map = B^-1x
    //                          B(A^-1x_map) = x
    //                         (B*A^-1)x_map = x
    // General rule (A*B)^-1 = B^-1*A^-1
    // So        (A*B^-1)^-1 = B*A^-1
    Eigen::Matrix4f m_transform = instance_to_model_tf * instance_to_map_tf.inverse();

    camera_pose_pt->clear();
    camera_pose_pt->resize(1);
    camera_pose_pt->points[0].x = camera_pose[0];
    camera_pose_pt->points[0].y = camera_pose[1];
    camera_pose_pt->points[0].z = camera_pose[2];
    transformPointCloud(*camera_pose_pt, *camera_pose_pt, _transform);
    transformPointCloud(*camera_pose_pt, *camera_pose_pt, m_transform);
    visualization::PointCloudColorHandlerCustom<PointT> camera_pt_handler_orig (camera_pose_pt, 255, 0, 0);  // Red
    viewer.addPointCloud (camera_pose_pt, camera_pt_handler_orig, "camera_pt_orig", v2);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "camera_pt_orig", v2);

    location_cloud->clear();
    location_cloud->resize(1);
    location_cloud->points[0].x = location[0];
    location_cloud->points[0].y = location[1];
    location_cloud->points[0].z = location[2];
    transformPointCloud(*location_cloud, *location_cloud, m_transform);
    visualization::PointCloudColorHandlerCustom<PointT> location_pt_handler_orig (location_cloud, 0, 255, 0);  // Green
    viewer.addPointCloud (location_cloud, location_pt_handler_orig, "location_pt_orig", v2);
    viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "location_pt_orig", v2);

    while (!viewer.wasStopped())
        viewer.spinOnce();

    return true;
}

bool ActiveExploration::visualize_segment_overlap(const PointCloud<PointT> &previous_cloud, const vector<vector<int> > &previous_segments,
                                                  const vector<Pose> &previous_poses, const vector<vector<OcTreeKey> > &previous_keys,
                                                  const vector<vector<int> > &overlaps)
{
    ROS_INFO("ActiveExploration::visualize_segment_overlap : starting");

    // Print out the associations
    ROS_INFO("Associations :");
    for (vector<vector<int> >::size_type i = 0; i < overlaps.size(); ++i)
    {
        string f = boost::lexical_cast<string>(i);
        if (overlaps[i].size() > 0)
        {
            f += " [";
            for (vector<int>::size_type j = 0; j < overlaps[i].size(); ++j)
                f += boost::lexical_cast<string>(overlaps[i][j]) + ",";
            f = f.substr(0, f.size()-1);  // Remove the last comma
            f += "]";
        }
        ROS_INFO("   %s", f.c_str());
    }

    // Start the visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer ("Overlaps");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // Show the transformed point clouds
    PointCloud<PointT>::Ptr cloud1 (new PointCloud<PointT>(_transformed_cloud));
    visualization::PointCloudColorHandlerCustom<PointT> cloud1_handler (cloud1, 230, 230, 20);  // Yellow
    viewer->addPointCloud (cloud1, cloud1_handler, "cloud1");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
    PointCloud<PointT>::Ptr cloud2 (new PointCloud<PointT>(previous_cloud));
    visualization::PointCloudColorHandlerCustom<PointT> cloud2_handler (cloud2, 20, 230, 20);  // Green
    viewer->addPointCloud (cloud2, cloud2_handler, "cloud2");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

    // Show each segment, its bounding box and the corresponding overlap with the segment in the other cloud
    PointCloud<PointT>::Ptr first_centroids (new PointCloud<PointT>());
    first_centroids->resize(_segments.size());
    string f;
    PointCloud<PointT>::Ptr seg (new PointCloud<PointT>());
    for (vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
    {
        // Show the segment
        if (_segments[i].size() > 0)
        {
            copyPointCloud(_transformed_cloud, _segments[i], *seg);
            visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 230, 20, 20);  // Red
            f = "segment" + boost::lexical_cast<string>(i);
            viewer->addPointCloud (seg, seg_handler, f);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
        }
        else
        {
            // Get the points that are located in the octomap
            vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(_tree, _segment_octree_keys[i], _map_key_to_points,
                                                                                         _current_ground_keys);
            // If there are points found
            if (unknown_seg_ix.size() > 0)
            {
                copyPointCloud(_transformed_cloud, unknown_seg_ix, *seg);
                visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 200, 100, 100);  // Pink
                f = "segment" + boost::lexical_cast<string>(i);
                viewer->addPointCloud (seg, seg_handler, f);
                viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
            }
        }
        // Show the bounding box
        // CUBE = (xmin, xmax, ymin, ymax, zmin, zmax, r, g, b, id, viewport) // rgb = 0-1
        Eigen::Vector4f bb_min = _poses[i].get_bb_min();
        Eigen::Vector4f bb_max = _poses[i].get_bb_max();
        Eigen::Vector4f centroid = _poses[i].get_centroid();
        f = "cube" + boost::lexical_cast<string>(i);
        viewer->addCube(bb_min[0], bb_max[0], bb_min[1], bb_max[1], bb_min[2], bb_max[2], 1, 0, 1, f); // Magenta

        first_centroids->points[i].x = centroid[0];
        first_centroids->points[i].y = centroid[1];
        first_centroids->points[i].z = centroid[2];

//        if (i < _segment_octree_keys.size())
//            ROS_INFO("Current segment %lu has %lu voxels", i, _segment_octree_keys[i].size());
//        else
//            ROS_WARN("Current segment %lu has invalid octree keys vector entry", i);
    }
    visualization::PointCloudColorHandlerCustom<PointT> first_centroid_handler (first_centroids, 230, 20, 230);  // Megenta
    f = "centroids";
    viewer->addPointCloud (first_centroids, first_centroid_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 12, f);

    PointCloud<PointT>::Ptr second_centroids (new PointCloud<PointT>());
    second_centroids->resize(previous_segments.size());
    for (vector<vector<int> >::size_type i = 0; i < previous_segments.size(); ++i)
    {
        // Show the segments
        if (previous_segments[i].size() > 0)
        {
            copyPointCloud(previous_cloud, previous_segments[i], *seg);
            visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 20, 20, 230);  // Blue
            f = "psegment" + boost::lexical_cast<string>(i);
            viewer->addPointCloud (seg, seg_handler, f);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
        }
        else
        {
            // Get the points that are located in the octomap
            vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(_tree, previous_keys[i], _map_key_to_points,
                                                                                         _current_ground_keys);
            // If there are points found
            if (unknown_seg_ix.size() > 0)
            {
                copyPointCloud(_transformed_cloud, unknown_seg_ix, *seg);
                visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 102, 178, 255);  // Light blue
                f = "psegment" + boost::lexical_cast<string>(i);
                viewer->addPointCloud (seg, seg_handler, f);
                viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
            }
        }
        // Show the bounding box
        // CUBE = (xmin, xmax, ymin, ymax, zmin, zmax, r, g, b, id, viewport) // rgb = 0-1
        Eigen::Vector4f bb_min = previous_poses[i].get_bb_min();
        Eigen::Vector4f bb_max = previous_poses[i].get_bb_max();
        Eigen::Vector4f centroid = previous_poses[i].get_centroid();
        f = "pcube" + boost::lexical_cast<string>(i);
        viewer->addCube(bb_min[0], bb_max[0], bb_min[1], bb_max[1], bb_min[2], bb_max[2], 0, 1, 1, f); // Cyan

        second_centroids->points[i].x = centroid[0];
        second_centroids->points[i].y = centroid[1];
        second_centroids->points[i].z = centroid[2];
    }
    visualization::PointCloudColorHandlerCustom<PointT> second_centroid_handler (second_centroids, 20, 230, 230);  // Cyan
    f = "pcentroids";
    viewer->addPointCloud (second_centroids, second_centroid_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 12, f);

    // Visualize the lines connecting the segments and visualize the new bounding boxes
    PointCloud<PointT>::Ptr merged_centroids (new PointCloud<PointT>());
    merged_centroids->resize(_segments.size());
    for (vector<vector<int> >::size_type i = 0; i < overlaps.size(); ++i)
    {
        Eigen::Vector4f first_centroid = _poses[i].get_centroid();
        Eigen::Vector4f first_bb_min = _poses[i].get_bb_min();
        Eigen::Vector4f first_bb_max = _poses[i].get_bb_max();
        // Draw a line from this point to every corresponding point
        for (vector<int>::size_type j = 0; j < overlaps[i].size(); ++j)
        {
            Eigen::Vector4f second_centroid = previous_poses[overlaps[i][j]].get_centroid();
            Eigen::Vector4f second_bb_min = previous_poses[overlaps[i][j]].get_bb_min();
            Eigen::Vector4f second_bb_max = previous_poses[overlaps[i][j]].get_bb_max();
            f = "line" + boost::lexical_cast<string>(i) + boost::lexical_cast<string>(j);
            //addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
            viewer->addLine(first_centroids->points[i], second_centroids->points[overlaps[i][j]], 1, 1, 1, f);
            // Sum the centroid
            for (size_t k = 0; k < 4; ++k)
            {
                first_centroid[k] += second_centroid[k];
                first_bb_min[k] = min(first_bb_min[k], second_bb_min[k]);
                first_bb_max[k] = max(first_bb_max[k], second_bb_max[k]);
            }
        }
        // If there were correspondences
        if (overlaps[i].size() > 0)
        {
            for (size_t k = 0; k < 4; ++k)
                first_centroid[k] = first_centroid[k] / (float) (overlaps[i].size()+1);
        }

        merged_centroids->points[i].x = first_centroid[0];
        merged_centroids->points[i].y = first_centroid[1];
        merged_centroids->points[i].z = first_centroid[2];
        // Show new bounding box
        f = "mcube" + boost::lexical_cast<string>(i);
        viewer->addCube(first_bb_min[0], first_bb_max[0],
                        first_bb_min[1], first_bb_max[1],
                        first_bb_min[2], first_bb_max[2],
                        1, 1, 1, f); // White
    }
    visualization::PointCloudColorHandlerCustom<PointT> merged_centroid_handler (merged_centroids, 255, 255, 255);  // White
    f = "mcentroids";
    viewer->addPointCloud (merged_centroids, merged_centroid_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 12, f);

    // Display the visualiser until 'q' key is pressed
    while (!viewer->wasStopped ())
        viewer->spinOnce ();

    if (viewer)
        delete viewer;
}

bool ActiveExploration::visualize_expected_clouds(const Eigen::Vector4f &location, const vector<int> &seg_indices,
                                                  const vector<vector<PointCloud<PointT> > > &expected_clouds,
                                                  const vector<vector<vector<int> > > &visible_indices)
{
    // Start the visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer ("Visible Points");

    // Unwrap the the transforms from the _instances_to_map_tfs vector
    vector<vector<Eigen::Matrix4f> > itfs;
    itfs.resize(_instances_to_map_tfs.size());
    for (vector<vector<InstToMapTF> >::size_type i = 0; i < _instances_to_map_tfs.size(); ++i)
    {
        itfs[i].resize(_instances_to_map_tfs[i].size());
        for (vector<InstToMapTF>::size_type j = 0; j < _instances_to_map_tfs[i].size(); ++j)
            itfs[i][j] = _instances_to_map_tfs[i][j]._transform;
    }
    // Threads to simultaneously 1) keep viewer open and 2) display clouds and wait for next input to update the clouds
    bool cloud_update_thread_success_val = true;
    bool *cloud_update_thread_success = &cloud_update_thread_success_val;
    boost::thread view_thread = boost::thread(&active_exploration_utils::run_viewer, viewer);
    // View each segment with its overlapping recognised instance and potenttial views
    boost::thread cloud_update_thread = boost::thread(&ActiveExploration::run_view_expected_clouds, this, viewer, location, seg_indices,
                                                      expected_clouds, visible_indices, _emaps, itfs, cloud_update_thread_success);
    // Join the threads
    view_thread.join();
    cloud_update_thread.join();
    // Finished, wait for thread to exit
    ROS_INFO("ActiveExploration::visualize_expected_clouds : finished viewing all point clouds");
    if (!*cloud_update_thread_success)
        ROS_WARN("ActiveExploration::visualize_expected_clouds : cloud update thread exited with FALSE");
    // Close the window
    if (viewer)
        delete viewer;
    if (!*cloud_update_thread_success)
        return false;

    ROS_INFO("ActiveExploration::visualize_expected_clouds : finished");
    return true;

//    // For each segment
//    cout << "Num points in model = " << _emaps[sx][k]._instance_cloud.size() << endl;
//    cout << "Num points in model visible = " << exp_pc.size() << endl;
//    cout << "Num points in world visible = " << vis_ix.size() << endl;
//    cout << "Percentage = " << percent_visible << endl;
}

/* === CHECK DATA === */

bool ActiveExploration::check_vector_sizes()
{
    ROS_INFO("ActiveExploration::check_vector_sizes : starting");
    // The size of all vectors should be the size of the segments
    bool success = true;
    int N_LEN = _segments.size();
    ROS_INFO("Checking the sizes of all vectors ...");
    ROS_INFO("  segments have size   %lu", _segments.size());
    ROS_INFO("  keys have size       %lu", _segment_octree_keys.size());
    ROS_INFO("  poses have size      %lu", _poses.size());
    ROS_INFO("  classes have size    %lu", _class_estimates.size());
    ROS_INFO("  entropies have size  %lu", _entropies.size());
    ROS_INFO("  rankings have size   %lu", _entropy_ranking.size());
    ROS_INFO("  instances have size  %lu", _instance_directories.size());
    ROS_INFO("  transforms have size %lu", _instances_to_map_tfs.size());
    ROS_INFO("  emaps have size      %lu", _emaps.size());

    // Segmented octree keys
    if (_segment_octree_keys.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _segment_octree_keys has incorrect size of %lu, should be %u",
                  _segment_octree_keys.size(), N_LEN);
        success = false;
    }
    // Poses
    if (_poses.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _poses has incorrect size of %lu, should be %u",
                  _poses.size(), N_LEN);
        success = false;
    }
    // Class estimates
    if (_class_estimates.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _class_estimates has incorrect size of %lu, should be %u",
                  _class_estimates.size(), N_LEN);
        success = false;
    }
    // Entropy
    if (_entropies.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _entropies has incorrect size of %lu, should be %u",
                  _entropies.size(), N_LEN);
        success = false;
    }
    // Entropy ranking
    if (_entropy_ranking.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _entropy_ranking has incorrect size of %lu, should be %u",
                  _entropy_ranking.size(), N_LEN);
        success = false;
    }
    // Instance directories
    if (_instance_directories.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _instance_directories has incorrect size of %lu, should be %u",
                  _instance_directories.size(), N_LEN);
        success = false;
    }
    // Transforms
    if (_instances_to_map_tfs.size() != N_LEN)
    {
        ROS_ERROR("ActiveExploration::check_vector_sizes : _instances_to_map_tfs has incorrect size of %lu, should be %u",
                  _instances_to_map_tfs.size(), N_LEN);
        success = false;
    }
    // If any have failed exit
    if (!success)
        return success;

    // Check the individual objects for the length of class estimates, poses and confidences
    // as well as the length of the vectors in _instance_directories, _instances_to_map_tfs and emaps
    for (size_t i = 0; i < _class_estimates.size(); ++i)
    {
        N_LEN = _class_estimates[i].class_type.size();
        // Confidences
        if (_class_estimates[i].confidence.size() != N_LEN)
        {
            ROS_ERROR("ActiveExploration::check_vector_sizes : _class_estimates[%lu].confidence has incorrect size of %lu, should be %u",
                      i, _class_estimates[i].confidence.size(), N_LEN);
            success = false;
        }
        // Poses
        if (_class_estimates[i].pose.size() != N_LEN)
        {
            ROS_ERROR("ActiveExploration::check_vector_sizes : _class_estimates[%lu].pose has incorrect size of %lu, should be %u",
                      i, _class_estimates[i].pose.size(), N_LEN);
            success = false;
        }
        // Instance directories
        if (_instance_directories[i].size() != N_LEN)
        {
            ROS_ERROR("ActiveExploration::check_vector_sizes : _instance_directories[%lu] has incorrect size of %lu, should be %u",
                      i, _instance_directories[i].size(), N_LEN);
            success = false;
        }
        // Transforms
        if (_instances_to_map_tfs[i].size() != N_LEN)
        {
            ROS_ERROR("ActiveExploration (check_vector_sizes) : _instances_to_map_tfs[%lu] has incorrect size of %lu, should be %u",
                      i, _instances_to_map_tfs[i].size(), N_LEN);
            success = false;
        }
        // Emaps
        if (_emaps[i].size() != N_LEN)
        {
            ROS_ERROR("ActiveExploration (check_vector_sizes) : _emaps[%lu] has incorrect size of %lu, should be %u",
                      i, _emaps[i].size(), N_LEN);
            success = false;
        }
    }

    if (!success)
        return false;

    ROS_INFO("ActiveExploration::check_vector_sizes : finished");
    return true;
}

/* === SETTERS === */

void ActiveExploration::set_data(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &image, const Eigen::Matrix4f &transform)
{
//    //clear_hypotheses();
//    set_cloud (cloud);
//    set_image (image);
//    set_transform (transform);
//    transformPointCloud(_cloud, _transformed_cloud, _transform);

    pcl::fromROSMsg(cloud, _cloud);
    _received_cloud = true;

    _image = image;
    _received_image = true;

    _transform = transform;
    _received_transform = true;

    transformPointCloud(_cloud, _transformed_cloud, _transform);

    compute_position();

    if (!add_to_octree())
        ROS_WARN("ActiveExploration::set_data : could not add point cloud to octree");

    ++_num_clouds;
}

void ActiveExploration::set_table_height_threshold(const double &table_height_threshold)
{
    _table_height_threshold = table_height_threshold;
}

void ActiveExploration::turn_on_visualization()
{
    _visualization_on = true;
}

void ActiveExploration::turn_off_visualization()
{
    _visualization_on = false;
}

void ActiveExploration::turn_on_saving(const int &num_objects, const int &num_classes)
{
    _saving_on = true;
    _expected_num_objects = num_objects;
    _expected_num_classes = num_classes;

    // Create the directory if it doesn't already exist
    if (!(boost::filesystem::exists(_save_dir)))
    {
        ROS_WARN("ActiveExploration::turn_on_saving : save directory %s does not exist, creating directory", _save_dir.c_str());
        boost::filesystem::create_directories(_save_dir);
        if (!(boost::filesystem::exists(_save_dir)))
            ROS_ERROR("ActiveExploration::turn_on_saving : could not create directory");
        else
            ROS_INFO("ActiveExploration::turn_on_saving : successfully created directory");
    }

    // Initialize saving files
    initialize_results_file();
    //initialize_object_details_files();
}

void ActiveExploration::turn_off_saving()
{
    _saving_on = false;
}

/* === GETTERS === */

ros::NodeHandle* ActiveExploration::get_ros_node_handle()
{
    return _n;
}

ros::NodeHandle* ActiveExploration::get_ros_node_handle() const
{
    return _n;
}

Eigen::Vector4f ActiveExploration::get_position()
{
    return _position;
}

Eigen::Vector4f ActiveExploration::get_position() const
{
    return _position;
}

PointCloud<PointT> ActiveExploration::get_cloud()
{
    return _cloud;
}

PointCloud<PointT> ActiveExploration::get_cloud() const
{
    return _cloud;
}

PointCloud<PointT> ActiveExploration::get_transformed_cloud()
{
    return _transformed_cloud;
}

PointCloud<PointT> ActiveExploration::get_transformed_cloud() const
{
    return _transformed_cloud;
}

sensor_msgs::PointCloud2 ActiveExploration::get_current_cloud()
{
    return _current_cloud;
}

sensor_msgs::PointCloud2 ActiveExploration::get_current_cloud() const
{
    return _current_cloud;
}

sensor_msgs::Image ActiveExploration::get_image()
{
    return _image;
}

sensor_msgs::Image ActiveExploration::get_image() const
{
    return _image;
}

sensor_msgs::Image ActiveExploration::get_current_image()
{
    return _current_image;
}

sensor_msgs::Image ActiveExploration::get_current_image() const
{
    return _current_image;
}

sensor_msgs::Image ActiveExploration::get_saliency_map()
{
    return _saliency_map;
}

sensor_msgs::Image ActiveExploration::get_saliency_map() const
{
    return _saliency_map;
}

sensor_msgs::Image ActiveExploration::get_current_saliency_map()
{
    return _current_saliency_map;
}

sensor_msgs::Image ActiveExploration::get_current_saliency_map() const
{
    return _current_saliency_map;
}

Eigen::Matrix4f ActiveExploration::get_transform()
{
    return _transform;
}

Eigen::Matrix4f ActiveExploration::get_transform() const
{
    return _transform;
}

Eigen::Matrix4f ActiveExploration::get_current_transform()
{
    return _current_transform;
}

Eigen::Matrix4f ActiveExploration::get_current_transform() const
{
    return _current_transform;
}

bool ActiveExploration::get_flag_received_cloud()
{
    return _received_cloud;
}

bool ActiveExploration::get_flag_received_cloud() const
{
    return _received_cloud;
}

bool ActiveExploration::get_flag_received_image()
{
    return _received_image;
}

bool ActiveExploration::get_flag_received_image() const
{
    return _received_image;
}

bool ActiveExploration::get_flag_received_transform()
{
    return _received_transform;
}

bool ActiveExploration::get_flag_received_transform() const
{
    return _received_transform;
}

vector<vector<int> > ActiveExploration::get_segments()
{
    return _segments;
}

vector<vector<int> > ActiveExploration::get_segments() const
{
    return _segments;
}

vector<vector<OcTreeKey> > ActiveExploration::get_segment_octree_keys()
{
    return _segment_octree_keys;
}

vector<vector<OcTreeKey> > ActiveExploration::get_segment_octree_keys() const
{
    return _segment_octree_keys;
}

vector<Pose> ActiveExploration::get_poses()
{
    return _poses;
}

vector<Pose> ActiveExploration::get_poses() const
{
    return _poses;
}

vector<Classification> ActiveExploration::get_class_estimates()
{
    return _class_estimates;
}

vector<Classification> ActiveExploration::get_class_estimates() const
{
    return _class_estimates;
}

vector<vector<InstLookUp> > ActiveExploration::get_instance_directories()
{
    return _instance_directories;
}

vector<vector<InstLookUp> > ActiveExploration::get_instance_directories() const
{
    return _instance_directories;
}

vector<vector<EntMap> > ActiveExploration::get_emaps()
{
    return _emaps;
}

vector<vector<EntMap> > ActiveExploration::get_emaps() const
{
    return _emaps;
}

vector<vector<InstToMapTF> > ActiveExploration::get_instances_to_map_tfs()
{
    return _instances_to_map_tfs;
}

vector<vector<InstToMapTF> > ActiveExploration::get_instances_to_map_tfs() const
{
    return _instances_to_map_tfs;
}

vector<double> ActiveExploration::get_entropies()
{
    return _entropies;
}

vector<double> ActiveExploration::get_entropies() const
{
    return _entropies;
}

vector<int> ActiveExploration::get_entropy_ranking()
{
    return _entropy_ranking;
}

vector<int> ActiveExploration::get_entropy_ranking() const
{
    return _entropy_ranking;
}

int ActiveExploration::get_num_clouds()
{
    return _num_clouds;
}

int ActiveExploration::get_num_clouds() const
{
    return _num_clouds;
}

string ActiveExploration::get_save_directory()
{
    return _save_dir;
}

string ActiveExploration::get_save_directory() const
{
    return _save_dir;
}

double ActiveExploration::get_max_object_distance()
{
    return _max_object_distance;
}

double ActiveExploration::get_max_object_distance() const
{
    return _max_object_distance;
}

double ActiveExploration::get_min_object_height()
{
    return _min_object_height;
}

double ActiveExploration::get_min_object_height() const
{
    return _min_object_height;
}

double ActiveExploration::get_max_object_height()
{
    return _max_object_height;
}

double ActiveExploration::get_max_object_height() const
{
    return _max_object_height;
}

double ActiveExploration::get_min_object_length()
{
    return _min_object_length;
}

double ActiveExploration::get_min_object_length() const
{
    return _min_object_length;
}

double ActiveExploration::get_max_object_length()
{
    return _max_object_length;
}

double ActiveExploration::get_max_object_length() const
{
    return _max_object_length;
}

double ActiveExploration::get_table_height_threshold()
{
    return _table_height_threshold;
}

double ActiveExploration::get_table_height_threshold() const
{
    return _table_height_threshold;
}

OcTree ActiveExploration::get_octree()
{
    return _tree;
}

OcTree ActiveExploration::get_octree() const
{
    return _tree;
}

int ActiveExploration::get_expected_num_objects()
{
    return _expected_num_objects;
}

int ActiveExploration::get_expected_num_objects() const
{
    return _expected_num_objects;
}

int ActiveExploration::get_expected_num_classes()
{
    return _expected_num_classes;
}

int ActiveExploration::get_expected_num_classes() const
{
    return _expected_num_classes;
}

double ActiveExploration::get_voxel_overlap_threshold()
{
    return _voxel_overlap_threshold;
}

double ActiveExploration::get_voxel_overlap_threshold() const
{
    return _voxel_overlap_threshold;
}

/* === PRIVATE FUNCTIONS === */

bool ActiveExploration::fix_path_names()
{
    if (_class_estimates.size() > 0)
    {
        string from = "//";
        string to = "/";
        for (size_t i = 0; i < _class_estimates.size(); ++i)
        {
            for (vector<std_msgs::String>::iterator it = _class_estimates[i].pose.begin(); it != _class_estimates[i].pose.end(); ++it)
            {
                // Replace the double backslash with a single backslash
                if (it->data.find("//"))
                {
                    string s = it->data;
                    replace_substr(s,from,to);
                    it->data = s;
                }
            }
        }
        return true;
    }
    else
    {
        ROS_ERROR("ActiveExploration::fix_path_names : could not fix path names because _class_estimates has 0 elements");
        return false;
    }
}

bool ActiveExploration::initialize_results_file()
{
    if (_expected_num_classes > 0 && _expected_num_objects > 0)
    {
        // Open the file
        string filename = add_backslash(_save_dir) + _RESULTS_FILE;
        ofstream out;
        out.open(filename.c_str());
        // Write results
        if (out.is_open())
        {
            // current_position num_objects total_entropy entropy+unseen_entropy
            out << 0 << " " << 0 << " " << 0;
            out << " " << _expected_num_objects;
            double max_uncertainty = (double)1 / (double)_expected_num_classes;
            double max_entropy = (double)_expected_num_objects * (double)_expected_num_classes * (-max_uncertainty * log(max_uncertainty));
            out << " " << max_entropy << " " << max_entropy;
            out << "\n";
            out.close();
        }
        else
        {
            ROS_ERROR("ActiveExploration::initialize_results_file : could not open file %s", filename.c_str());
        }
    }
    return true;
}

bool ActiveExploration::initialize_object_details_file()
{
    return true;
}

bool ActiveExploration::save_octree()
{
    string f = add_backslash(_save_dir) + "tree_" + boost::lexical_cast<string>(_num_clouds) + ".bt";
    _tree.writeBinary(f);
    return true;
}

bool ActiveExploration::save_results()
{
    ROS_WARN("ActiveExploration::save_results : starting");
    // If no valid entropy values to write
    if (_entropies.size() == 0)
    {
        ROS_ERROR("ActiveExploration::save_results : entropy vector is empty");
        return false;
    }
    if (_expected_num_classes <= 0)
        ROS_WARN("ActiveExploration::save_results : _expected_num_classes is not set correctly");
    if (_expected_num_objects <= 0)
        ROS_WARN("ActiveExploration::save_results : _expected_num_objects is not set correctly");
    // Open the file
    string filename = add_backslash(_save_dir) + _RESULTS_FILE;
    ofstream out;
    out.open(filename.c_str(), ios::app);
    // Write results
    if (out.is_open())
    {
        // current_position num_objects total_entropy entropy+unseen_entropy
        out << _position[0] << " " << _position[1] << " " << _position[2];
        out << " " << _segments.size();
        double e_sum = 0;
        for (int i = 0; i < _entropies.size(); ++i)
            e_sum += _entropies[i];
        out << " " << e_sum;
        // If the number of segments is less than the number of objects then add extra entropy due to the unseen objects
        if (_entropies.size() < _expected_num_objects && _expected_num_objects > 0)
        {
            vector<double> max_uncertainty;
            max_uncertainty.resize(_expected_num_classes);
            double av_prob = (double)1.0 / (double)_expected_num_classes;
            for (int i = 0; i < _expected_num_classes; ++i)
                max_uncertainty[i] = av_prob;
            // Get the entropy for the maximum uncertainty
            double max_ent = entropy(max_uncertainty);
            // Add this to the sum
            double e_sum_unseen = e_sum + (_expected_num_objects-_entropies.size()) * max_ent;
            // Write this also to the file
            out << " " << e_sum_unseen;
        }
        else
        {
            out << " " << e_sum;
        }
        out << "\n";
        out.close();
        ROS_WARN("ActiveExploration::save_results : finished");
        return true;
    }
    else
    {
        ROS_ERROR("ActiveExploration::save_results : could not open file %s", filename.c_str());
        return true;
    }
    return false;
}

bool ActiveExploration::save_object_details(const vector<vector<int> > &associations, const int &num_previous)
{
    ROS_WARN("ActiveExploration::save_object_details : starting");
    // Get the next index to write to
    int next_index = get_next_write_index(_save_dir, _PROB_FILE_PREFIX);
    if (next_index < 0)
    {
        ROS_ERROR("ActiveExploration::save_object_details : could not find a valid index to for writing");
        return false;
    }
    // If associations is not empty then the associations list needs to be written
    if (associations.size() > 0)
    {
        // If the associations list is not the same size as the segments list
        if (_segments.size() != associations.size())
        {
            ROS_ERROR("ActiveExploration::save_object_details : segments has %lu elements and associations has %lu elements",
                      _segments.size(), associations.size());
            return false;
        }
    }
    // Open the file for writing
    string file_ix = boost::lexical_cast<string>(next_index);
    string filename = add_backslash(_save_dir) + _PROB_FILE_PREFIX + file_ix + _PROB_FILE_EXT;
    ofstream out;
    out.open(filename.c_str());
    if (out.is_open())
    {
        // For each segment
        for (vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
        {
            // Write object id
            out << "object " << i << endl;
            // Write the association vector
            bool save_assoc = false;
            if (i < associations.size())
            {
                if (associations[i].size() > 0)
                {
                    save_assoc = true;
                    for (vector<int>::size_type j = 0; j < associations[i].size(); ++j)
                        out << associations[i][j] << " ";
                    out << endl;
                }
            }
            if (!save_assoc)
                out << "null" << endl;
            // Write the pose
            Eigen::Vector4f centroid = _poses[i].get_centroid();
            Eigen::Vector4f bb_min = _poses[i].get_bb_min();
            Eigen::Vector4f bb_max = _poses[i].get_bb_max();
            out << centroid[0] << " " << centroid[1] << " " << centroid[2] << " "
                << bb_min[0] << " " << bb_min[1] << " " << bb_min[2] << " "
                << bb_max[0] << " " << bb_max[1] << " " << bb_max[2] << endl;
            // Write the class estimates
            for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                out << _class_estimates[i].class_type[j].data << " " << _class_estimates[i].confidence[j] << " ";
            out << endl;
            // Write the entropy
            out << _entropies[i] << endl;
            // Write the point cloud if it is available
            string ob_id = boost::lexical_cast<string>(i);
            string pc_file = add_backslash(_save_dir) + _CLOUD_FILE_PREFIX + file_ix + "_" + ob_id + ".pcd";
            PointCloud<PointT> ob_pc;
            if (_segments[i].size() > 0)
            {
                copyPointCloud(_transformed_cloud, _segments[i], ob_pc);
                io::savePCDFileBinary<PointT>(pc_file, ob_pc);
            }
            else
            {
                // Get the points that are located in the octomap
                vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(_tree, _segment_octree_keys[i], _map_key_to_points,
                                                                                             _current_ground_keys);
                // If there are points found
                if (unknown_seg_ix.size() > 0)
                {
                    copyPointCloud(_transformed_cloud, unknown_seg_ix, ob_pc);
                    io::savePCDFileBinary<PointT>(pc_file, ob_pc);
                }
                else
                {
                    ROS_WARN("ActiveExploration::save_object_details : no cloud available for segment %lu", i);
                }
            }
        }
        out.close();
        ROS_WARN("ActiveExploration::save_object_details : finished");
        return true;
    }
    else
    {
        ROS_ERROR("ActiveExploration::save_object_details : could not open file %s", filename.c_str());
        return true;
    }
    return false;
}

bool ActiveExploration::update_hypotheses(const vector<vector<OcTreeKey> > &pre_octree_keys,
                                          const vector<Classification> &pre_class_estimates, const vector<Pose> &pre_poses,
                                          const vector<vector<InstLookUp> > &pre_instance_directories,
                                          const vector<vector<InstToMapTF> > &pre_transforms,
                                          const vector<vector<int> > &segment_indices,
                                          vector<vector<int> > &overlaps)
{
    // Update
    ROS_INFO("ActiveExploration::update_hypotheses : starting");

    // If the previous active exploration object is empty then just do normal processing
    if (pre_octree_keys.size() == 0)
    {
        ROS_WARN("ActiveExploration::update_hypotheses : previous is empty, performing processing without updates");
        if (segment_indices.size() == 0)
        {
            if (!process())
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : could not process input");
                return false;
            }
        }
        else
        {
            if (!process(segment_indices))
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : could not process input");
                return false;
            }
        }
    }
    else
    {
        // Process the current point cloud
        // Segment
        if (segment_indices.size() == 0)
        {
            if (!segment())
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : error in segmentation");
                return false;
            }
            // Filter out ground, table and objects far away
            if (!filter_segments())
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : error in filtering segmentation");
                return false;
            }
        }
        else
        {
            ROS_WARN("ActiveExploration::update_hypotheses : calling segmentation with specific segment indices");
            if (!segment(segment_indices))
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : error in segmentation");
                return false;
            }
        }
        // Get the keys in the tree for each each segment
        if (!extract_segment_octree_keys())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : error in extracting segment tree keys");
            return false;
        }
        // Estimate poses
        if (!estimate_pose())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : error in estimating poses");
            return false;
        }
        // Classify
        if (!classify())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : error in classification");
            return false;
        }

        // Find the overlaps with the current segmented point cloud
        overlaps = active_exploration_utils::segment_overlap(_poses, _segment_octree_keys, pre_poses, pre_octree_keys, _voxel_overlap_threshold);
        ROS_INFO("ActiveExploration::update_hypotheses : overlaps vector has length %lu", overlaps.size());
        if (_class_estimates.size() != overlaps.size())
            ROS_WARN("ActiveExploration::update_hypotheses : overlaps size %lu does not equal _class_estimates size %lu", overlaps.size(), _class_estimates.size());

        vector<vector<int> > updated_associations;
        Hypothesis result_hyp;
        Hypothesis previous_hyp;
        previous_hyp._octree_keys = pre_octree_keys;
        previous_hyp._class_estimates = pre_class_estimates;
        previous_hyp._poses = pre_poses;
        previous_hyp._instance_directories = pre_instance_directories;
        previous_hyp._transforms = pre_transforms;
        Hypothesis current_hyp;
        current_hyp._segments = _segments;
        current_hyp._octree_keys = _segment_octree_keys;
        current_hyp._class_estimates = _class_estimates;
        current_hyp._poses = _poses;
        current_hyp._instance_directories = _instance_directories;
        current_hyp._transforms = _instances_to_map_tfs;
        current_hyp._emaps = _emaps;
        current_hyp._entropies = _entropies;
        current_hyp._entropy_ranking = _entropy_ranking;

        if (!active_exploration_utils::update_hypotheses(overlaps, _tree, previous_hyp, current_hyp, updated_associations, result_hyp))
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : Could not update hypotheses");
            return false;
        }
        // Otherwise over write the current information
        _segments = result_hyp._segments;
        _segment_octree_keys = result_hyp._octree_keys;
        _class_estimates = result_hyp._class_estimates;
        _poses = result_hyp._poses;
        _instance_directories = result_hyp._instance_directories;
        _instances_to_map_tfs = result_hyp._transforms;
        _emaps = result_hyp._emaps;
        _entropies = result_hyp._entropies;
        _entropy_ranking = result_hyp._entropy_ranking;


//        cout << "segments" << endl;
//        cout << "current has " << current << endl;
//        cout << "previous has " << previous << endl;
//        int count_used = 0;
//        for (size_t i = 0; i < overlaps.size(); ++i)
//        {
//            cout << "seg " << i << " : ";
//            for (size_t j = 0; j < overlaps[i].size(); ++j)
//            {
//                ++count_used;
//                cout << overlaps[i][j] << " ";
//            }
//            cout << endl;
//        }
//        cout << "Total is " << _segments.size() << endl;
//        cout << "Total should be : current + (previous - count_used) = " << current
//             << " + (" << previous << " - " << count_used << ") = "
//             << current + (previous - count_used) << endl;

//        cout << "FOUND SEG OVERLAPS";
//        cin.ignore();

        // Retrive the entropy maps
        if (!retrieve_entropy_maps())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : error in retrieveing entropy maps");
            return false;
        }
        // Compute entropy
        if (!compute_entropy())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : error in computing entropy");
            return false;
        }
        // Rank entropies
        if (!rank_entropy())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : error in ranking entropy");
            return false;
        }
        // Check the size of all vectors
        if (!check_vector_sizes())
        {
            ROS_ERROR("ActiveExploration::update_hypotheses : vector sizes do not match");
            return false;
        }
        // Save data to file
        if (_saving_on)
        {
            if (!save_results())
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : error in saving results");
                return false;
            }
//            if (!save_octree())
//            {
//                ROS_ERROR("ActiveExploration::update_hypotheses : error in saving octree");
//                return false;
//            }
            if (!save_object_details(updated_associations, pre_class_estimates.size()))  // CHANGED, using updated associations now
            {
                ROS_ERROR("ActiveExploration::update_hypotheses : error in saving object details");
                return false;
            }
        }
    }

    // Successfully updated the hypotheses with a new input
    ROS_INFO("ActiveExploration::update_hypotheses : finished");
    return true;
}

bool ActiveExploration::update_class_estimates(const vector<vector<int> > &overlaps, const vector<vector<OcTreeKey> > &octree_keys,
                                               const vector<Classification> &class_estimates, const vector<Pose> &poses,
                                               const vector<vector<InstLookUp> > &instance_directories,
                                               const vector<vector<InstToMapTF> > &transforms,
                                               vector<vector<int> > &updated_associations)
{
    ROS_INFO("ActiveExploration::update_class_estimates : starting");
    //                   class , p file , conf, inst dir  , transform
    typedef boost::tuple<string, string, float, InstLookUp, InstToMapTF> class_info;

    // Add a small epsilon to each entropy
    for (size_t i = 0; i < _class_estimates.size(); ++i)
    {
        for (size_t j = 0; j < _class_estimates[i].confidence.size(); ++j)
            _class_estimates[i].confidence[j] += _EPS;
    }
    vector<Classification> class_estimates_cp = class_estimates;
    for (size_t i = 0; i < class_estimates_cp.size(); ++i)
    {
        for (size_t j = 0; j < class_estimates_cp[i].confidence.size(); ++j)
            class_estimates_cp[i].confidence[j] += _EPS;
    }

//    // Print out the information
//    cout << "** BEFORE UPDATING ** PREVIOUS CLASS RESULTS **" << endl;
//    cout << "Size octree keys " << octree_keys.size() << endl;
//    cout << "Size poses " << poses.size() << endl;
//    cout << "Size class estimates " << class_estimates.size() << endl;
//    cout << "Size instance directories " << instance_directories.size() << endl;
//    cout << "Size instance tfs " << transforms.size() << endl;
//    for (size_t i = 0; i < class_estimates.size(); ++i)
//    {
//        cout << "Segment " << i << endl;
//        cout << "num ests = " << class_estimates[i].class_type.size()
//             << " num dirs = " << instance_directories[i].size()
//             << " num tfs = " << transforms[i].size() << endl;
//        for (size_t j = 0; j < class_estimates[i].class_type.size(); ++j)
//        {
//            cout << "class " << j << endl;
//            cout << "   est " << class_estimates[i].class_type[j].data << endl;
//            cout << "   conf " << class_estimates[i].confidence[j] << endl;
//            cout << "   inst " << instance_directories[i][j]._class_type << "/"
//                                << instance_directories[i][j]._instance_name << "/"
//                                << instance_directories[i][j]._ix << endl;
//            cout << "   tf " << transforms[i][j]._transform << endl;
//            cout << "   score " << transforms[i][j]._score << endl;
//        }
//    }

//    cout << "** BEFORE UPDATING ** LATEST CLASS RESULTS **" << endl;
//    cout << "Size segments " << _segments.size() << endl;
//    cout << "Size octree keys " << _segment_octree_keys.size() << endl;
//    cout << "Size poses " << _poses.size() << endl;
//    cout << "Size class estimates " << _class_estimates.size() << endl;
//    cout << "Size instance directories " << _instance_directories.size() << endl;
//    cout << "Size instance tfs " << _instances_to_map_tfs.size() << endl;
//    for (size_t i = 0; i < _class_estimates.size(); ++i)
//    {
//        cout << "Segment " << i << endl;
//        cout << "num ests = " << _class_estimates[i].class_type.size()
//             << " num dirs = " << _instance_directories[i].size()
//             << " num tfs = " << _instances_to_map_tfs[i].size() << endl;
//        for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
//        {
//            cout << "class " << j << endl;
//            cout << "   est " << _class_estimates[i].class_type[j].data << endl;
//            cout << "   conf " << _class_estimates[i].confidence[j] << endl;
//            cout << "   inst " << _instance_directories[i][j]._class_type << "/"
//                                << _instance_directories[i][j]._instance_name << "/"
//                                << _instance_directories[i][j]._ix << endl;
//            cout << "   tf " << _instances_to_map_tfs[i][j]._transform << endl;
//            cout << "   score " << _instances_to_map_tfs[i][j]._score << endl;
//        }
//    }

    // Assumes that the current _class_estimates is resized
    if (_class_estimates.size() != overlaps.size())
    {
        ROS_ERROR("ActiveExploration::update_class_estimates : incorrect class estimates size");
        ROS_ERROR("_class_estimates is %lu and overlaps is %lu", _class_estimates.size(), overlaps.size());
        return false;
    }
    // Go through the vector of overlaps and update the class estimates
    vector<int> merged_into_current;
    vector<vector<class_info> > train_instances;
    vector<int> used_ix;
    updated_associations.clear();
    updated_associations.resize(overlaps.size());
    for (vector<vector<int> >::size_type i = 0; i < overlaps.size(); ++i)
    {
        ROS_INFO("Segment %lu", i);
        updated_associations[i] = overlaps[i];
        // If this segment has not already been merged into a previous segment
        if (find(merged_into_current.begin(), merged_into_current.end(), i) == merged_into_current.end())
        {
            ROS_INFO(" - original class estimates -");
            for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                ROS_INFO("  %-15s %.2f", _class_estimates[i].class_type[j].data.c_str(), _class_estimates[i].confidence[j]);
//            ROS_INFO(" - original instance directories -");
//            for (size_t j = 0; j < _instance_directories[i].size(); ++j)
//                ROS_INFO("  %s/%s/%s", _instance_directories[i][j]._class_type.c_str(), _instance_directories[i][j]._instance_name.c_str(),
//                                       _instance_directories[i][j]._ix.c_str());
            // Maintain a vector of the best scoring poses and transform files
            vector<class_info> best_train_instance;
            double min_eps = numeric_limits<double>::infinity();
            for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
            {
                best_train_instance.push_back(boost::make_tuple(_class_estimates[i].class_type[j].data,
                                                                _class_estimates[i].pose[j].data,
                                                                _class_estimates[i].confidence[j],
                                                                _instance_directories[i][j],
                                                                _instances_to_map_tfs[i][j]));
                if (_class_estimates[i].confidence[j] < min_eps)
                    min_eps = _class_estimates[i].confidence[j];
            }
            // Check the min eps value, if it is larger than _EPS then use _EPS
            if (min_eps > _EPS)
                min_eps = _EPS;
            // If any other segment has the same overlaps then must also include this in the new segment and merge them together
            vector<int> merges;
            vector<int> curr_overlaps = overlaps[i];
            for (vector<int>::size_type j = i+1; j < overlaps.size(); ++j)
            {
                bool do_merge = false;
                for (vector<int>::size_type k = 0; k < overlaps[i].size(); ++k)
                {
                    // If overlaps[j] contains any element in overlaps[i]
                    // then merge overlaps[i] and overlaps[j]
                    if (find(overlaps[j].begin(), overlaps[j].end(),overlaps[i][k]) != overlaps[j].end())
                    {
                        merges.push_back(j);
                        do_merge = true;
                    }
                }
                // Add the merged object's overlaps to the current list of overlaps if they don't already exist
                if (do_merge)
                {
                    // Include overlaps[j] into the list of overlaps[i]
                    curr_overlaps.insert(curr_overlaps.end(), overlaps[j].begin(), overlaps[j].end());
                    sort(curr_overlaps.begin(), curr_overlaps.end());
                    curr_overlaps.erase(unique(curr_overlaps.begin(), curr_overlaps.end()), curr_overlaps.end());
                    merged_into_current.push_back(j);
                    // Add to the new associations
                    updated_associations[i].insert(updated_associations[i].end(), overlaps[j].begin(), overlaps[j].end());
                    sort(updated_associations[i].begin(), updated_associations[i].end());
                    updated_associations[i].erase(unique(updated_associations[i].begin(), updated_associations[i].end()),
                                                  updated_associations[i].end());
                }
            }
            // If found merges
            if (merges.size() > 0)
            {
                // Remove duplicates in merges
                sort(merges.begin(), merges.end());
                merges.erase(unique(merges.begin(), merges.end()), merges.end());
                // Merge
                for (vector<int>::size_type j = 0; j < merges.size(); ++j)
                {
                    // Merge segmentation
                    if (!merge_segments(i, _segments[merges[j]]))
                    {
                        ROS_ERROR("ActiveExploration::update_class_estimates : could not update segmentation %lu with current segment %u",
                                  i, merges[j]);
                        return false;
                    }
                    // Merge the class estimates
                    if (!merge_estimates(i, _class_estimates[merges[j]], min_eps))
                    {
                        ROS_ERROR("ActiveExploration::update_class_estimates : could not update class estimate %lu with current segment %u",
                                  i, merges[j]);
                        return false;
                    }
                    // Merge the segment octree keys
                    if (!merge_segment_octree_keys(i, _segment_octree_keys[merges[j]]))
                    {
                        ROS_ERROR("ActiveExploration::update_class_estimates : could not update segment tree keys %lu with current segment %u",
                                  i, merges[j]);
                        return false;
                    }
                    // Merge the poses
                    if (!merge_poses(i, _poses[merges[j]]))
                    {
                        ROS_ERROR("ActiveExploration::update_class_estimates : could not update pose %lu with current segment %u",
                                  i, merges[j]);
                        return false;
                    }
                    vector<int> not_matched;
                    for (size_t k = 0; k < best_train_instance.size(); ++k)
                        not_matched.push_back(k);
                    // Update the best files and transforms vector
                    for (size_t k = 0; k < _class_estimates[merges[j]].class_type.size(); ++k)
                    {
                        bool found_match = false;
                        for (size_t kk = 0; kk < best_train_instance.size(); ++kk)
                        {
                            // If the same class
                            if (strcmp(_class_estimates[merges[j]].class_type[k].data.c_str(), best_train_instance[kk].get<0>().c_str()) == 0)
                            {
                                found_match = true;
//                                // If the confidence is better then replace with the files
//                                if (_class_estimates[merges[j]].confidence[k] > best_train_instance[kk].get<2>())
                                // If the alignment score is better then replace with the files and transforms
                                if (_instances_to_map_tfs[merges[j]][k]._score < best_train_instance[kk].get<4>()._score)
                                {
                                    // Replace
                                    best_train_instance[kk] = boost::make_tuple(_class_estimates[merges[j]].class_type[k].data,
                                                                                _class_estimates[merges[j]].pose[k].data,
                                                                                _class_estimates[merges[j]].confidence[k],
                                                                                _instance_directories[merges[j]][k],
                                                                                _instances_to_map_tfs[merges[j]][k]);
                                }
                                vector<int>::iterator pos = find(not_matched.begin(), not_matched.end(), kk);
                                if (pos != not_matched.end())
                                    not_matched.erase(pos);
                            }
                        }
                        // If not found then need to add this new class to the list,
                        // but the confidence must be multiplied by a small epsilon value
                        if (!found_match)
                        {
                            best_train_instance.push_back(boost::make_tuple(_class_estimates[merges[j]].class_type[k].data,
                                                                            _class_estimates[merges[j]].pose[k].data,
                                                                            _class_estimates[merges[j]].confidence[k]*min_eps,
                                                                            _instance_directories[merges[j]][k],
                                                                            _instances_to_map_tfs[merges[j]][k]));
                        }
                    }
                    // Add the unmatched elements from the merge index
                    for (vector<int>::size_type k = 0; k < not_matched.size(); ++k)
                    {
                        // The confidence is multiplied by min_eps because it has 0 confidence since it was not found
                        // in the first classification
                        best_train_instance[k] = boost::make_tuple(best_train_instance[k].get<0>(),
                                                                   best_train_instance[k].get<1>(),
                                                                   best_train_instance[k].get<2>()*min_eps,
                                                                   best_train_instance[k].get<3>(),
                                                                   best_train_instance[k].get<4>());
                    }
                }
            }
            // Update with all of the previous segments by searching through the overlaps
            for (vector<int>::size_type j = 0; j < curr_overlaps.size(); ++j)
            {
                // Merge the class estimates
                if (!merge_estimates(i, class_estimates_cp[curr_overlaps[j]], min_eps))
                {
                    ROS_ERROR("ActiveExploration::update_class_estimates : could not update class estimate %lu", i);
                    return false;
                }
                // Merge the segment tree keys
                if (!merge_segment_octree_keys(i, octree_keys[curr_overlaps[j]]))
                {
                    ROS_ERROR("ActiveExploration::update_class_estimates : could not update segment octree keys %lu", i);
                    return false;
                }
                // Merge the poses
                if (!merge_poses(i, poses[curr_overlaps[j]]))
                {
                    ROS_ERROR("ActiveExploration::update_class_estimates : could not update pose %lu", i);
                    return false;
                }
                vector<int> not_matched;
                for (size_t k = 0; k < best_train_instance.size(); ++k)
                    not_matched.push_back(k);
                // Update the best files vector
                for (size_t k = 0; k < class_estimates_cp[curr_overlaps[j]].class_type.size(); ++k)
                {
                    bool found_match = false;
                    for (size_t kk = 0; kk < best_train_instance.size(); ++kk)
                    {
                        // If the same class
                        if (strcmp(class_estimates_cp[curr_overlaps[j]].class_type[k].data.c_str(), best_train_instance[kk].get<0>().c_str()) == 0)
                        {
                            found_match = true;
//                            // If the confidence is better then replace with the files belonging to this object
//                            if (class_estimates[curr_overlaps[j]].confidence[k] > best_train_instance[kk].get<2>())
                            // If the alignment score is better then replace with the files and transforms
                            if (transforms[curr_overlaps[j]][k]._score < best_train_instance[kk].get<4>()._score)
                            {
                                // Replace
                                best_train_instance[kk] = boost::make_tuple(class_estimates_cp[curr_overlaps[j]].class_type[k].data,
                                                                            class_estimates_cp[curr_overlaps[j]].pose[k].data,
                                                                            class_estimates_cp[curr_overlaps[j]].confidence[k],
                                                                            instance_directories[curr_overlaps[j]][k],
                                                                            transforms[curr_overlaps[j]][k]);
                            }
                            vector<int>::iterator pos = find(not_matched.begin(), not_matched.end(), kk);
                            if (pos != not_matched.end())
                                not_matched.erase(pos);
                        }
                    }
                    // If not found then need to add this new class to the list
                    // but the confidence must be multiplied by a small epsilon value
                    if (!found_match)
                    {
                        best_train_instance.push_back(boost::make_tuple(class_estimates_cp[curr_overlaps[j]].class_type[k].data,
                                                                        class_estimates_cp[curr_overlaps[j]].pose[k].data,
                                                                        class_estimates_cp[curr_overlaps[j]].confidence[k]*min_eps,
                                                                        instance_directories[curr_overlaps[j]][k],
                                                                        transforms[curr_overlaps[j]][k]));
                    }
                }
                // Add the unmatched elements from the merge index
                for (vector<int>::size_type k = 0; k < not_matched.size(); ++k)
                {
                    // The confidence is multiplied by min_eps because it has 0 confidence since it was not found
                    // in the first classification
                    best_train_instance[k] = boost::make_tuple(best_train_instance[k].get<0>(),
                                                               best_train_instance[k].get<1>(),
                                                               best_train_instance[k].get<2>()*min_eps,
                                                               best_train_instance[k].get<3>(),
                                                               best_train_instance[k].get<4>());
                }
                // Add the index to the list of used indices
                used_ix.push_back(curr_overlaps[j]);
            }
            train_instances.push_back (best_train_instance);
            ROS_INFO(" - combined class estimates -");
            for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                ROS_INFO("  %-15s %.2f", _class_estimates[i].class_type[j].data.c_str(), _class_estimates[i].confidence[j]);
        }
    }

    // Erase the segment indices, class estimates and poses
    // that were merged into other segments
    sort(merged_into_current.begin(), merged_into_current.end());  // sort
    merged_into_current.erase(unique(merged_into_current.begin(), merged_into_current.end()), merged_into_current.end());  // unique
    reverse(merged_into_current.begin(), merged_into_current.end());  // reverse
    ROS_INFO("Removing %lu segments that were merged", merged_into_current.size());
    // Iterate from highest index to lowest
    for (vector<int>::size_type i = 0; i < merged_into_current.size(); ++i)
    {
        _segments.erase(_segments.begin() + merged_into_current[i]);
        _class_estimates.erase(_class_estimates.begin() + merged_into_current[i]);
        _segment_octree_keys.erase(_segment_octree_keys.begin() + merged_into_current[i]);
        _poses.erase(_poses.begin() + merged_into_current[i]);
        updated_associations.erase(updated_associations.begin() + merged_into_current[i]);
    }
    // Update the pose files, instance training directories and transforms
    _instance_directories.clear();
    _instance_directories.resize(_class_estimates.size());
    _instances_to_map_tfs.clear();
    _instances_to_map_tfs.resize(_class_estimates.size());
    //ROS_INFO("Updating pose estimates to highest scoring classification");
    for (size_t i = 0; i < _class_estimates.size(); ++i)
    {
        _instance_directories[i].resize(_class_estimates[i].class_type.size());
        _instances_to_map_tfs[i].resize(_class_estimates[i].class_type.size());
        // Add to the lists in order of the confidences
        for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
        {
            // Find the corresponding class type in the best_train_instance vector
            for (size_t k = 0; k < train_instances[i].size(); ++k)
            {
                // If the same class type
                if (strcmp(_class_estimates[i].class_type[j].data.c_str(),train_instances[i][k].get<0>().c_str()) == 0)
                {
                    _class_estimates[i].pose[j].data = train_instances[i][k].get<1>();
                    _instance_directories[i][j] = train_instances[i][k].get<3>();
                    _instances_to_map_tfs[i][j] = train_instances[i][k].get<4>();
                    break;
                }
            }
        }
    }
    // Add any objects from the previous list that were not matched with any objects in the current list
    sort(used_ix.begin(), used_ix.end());
    used_ix.erase(unique(used_ix.begin(), used_ix.end()),used_ix.end());
    //ROS_INFO("Adding segments that were missed");
    for (size_t i = 0; i < class_estimates.size(); ++i)
    {
        // If this index is not in the used_ix list
        if (find(used_ix.begin(),used_ix.end(),i) == used_ix.end())
        {
            //ROS_INFO("   adding %lu",i);
            // Append this object to the list of hypotheses
            // Empty segment (because there is no reference to a point cloud)
            vector<int> seg;
            _segments.push_back(seg);
            // Segment octree keys
            _segment_octree_keys.push_back (octree_keys[i]);
            // Pose
            _poses.push_back(poses[i]);
            // Class estimate
            _class_estimates.push_back(class_estimates[i]);
            // Instance directory
            _instance_directories.push_back(instance_directories[i]);
            // Transforms
            _instances_to_map_tfs.push_back(transforms[i]);
            // New associations
            vector<int> temp;
            temp.push_back(i);
            updated_associations.push_back(temp);
        }
    }

//    // Print out the merged information
//    cout << "Size segments " << _segments.size() << endl;
//    cout << "Size octree keys " << _segment_octree_keys.size() << endl;
//    cout << "Size poses " << _poses.size() << endl;
//    cout << "Size class estimates " << _class_estimates.size() << endl;
//    cout << "Size instance directories " << _instance_directories.size() << endl;
//    cout << "Size instance tfs " << _instances_to_map_tfs.size() << endl;
//    for (size_t i = 0; i < _class_estimates.size(); ++i)
//    {
//        cout << "Segment " << i << endl;
//        cout << "num ests = " << _class_estimates[i].class_type.size()
//             << " num dirs = " << _instance_directories[i].size()
//             << " num tfs = " << _instances_to_map_tfs[i].size() << endl;
//        for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
//        {
//            cout << "class " << j << endl;
//            cout << "   est " << _class_estimates[i].class_type[j].data << endl;
//            cout << "   conf " << _class_estimates[i].confidence[j] << endl;
//            cout << "   inst " << _instance_directories[i][j]._class_type << "/"
//                                << _instance_directories[i][j]._instance_name << "/"
//                                << _instance_directories[i][j]._ix << endl;
//            cout << "   tf " << _instances_to_map_tfs[i][j]._transform << endl;
//            cout << "   score " << _instances_to_map_tfs[i][j]._score << endl;
//        }
//    }


//    cin.ignore();

    ROS_INFO("ActiveExploration::update_class_estimates : finished");
    return true;
}

bool ActiveExploration::merge_segments(const int &i, const vector<int> &seg)
{
    // Check it is a valid index
    if (i < 0)
    {
        ROS_ERROR("ActiveExploration::merge_segments : input index %u is negative", i);
        return false;
    }
    if (i >= _segments.size())
    {
        ROS_ERROR("ActiveExploration::merge_segments : input index %u is too large for _segments of size %lu", i, _segments.size());
        return false;
    }
    // Otherwise merge the segments
    vector<int> s_new;
    if (!merge_segments(_segments[i], seg, s_new))
    {
        ROS_ERROR("ActiveExploration::merge_segments : could not merge segments");
        return false;
    }
    _segments[i] = s_new;
    return true;
}

bool ActiveExploration::merge_segments(const vector<int> &first, const vector<int> &second, vector<int> &output)
{
    // Clear the output object
    output.clear();

    if (first.size() == 0)
    {
        ROS_WARN("ActiveExploration::merge_segments : first segment has no elements");
        output = second;
        return true;
    }
    if (second.size() == 0)
    {
        ROS_WARN("ActiveExploration::merge_segments : second segment has no elements");
        output = first;
        return true;
    }
    // Otherwise merge the segments
    output = first;
    for (size_t i = 0; i < second.size(); ++i)
        output.push_back(second[i]);
    // Sort
    sort(output.begin(), output.end());
    // Remove duplicates
    output.erase(unique(output.begin(), output.end()), output.end());
    return true;
}

bool ActiveExploration::merge_estimates(const int &i, const Classification &est, const double &min_eps)
{
    // Check it is a valid index
    if (i < 0)
    {
        ROS_ERROR("ActiveExploration::merge_segments : input index %u is negative", i);
        return false;
    }
    if (i >= _class_estimates.size())
    {
        ROS_ERROR("ActiveExploration::merge_segments : input index %u is too large for _class_estimates of size %lu", i, _class_estimates.size());
        return false;
    }
    // Otherwise merge the class estimates
    Classification c = _class_estimates[i];
    Classification c_new;
    if (!merge_estimates(c, est, min_eps, c_new))
    {
        ROS_ERROR("ActiveExploration::merge_segments : could not merge estimates");
        return false;
    }
    _class_estimates[i] = c_new;
    return true;
}

bool ActiveExploration::merge_estimates(const Classification &first, const Classification &second, const double &min_eps, Classification &output)
{
    typedef boost::tuple<string,string,float> class_info;

    // Clear the output object
    output.class_type.clear();
    output.confidence.clear();
    output.pose.clear();

    if (first.class_type.size() == 0)
    {
        ROS_WARN("ActiveExploration::merge_segments : first class estimate has no elements");
        output = second;
        return true;
    }
    if (second.class_type.size() == 0)
    {
        ROS_WARN("ActiveExploration::merge_segments : second class estimate has no elements");
        output = first;
        return true;
    }
    // Otherwise merge the estimates
    vector<int> not_matched;
    for (size_t i = 0; i < second.class_type.size(); ++i)
        not_matched.push_back(i);
    // Go through each element and update the class
    vector<class_info> combined_res;
    combined_res.resize(first.class_type.size());
    for (size_t i = 0; i < first.class_type.size(); ++i)
    {
        combined_res[i] = boost::make_tuple(first.class_type[i].data, first.pose[i].data, first.confidence[i]);
        bool found_in_second = false;
        for (size_t j = 0; j < second.class_type.size(); ++j)
        {
            // If the same class
            if (strcmp(first.class_type[i].data.c_str(), second.class_type[j].data.c_str()) == 0)
            {
                found_in_second = true;
                combined_res[i] = boost::make_tuple(combined_res[i].get<0>(),
                                                    combined_res[i].get<1>(),
                                                    combined_res[i].get<2>()*second.confidence[j]);
                vector<int>::iterator pos = find(not_matched.begin(), not_matched.end(), j);
                if (pos != not_matched.end())
                    not_matched.erase(pos);
            }
        }
        // If this element was not found in the second then must multiply the confidence by
        // min_eps because it has essentially 0 confidence in the second classification
        if (!found_in_second)
            combined_res[i] = boost::make_tuple(first.class_type[i].data, first.pose[i].data, first.confidence[i]*min_eps);
    }
    // Add the unmatched elements from second
    for (vector<int>::size_type i = 0; i < not_matched.size(); ++i)
    {
        // The confidence is multiplied by min_eps because it has 0 confidence since it was not found
        // in the first classification
        combined_res.push_back(boost::make_tuple(second.class_type[not_matched[i]].data,
                                                 second.pose[not_matched[i]].data,
                                                 second.confidence[not_matched[i]]*min_eps));
    }
    // Normalise the confidences
    float normalization_const = 0;
    for (vector<class_info>::size_type i = 0; i < combined_res.size(); ++i)
        normalization_const += combined_res[i].get<2>();
    vector<pair<int,float> > sort_vec;
    for (vector<class_info>::size_type i = 0; i < combined_res.size(); ++i)
    {
        combined_res[i] = boost::make_tuple(combined_res[i].get<0>(),
                                            combined_res[i].get<1>(),
                                            combined_res[i].get<2>()/normalization_const);
        sort_vec.push_back(make_pair(i,combined_res[i].get<2>()));
    }
    // Sort the classes in order of highest confidence to lowest confidence
    sort(sort_vec.begin(), sort_vec.end(), compare<float>);
    reverse(sort_vec.begin(), sort_vec.end());
    // Assign the values to the output object
    output.class_type.resize(combined_res.size());
    output.confidence.resize(combined_res.size());
    output.pose.resize(combined_res.size());
    for (vector<class_info>::size_type i = 0; i < combined_res.size(); ++i)
    {
        std_msgs::String str_tmp;
        str_tmp.data = combined_res[sort_vec[i].first].get<0>();
        output.class_type[i] = str_tmp;
        str_tmp.data = combined_res[sort_vec[i].first].get<1>();
        output.pose[i] = str_tmp;
        output.confidence[i] = combined_res[sort_vec[i].first].get<2>();
//        cout << "class " << combined_res[sort_vec[i].first].get<0>()
//             << " conf = " << combined_res[sort_vec[i].first].get<2>() << endl;
    }

    return true;
}

bool ActiveExploration::merge_segment_octree_keys(const int &i, const vector<OcTreeKey> &keys)
{
    // Check it is a valid index
    if (i < 0)
    {
        ROS_ERROR("ActiveExploration::merge_segment_octree_keys : input index %u is negative", i);
        return false;
    }
    if (i >= _segment_octree_keys.size())
    {
        ROS_ERROR("ActiveExploration::merge_segment_octree_keys : input index %u is too large for _segment_octree_keys of size %lu",
                  i, _poses.size());
        return false;
    }
    // Otherwise merge the poses
    vector<OcTreeKey> k = _segment_octree_keys[i];
    vector<OcTreeKey> k_new;
    if (!merge_segment_octree_keys(k, keys, k_new))
    {
        ROS_ERROR("ActiveExploration::merge_segment_octree_keys : could not merge segment octree keys");
        return false;
    }
    _segment_octree_keys[i] = k_new;
    return true;
}

bool ActiveExploration::merge_segment_octree_keys(const vector<OcTreeKey> &first, const vector<OcTreeKey> &second, vector<OcTreeKey> &output)
{
    output.clear();
    if (first.size() == 0)
    {
        ROS_WARN("ActiveExploration::merge_segment_octree_keys : first vector is empty");
        for (size_t i = 0; i < second.size(); ++i)
        {
            OcTreeNode *node = _tree.search(second[i]);
            if (node)
            {
                if (node->getOccupancy() > 0.5)
                    output.push_back(second[i]);
                else
                    cout << "Rejecting key" << endl;
            }
        }
        //output = second;
        return true;
    }
    if (second.size() == 0)
    {
        ROS_WARN("ActiveExploration::merge_segment_octree_keys : second vector is empty");
        for (size_t i = 0; i < first.size(); ++i)
        {
            OcTreeNode *node = _tree.search(first[i]);
            if (node)
            {
                if (node->getOccupancy() > 0.5)
                    output.push_back(first[i]);
                else
                    cout << "Rejecting key" << endl;
            }
        }
        //output = first;
        return true;
    }
    // Otherwise merge the octree key vectors
    output.clear();
    // Insert the values from the first vector
    for (size_t i = 0; i < first.size(); ++i)
    {
        OcTreeNode *node = _tree.search(first[i]);
        if (node)
        {
            if (node->getOccupancy() > 0.5)
                output.push_back(first[i]);
        }
    }
    // Insert the values from the second vector
    for (size_t i = 0; i < second.size(); ++i)
    {
        OcTreeNode *node = _tree.search(second[i]);
        if (node)
        {
            if (node->getOccupancy() > 0.5)
                output.push_back(second[i]);
        }
    }
//    // Insert the values from the first vector
//    output.insert (output.end(), first.begin(), first.end());
//    // Insert the values from the second vector
//    output.insert (output.end(), second.begin(), second.end());
    // Sort
    sort(output.begin(), output.end(), compare_octree_key);
    // Remove duplicates
    output.erase(unique(output.begin(), output.end(), equal_octree_key), output.end());

    return true;
}

bool ActiveExploration::merge_poses(const int &i, const Pose &pose)
{
    // Check it is a valid index
    if (i < 0)
    {
        ROS_ERROR("ActiveExploration::merge_poses : input index %u is negative", i);
        return false;
    }
    if (i >= _poses.size())
    {
        ROS_ERROR("ActiveExploration::merge_poses : input index %u is too large for _poses of size %lu", i, _poses.size());
        return false;
    }
    // Otherwise merge the poses
    Pose p = _poses[i];
    Pose p_new;
    if (!merge_poses(p, pose, p_new))
    {
        ROS_ERROR("ActiveExploration::merge_poses : could not merge poses");
        return false;
    }
    _poses[i] = p_new;
    return true;
}

bool ActiveExploration::merge_poses(const Pose &first, const Pose &second, Pose &output)
{
    if (!first.is_valid())
    {
        ROS_WARN("ActiveExploration::merge_poses : first pose is invalid");
        output = second;
        return true;
    }
    if (!second.is_valid())
    {
        ROS_WARN("ActiveExploration::merge_poses : second pose is invalid");
        output = first;
        return true;
    }
    // Otherwise merge the poses
    // Compute the mean of the centroid
    Eigen::Vector4f first_centroid = first.get_centroid();
    Eigen::Vector4f second_centroid = second.get_centroid();
    Eigen::Vector4f mean_centroid;
    for (size_t i = 0; i < 4; ++i)
        mean_centroid[i] = (first_centroid[i] + second_centroid[i]) / 2;
    // Compute the minimum min bounding box
    Eigen::Vector4f first_bb_min = first.get_bb_min();
    Eigen::Vector4f second_bb_min = second.get_bb_min();
    Eigen::Vector4f min_min;
    for (size_t i = 0; i < 4; ++i)
        min_min[i] = min(first_bb_min[i], second_bb_min[i]);
    // Compute the maximum max bounding box
    Eigen::Vector4f first_bb_max = first.get_bb_max();
    Eigen::Vector4f second_bb_max = second.get_bb_max();
    Eigen::Vector4f max_max;
    for (size_t i = 0; i < 4; ++i)
        max_max[i] = max(first_bb_max[i], second_bb_max[i]);
    // Create the new pose
    output = Pose(mean_centroid, min_min, max_max);

    return true;
}

bool ActiveExploration::extract_instance_directories()
{
    _instance_directories.clear();
    if (_class_estimates.size() > 0)
    {
        _instance_directories.resize(_class_estimates.size());
        for (size_t i = 0; i < _class_estimates.size(); ++i)
        {
            for (vector<std_msgs::String>::const_iterator it = _class_estimates[i].pose.begin(); it != _class_estimates[i].pose.end(); ++it)
            {
                string inst_dir = _NULL_FILE;
                string class_type = rem_backslash(_class_estimates[i].class_type[distance<vector<std_msgs::String>::const_iterator>
                                    (_class_estimates[i].pose.begin(),it)].data);
                string inst_name = _NULL_FILE;
                string ix = _NULL_FILE;
                // Extract the pose filename
                string s = it->data;
                // Split the filename and read off the directory and the integer of the pose
                size_t found = s.find_last_of("/\\");
                if (found != string::npos)
                {
                    string folder = s.substr(0,found); // path to instance
                    string pos = s.substr(found+1);  // name of instance in the directory
                    // The instance folder is one up from "folder" (the descriptors type)
                    found = folder.find_last_of("/\\");
                    if (found != string::npos)
                    {
                        inst_dir = folder.substr(0,found);
                        // The instance name the last directory in the path
                        size_t found2 = inst_dir.find_last_of("/\\");
                        if (found2 != string::npos)
                            inst_name = inst_dir.substr(found2+1);
                        else
                            ROS_WARN("ActiveExploration::extract_instance_directories : could not get instance name from path %s",
                                     inst_dir.c_str());
                    }
                    else
                    {
                        ROS_WARN("ActiveExploration::extract_instance_directories : could not get instance path in directory %s",
                                 folder.c_str());
                    }
                    // The index is between the underscore and dot in pos
                    size_t underscore = pos.find_last_of("_");
                    size_t dot = pos.find_last_of(".");
                    if (underscore != string::npos && dot != string::npos && (dot - found) > 0)
                    {
                        int str_len = dot - underscore;
                        ix = pos.substr(underscore+1,str_len-1);
                        //ix = atoi(str_ix.c_str());
                    }
                    else
                    {
                        ROS_WARN("ActiveExploration::extract_instance_directories : could not get the pose number in file %s",
                                 pos.c_str());
                    }
                }
                else
                {
                    ROS_WARN("ActiveExploration::extract_instance_directories : could not split filename %s",
                             s.c_str());
                }
                // Add to the list
                InstLookUp lu;
                lu._dir = inst_dir;
                lu._class_type = class_type;
                lu._instance_name = inst_name;
                lu._ix = ix;
                _instance_directories[i].push_back(lu);
                //cout << "pose file " << s << endl;
                //cout << "instance directory " << inst_dir << endl;
                //cout << "class " << class_type << endl;
                //cout << "instance name " << inst_name << endl;
                //cout << "index " << ix << endl;
            }
        }
        return true;
    }
    else
    {
        ROS_ERROR("ActiveExploration::extract_instance_directories : could not extract instance directory because _class_estmates is has 0 elements");
        return false;
    }
}

bool ActiveExploration::transform_instances_to_map()
{
    return transform_instances_to_map(_instances_to_map_tfs);
}

bool ActiveExploration::transform_instances_to_map(vector<vector<InstToMapTF> > &transforms)
{
    transforms.clear();
    if (_segments.size() == 0)
    {
        ROS_ERROR("ActiveExploration::transform_instances_to_map : segments has zero elements");
        return false;
    }
    // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud
    transforms.resize(_segments.size());
    // For each segment
    for (vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
    {
        if (_segments[i].size() == 0)
        {
            ROS_WARN("ActiveExploration::transform_instances_to_map : segment %lu has no points", i);
            InstToMapTF itf;
            itf._transform = Eigen::Matrix4f::Identity();
            itf._score = numeric_limits<double>::infinity();
            transforms[i].push_back (itf);
        }
        else if (_instance_directories[i].size() == 0)
        {
            ROS_WARN("ActiveExploration::transform_instances_to_map : segment %lu has no recognised instances", i);
            InstToMapTF itf;
            itf._transform = Eigen::Matrix4f::Identity();
            itf._score = numeric_limits<double>::infinity();
            transforms[i].push_back (itf);
        }
        else
        {
            // Get the subcloud
            PointCloud<PointT> seg;
            if (!_received_transform)
            {
                ROS_WARN("ActiveExploration::transform_instances_to_map : no available transform");
                copyPointCloud(_cloud, _segments[i], seg);
            }
            else
            {
                copyPointCloud(_transformed_cloud, _segments[i], seg);
            }

            // Read the clouds
            for (vector<InstLookUp>::size_type j = 0; j < _instance_directories[i].size(); ++j)
            {
                string view_file = add_backslash(_instance_directories[i][j]._dir) + "view_" + _instance_directories[i][j]._ix + ".pcd";
                //cout << "view file " << view_file << endl;
                // If the view file is invalid
                if (strcmp(_instance_directories[i][j]._class_type.c_str(),_NULL_FILE) == 0 ||
                    strcmp(_instance_directories[i][j]._instance_name.c_str(),_NULL_FILE) == 0 ||
                    strcmp(_instance_directories[i][j]._ix.c_str(),_NULL_FILE) == 0)
                {
                    ROS_WARN("ActiveExploration::transform_instances_to_map : invalid instance directory %s/%s/%s",
                             _instance_directories[i][j]._class_type.c_str(), _instance_directories[i][j]._instance_name.c_str(),
                             _instance_directories[i][j]._ix.c_str());
                    InstToMapTF itf;
                    itf._transform = Eigen::Matrix4f::Identity();
                    itf._score = numeric_limits<double>::infinity();
                    transforms[i].push_back (itf);
                }
                // Otherwise
                else
                {
                    PointCloud<PointT> view;
                    if (io::loadPCDFile<PointT> (view_file.c_str(), view) == -1)
                        ROS_WARN("ActiveExploration::transform_instances_to_map : could not load point cloud %s", view_file.c_str());
                    Eigen::Matrix4f tr;
                    double score;
                    //transform_cloud_to_cloud (view, seg, _poses[i].get_centroid(), tr, score);
                    transform_cloud_to_cloud (view, seg, tr, score);
                    // TODO check that this is correct
                    // tr brings it to the map frame (i.e. segment * _transform)
                    // _transform.inverse() * tr brings the point cloud back to the raw frame
                    InstToMapTF itf;
                    itf._transform = tr;
                    itf._score = score;
                    transforms[i].push_back (itf);
                    //cout << "transform segment " << i << " instance " << j << endl;
                    //cout << view_file << endl;
                    //cout << tr << endl;


//                    // Check what the clouds look like
//                    cout << "icp transform" << endl;
//                    cout << tr << endl;
//                    Eigen::Matrix4f tri = _transform.inverse();
//                    io::savePCDFileASCII("observation.pcd", *cloud);
//                    PointCloud<PointT>::Ptr cloud_transformed (new PointCloud<PointT>());
//                    transformPointCloud (*cloud, *cloud_transformed, _transform);
//                    io::savePCDFileASCII("transformed_observation.pcd", *cloud_transformed);
//                    PointCloud<PointT>::Ptr original (new PointCloud<PointT>());
//                    copyPointCloud(*cloud, _segments[i].data, *original);
//                    io::savePCDFileASCII("original_segment.pcd", *original);
//                    PointCloud<PointT>::Ptr seg_transformed (new PointCloud<PointT>());
//                    transformPointCloud (*original, *seg_transformed, _transform);
//                    io::savePCDFileASCII("transformed_segment.pcd", *seg_transformed);
//                    io::savePCDFileASCII("original_view.pcd", *view);
//                    PointCloud<PointT>::Ptr view_transform (new PointCloud<PointT>());
//                    transformPointCloud (*view, *view_transform, tr);
//                    io::savePCDFileASCII("transformed_view.pcd", *view_transform);
//                    PointCloud<PointT>::Ptr view_transformed_to_raw (new PointCloud<PointT>());
//                    Eigen::Matrix4f tr_raw = _transform.inverse() * tr;
//                    transformPointCloud (*view, *view_transformed_to_raw, tr_raw);
//                    io::savePCDFileASCII("transformed_to_raw_view.pcd", *view_transformed_to_raw);
//                    return false;


                }
            }
        }
    }
    return true;
}

bool ActiveExploration::transform_instances_to_map(vector<vector<Eigen::Matrix4f> > &transforms)
{
    vector<vector<InstToMapTF> > itfs;
    if (!transform_instances_to_map(itfs))
    {
        ROS_ERROR("ActiveExploration::transform_instances_to_map : could not get transforms");
        return false;
    }
    // Otherwise unwrap into just the transforms
    transforms.clear();
    transforms.resize(itfs.size());
    for (vector<vector<InstToMapTF> >::size_type i = 0; i < itfs.size(); ++i)
    {
        transforms[i].resize(itfs[i].size());
        for (vector<InstToMapTF>::size_type j = 0; j < itfs[i].size(); ++j)
            transforms[i][j] = itfs[i][j]._transform;
    }
    return true;
}

bool ActiveExploration::add_to_octree()
{
    ROS_INFO("ActiveExploration::add_to_octree : adding point cloud to octree");
    // Otherwise insert the point cloud
    point3d pos (_position[0], _position[1], _position[2]);
    sensor_msgs::PointCloud2 sm_cloud;
    pcl::toROSMsg(_transformed_cloud, sm_cloud);
    octomap::Pointcloud o_cloud;
    pointCloud2ToOctomap(sm_cloud, o_cloud);
    _tree.insertPointCloud(o_cloud, pos);

    _map_key_to_points.clear();
    for (size_t i = 0; i < _transformed_cloud.size(); ++i)
    {
        OcTreeKey key;
        if (_tree.coordToKeyChecked(_transformed_cloud.points[i].x, _transformed_cloud.points[i].y, _transformed_cloud.points[i].z, key))
            _map_key_to_points[key].push_back(i);
    }

    return true;
}

void ActiveExploration::compute_position()
{
    // Compute the current position from the point cloud
    Eigen::Vector4f in_pos = extract_camera_position (_cloud);
    if (_received_transform)
        _position = transform_eigvec(in_pos, _transform);
    else
        _position = in_pos;
//    // Transform to model coordinate frame
//    PointCloud<PointT> pt;
//    pt.resize(1);
//    pt.points[0].x = in_pos[0];
//    pt.points[0].y = in_pos[1];
//    pt.points[0].z = in_pos[2];
//    // Transform to map
//    transformPointCloud(pt, pt, _transform);
//    // Convert back to Eigen::Vector4f
//    _position[0] = pt.points[0].x;
//    _position[1] = pt.points[0].y;
//    _position[2] = pt.points[1].z;
}

bool ActiveExploration::is_valid_segment(const PointCloud<PointT> &cloud, const int &ix)
{
    PointT min_pt, max_pt;
    getMinMax3D (cloud, min_pt, max_pt);
    double x_length = fabs(max_pt.data[0] - min_pt.data[0]);
    double y_length = fabs(max_pt.data[1] - min_pt.data[1]);
    double z_height = fabs(max_pt.data[2] - min_pt.data[2]);
    // Reject objects thare are too tall, e.g. people, walls, or too small
    if (z_height < _min_object_height || z_height > _max_object_height)
    {
        ROS_INFO("utils::is_valid_segment : rejecting segment %u which has height %.2f", ix, z_height);
        return false;
    }
    // Reject objects that are too short or too long
    else if (x_length < _min_object_length || x_length > _max_object_length)
    {
        ROS_INFO("utils::is_valid_segment : rejecting segment %u which has x length %.2f", ix, x_length);
        return false;
    }
    else if (y_length < _min_object_length || y_length > _max_object_length)
    {
        ROS_INFO("utils::is_valid_segment : rejecting segment %u which has y length %.2f", ix, y_length);
        return false;
    }

    // Return
    return true;
}

bool ActiveExploration::is_valid_segment(const PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const int &ix)
{
    // Transform the point cloud
    PointCloud<PointT> transformed_cloud;
    transformPointCloud(cloud, transformed_cloud, transform);
    return is_valid_segment(transformed_cloud, ix);
}

bool ActiveExploration::fromROSMsg(const squirrel_object_perception_msgs::EntropyMap &in, EntMap &out)
{
    try
    {
        // Clear the data
        out._ixs.clear();
        out._clouds.clear();
        out._transforms.clear();
        out._centroids.clear();
        out._camera_poses.clear();
        out._camera_orientations.clear();
        out._surface_areas.clear();
        out._class_entropies.clear();
        out._recognition_probabilities.clear();
        // Set the cloud
        PointCloud<PointT> total_cloud;
        pcl::fromROSMsg(in.response.cloud, total_cloud);
        out._instance_cloud = total_cloud;
        // Get the octree filename
        out._octree_file = in.response.octree_file.data;
        // For each view
        for (size_t i = 0; i < in.response.entropy_map.size(); ++i)
        {
            // View index
            out._ixs.push_back(atoi(in.response.entropy_map[i].view_name.data.c_str()));
            // Point cloud
            PointCloud<PointT> cloud;
            pcl::fromROSMsg(in.response.entropy_map[i].cloud, cloud);
            out._clouds.push_back (cloud);
            // Transform to the model
            std_msgs::Float64MultiArray in_t = in.response.entropy_map[i].transform;
            Eigen::Matrix4f out_t = Eigen::Matrix4f::Identity();
            int r = 0, c = 0;
            for (int m = 0; m < in_t.data.size(); ++m)
            {
                out_t(r,c) = in_t.data[m];
                ++c;
                if (c == 4)
                {
                    ++r;
                    c = 0;
                }
            }
            out._transforms.push_back(out_t);
            // Centroid
            Eigen::Vector4f centroid = Eigen::Vector4f(0,0,0,0);
            for (int j = 0; j < 4; ++j)
                centroid[j] = in.response.entropy_map[i].centroid[0];
            out._centroids.push_back (centroid);
            // Camera Pose and orientation
            geometry_msgs::Pose ros_pos = in.response.entropy_map[i].camera_pose;
            Eigen::Vector4f camera_pos = Eigen::Vector4f(0,0,0,0);
            camera_pos[0] = ros_pos.position.x;
            camera_pos[1] = ros_pos.position.y;
            camera_pos[2] = ros_pos.position.z;
            out._camera_poses.push_back (camera_pos);
            Eigen::Quaternionf camera_ori (ros_pos.orientation.w, ros_pos.orientation.x,
                                           ros_pos.orientation.y, ros_pos.orientation.z);
            out._camera_orientations.push_back (camera_ori);
            // Surface area
            out._surface_areas.push_back(in.response.entropy_map[i].surface_area_proportion);
            // Class entropy
            out._class_entropies.push_back(in.response.entropy_map[i].class_entropy);
            // Recognition probability
            out._recognition_probabilities.push_back(in.response.entropy_map[i].recognition_probability);
        }
        out._valid = true;
        return true;
    }
    catch (...)
    {
        ROS_ERROR("ActiveExploration::fromROSMsg : could not convert ros message to EntMap");
        return false;
    }
}

void ActiveExploration::run_view_segment(visualization::PCLVisualizer *viewer, const std::vector<std::vector<EntMap> > &emaps,
                                         const std::vector<std::vector<Eigen::Matrix4f> > &itfs, const SIM_TYPE &sim, bool *exit_status)
{
    ROS_INFO("ActiveExploration::run_view_segment : starting");
    *exit_status = true;

    // Add the current cloud
    PointCloud<PointT>::Ptr scene_cloud (new PointCloud<PointT>(_transformed_cloud));
    visualization::PointCloudColorHandlerCustom<PointT> scene_pts_handler (scene_cloud, 255, 255, 255);  // White
    viewer->addPointCloud (scene_cloud, scene_pts_handler, "scene_cloud");
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_cloud");

    enum UTIL_TYPE {AREA = 0, ENTROPY = 1, PROBABILITY = 2};
    UTIL_TYPE u_type = AREA;
    if (sim == MIN_CLASS_ENTROPY || sim == MIN_CLASS_ENTROPY_UNOCCLUDED || sim == NEAREST_MIN_ENTROPY)
        u_type = ENTROPY;
    else if (sim == MAX_CLASS_PROB || sim == MAX_CLASS_PROB_UNOCCLUDED || sim == NEAREST_MAX_CLASS_PROB)
        u_type = PROBABILITY;

    // Display the segment point clouds one at a time
    string s_name, i_total_name, i_view_name, c_name, m_name, e_name;
    char input;
    PointCloud<PointT>::Ptr seg (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr i_total (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr i_view (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr c_view (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr m_view (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr e_view (new PointCloud<PointT>());
    for (size_t i = 0; i < _segments.size(); ++i)
    {
        // Next input
        cout << "Enter for next view or q to quit ...";
        input = cin.get();
        if (input == 'q' || input == 'Q')
        {
            ROS_WARN("ActiveExploration::run_view_segment : exiting loop after %lu/%lu viewpoints", i+1, _segments.size());
            *exit_status = false;
            break;
        }

        // Get the segments, instances and views of instances

        // Segment
        seg->clear();
        s_name = "segment_cloud_" + boost::lexical_cast<string>(i);
        copyPointCloud(*scene_cloud, _segments[i], *seg);

        // Instance total cloud and the most likely view that matches
        i_total->clear();
        i_view->clear();
        i_total_name = "instance_cloud_" + boost::lexical_cast<string>(i);
        i_view_name = "view_cloud_" + boost::lexical_cast<string>(i);
        string view_file = add_backslash(_instance_directories[i][0]._dir) + "view_" + _instance_directories[i][0]._ix + ".pcd";
        //cout << "best match is " << view_file << endl;
        int view_ix = atoi(_instance_directories[i][0]._ix.c_str()); // most likely viewpoint
        int vec_ix = 0; // vector index
        vector<int>::const_iterator it = find(emaps[i][0]._ixs.begin(), emaps[i][0]._ixs.end(), view_ix);
        if (it != emaps[i][0]._ixs.end())
            vec_ix = distance<vector<int>::const_iterator>(emaps[i][0]._ixs.begin(), it);
        else
            ROS_WARN("ActiveExploration::run_view_segment : cannot find view index %u in entropy map %lu (%s/%s)",
                     view_ix, i, _instance_directories[i][0]._class_type.c_str(), _instance_directories[i][0]._instance_name.c_str());

        if (emaps[i].size() > 0)
        {
            // Transform the model cloud to map frame
            // itfs[i][vec_ix] gives the transform for the instance to the model frame
            // A) Bring the model to the instance frame using emaps[i][vec_ix].inverse()
            // B) Bring the model from the instance frame to the map frame using itfs[i][vec_ix]
            // Matrix multiplication works by:
            //     X*Y*v = X*(Y*v) --> from right to left --> last transform is applied first
            // Therefore want T = B*A


            //Eigen::Matrix4f i_transform = itfs[i][0] * _transforms_to_instances[i][0].inverse();  // CHANGED
            Eigen::Matrix4f i_transform = itfs[i][0] * emaps[i][0]._transforms[vec_ix].inverse();

            transformPointCloud(emaps[i][0]._instance_cloud, *i_total, i_transform);
            // Transform the view points to the map frame
            transformPointCloud(emaps[i][0]._clouds[vec_ix], *i_view, itfs[i][0]);
        }
        else
        {
            ROS_WARN("ActiveExploration::run_view_segment : segment %lu has no instances", i);
        }

        // Camera views for this instance
        c_view->clear();
        c_name = "camera_positions_" + boost::lexical_cast<string>(i);
        for (size_t j = 0; j < emaps[i][0]._camera_poses.size(); ++j)
        {
            PointCloud<PointT> pt;
            pt.resize(1);
            pt.points[0].x = emaps[i][0]._camera_poses[j][0];
            pt.points[0].y = emaps[i][0]._camera_poses[j][1];
            pt.points[0].z = emaps[i][0]._camera_poses[j][2];
            // Transform to model frame
            transformPointCloud(pt, pt, emaps[i][0]._transforms[j]);
            // Add to list
            c_view->push_back(pt.points[0]);
        }
        Eigen::Matrix4f i_transform = itfs[i][0] * emaps[i][0]._transforms[vec_ix].inverse();
        transformPointCloud(*c_view, *c_view, i_transform);

        // The camera position for this particular view
        m_view->clear();
        m_name = "model_position_" + boost::lexical_cast<string>(i);
        m_view->push_back (c_view->points[vec_ix]);

        // The location with the largest visible surface area or the lowest entropy
        int best_ix = -1;
        if (u_type == ENTROPY)
            best_ix = min_element(emaps[i][0]._class_entropies.begin(),
                                  emaps[i][0]._class_entropies.end()) -
                                  emaps[i][0]._class_entropies.begin();
        else if (u_type == PROBABILITY)
            best_ix = min_element(emaps[i][0]._recognition_probabilities.begin(),
                                  emaps[i][0]._recognition_probabilities.end()) -
                                  emaps[i][0]._recognition_probabilities.begin();
        else
            best_ix = max_element(emaps[i][0]._surface_areas.begin(),
                                  emaps[i][0]._surface_areas.end()) -
                                  emaps[i][0]._surface_areas.begin();
//        for (int j = 0; j < emaps[i][0]._surface_areas.size(); ++j)
//            cout << j << " -> " << emaps[i][0]._surface_areas[j] << endl;
//        cout << "Index for best entropy is " << best_ix << endl;
//        cout << "Original location is " << emaps[i][0]._camera_poses[best_ix][0] << " "
//                                        << emaps[i][0]._camera_poses[best_ix][1] << " "
//                                        << emaps[i][0]._camera_poses[best_ix][2] << endl;
        if (best_ix > 0)
        {
            e_view->clear();
            e_name = "best_entropy_position_" + boost::lexical_cast<string>(i);
            e_view->push_back (c_view->points[best_ix]);
        }

        // Visualize
        // Segment
        visualization::PointCloudColorHandlerCustom<PointT> segment_handler (seg, 230, 20, 20); // Red
        viewer->addPointCloud (seg, segment_handler, s_name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, s_name);
        printf(ANSI_COLOR_RED  "SEGMENT"  ANSI_COLOR_RESET);
        cin.ignore();
        // View cloud
        visualization::PointCloudColorHandlerCustom<PointT> instance_view_handler (i_view, 20, 230, 20); // Green
        viewer->addPointCloud (i_view, instance_view_handler, i_view_name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, i_view_name);
        // Camera position for model
        visualization::PointCloudColorHandlerCustom<PointT> model_position_handler (m_view, 20, 230, 20); // Green
        viewer->addPointCloud (m_view, model_position_handler, m_name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, m_name);
        printf(ANSI_COLOR_GREEN  "INSTANCE VIEW POINT"  ANSI_COLOR_RESET);
        cin.ignore();
        // Instance cloud
        visualization::PointCloudColorHandlerCustom<PointT> instance_handler (i_total, 230, 230, 20); // Yellow
        viewer->addPointCloud (i_total, instance_handler, i_total_name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, i_total_name);
        printf(ANSI_COLOR_YELLOW  "FULL INSTANCE CLOUD"  ANSI_COLOR_RESET "\n");
        // Camera positions
        visualization::PointCloudColorHandlerCustom<PointT> camera_positions_handler (c_view, 20, 20, 230); // Blue
        viewer->addPointCloud (c_view, camera_positions_handler, c_name);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 8, c_name);
        printf(ANSI_COLOR_BLUE  "TRAINING VIEWS"  ANSI_COLOR_RESET "\n");
//        // Re-show the camera position for model
//        viewer->removePointCloud(m_name);
//        viewer->addPointCloud (m_view, model_position_handler, m_name);
//        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, m_name);
        // Camera position for lowest entropy location
        printf(ANSI_COLOR_MAGENTA  "LOWEST ENTROPY VIEW POINT"  ANSI_COLOR_RESET);
        if (best_ix > 0 && e_view->size() > 0)
        {
            visualization::PointCloudColorHandlerCustom<PointT> entropy_position_handler (e_view, 255, 0, 255); // Magenta
            viewer->addPointCloud (e_view, entropy_position_handler, e_name);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, e_name);
        }
        else
        {
            ROS_WARN("ActiveExploration::run_view_segment : cannot display best index %i and points %lu", best_ix, e_view->size());
        }
        cin.ignore();
    }

    ROS_INFO("ActiveExploration::run_view_segment : finished");
    return;
}

void ActiveExploration::run_view_expected_clouds(visualization::PCLVisualizer *viewer, const Eigen::Vector4f &location,
                                                 const vector<int> &seg_indices, const vector<vector<PointCloud<PointT> > > &expected_clouds,
                                                 const vector<vector<vector<int> > > &visible_indices, const vector<vector<EntMap> > &emaps,
                                                 const vector<vector<Eigen::Matrix4f> > &itfs, bool *exit_status)
{
    ROS_INFO("ActiveExploration::run_view_expected_clouds : starting");
    *exit_status = true;

    int v1 = 0;
    viewer->createViewPort(0.0, 0.0, 0.6, 1.0, v1);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v1); // Setting background to a dark grey
    int v2 = 1;
    viewer->createViewPort(0.6, 0.5, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v2); // Setting background to a dark grey
    int v3 = 2;
    viewer->createViewPort(0.6, 0.0, 1.0, 0.5, v3);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v3); // Setting background to a dark grey

    // Add the current cloud
    PointCloud<PointT>::Ptr scene_cloud (new PointCloud<PointT>(_transformed_cloud));
    visualization::PointCloudColorHandlerCustom<PointT> scene_pts_handler (scene_cloud, 255, 255, 255);  // White
    viewer->addPointCloud (scene_cloud, scene_pts_handler, "scene_cloud", v1);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud", v1);
    // Add the map location
    PointCloud<PointT>::Ptr location_cloud (new PointCloud<PointT>());
    location_cloud->resize(1);
    location_cloud->points[0].x = location[0];
    location_cloud->points[0].y = location[1];
    location_cloud->points[0].z = location[2];
    visualization::PointCloudColorHandlerCustom<PointT> location_pts_handler (scene_cloud, 20, 230, 230);  // Cyan
    viewer->addPointCloud (location_cloud, location_pts_handler, "location", v1);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "location", v1);

    // Display the segment point clouds one at a time with their predicted views
    string s_name, i_total_name, e_name, e_name2, v_name, v_name2;
    char input;
    PointCloud<PointT>::Ptr seg (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr i_total (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr e_view (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr v_view (new PointCloud<PointT>());
    for (vector<int>::size_type i = 0; i < seg_indices.size(); ++i)
    {
        // Next input
        cout << "Enter for next view or q to quit ...";
        input = cin.get();
        if (input == 'q' || input == 'Q')
        {
            ROS_WARN("ActiveExploration::run_view_expected_clouds : exiting loop after %lu/%lu viewpoints", i+1, _segments.size());
            *exit_status = false;
            break;
        }

        // Get the segments, instances and views of instances
        int seg_ix = seg_indices[i];

        // Segment
        seg->clear();
        s_name = "segment_cloud_" + boost::lexical_cast<string>(seg_ix);
        copyPointCloud(*scene_cloud, _segments[seg_ix], *seg);

        // Instance total cloud
        i_total->clear();
        i_total_name = "instance_cloud_" + boost::lexical_cast<string>(seg_ix);
        int view_ix = atoi(_instance_directories[seg_ix][0]._ix.c_str()); // most likely viewpoint
        int vec_ix = 0; // vector index
        vector<int>::const_iterator it = find(emaps[seg_ix][0]._ixs.begin(), emaps[seg_ix][0]._ixs.end(), view_ix);
        if (it != emaps[seg_ix][0]._ixs.end())
            vec_ix = distance<vector<int>::const_iterator>(emaps[seg_ix][0]._ixs.begin(), it);
        else
            ROS_WARN("ActiveExploration::run_view_expected_clouds : cannot find view index %u in entropy map %u (%s/%s)",
                     view_ix, seg_ix, _instance_directories[seg_ix][0]._class_type.c_str(), _instance_directories[seg_ix][0]._instance_name.c_str());
        if (emaps[i].size() > 0)
        {
            // Transform the model cloud to map frame
            // itfs[i][vec_ix] gives the transform for the instance to the model frame
            // A) Bring the model to the instance frame using emaps[i][vec_ix].inverse()
            // B) Bring the model from the instance frame to the map frame using itfs[i][vec_ix]
            // Matrix multiplication works by:
            //     X*Y*v = X*(Y*v) --> from right to left --> last transform is applied first
            // Therefore want T = B*A
            Eigen::Matrix4f i_transform = itfs[seg_ix][0] * emaps[seg_ix][0]._transforms[vec_ix].inverse();
            transformPointCloud(emaps[seg_ix][0]._instance_cloud, *i_total, i_transform);
        }
        else
        {
            ROS_WARN("ActiveExploration::run_view_expected_clouds : segment %u has no instances", seg_ix);
        }

        // The expected point cloud
        e_view->clear();
        e_name = "expected_cloud_" + boost::lexical_cast<string>(seg_ix);
        e_name2 = "expected_cloud_" + boost::lexical_cast<string>(seg_ix) + "_2";
        copyPointCloud(expected_clouds[i][0], *e_view);

        // The visible points in the world
        v_view->clear();
        v_name = "visible_cloud_" + boost::lexical_cast<string>(seg_ix);
        v_name2 = "visible_cloud_" + boost::lexical_cast<string>(seg_ix) + "_2";
        copyPointCloud(expected_clouds[i][0], visible_indices[i][0], *v_view);

        // Visualize
        // Segment
        visualization::PointCloudColorHandlerCustom<PointT> segment_handler (seg, 230, 20, 20); // Red
        viewer->addPointCloud (seg, segment_handler, s_name, v1);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, s_name, v1);
        printf(ANSI_COLOR_RED  "SEGMENT"  ANSI_COLOR_RESET);
        cin.ignore();
        // Instance cloud
        visualization::PointCloudColorHandlerCustom<PointT> instance_handler (i_total, 230, 230, 20); // Yellow
        viewer->addPointCloud (i_total, instance_handler, i_total_name, v1);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, i_total_name, v1);
        printf(ANSI_COLOR_YELLOW  "FULL INSTANCE CLOUD (%lu)"  ANSI_COLOR_RESET "\n", i_total->size());
        cin.ignore();
        // Expected cloud
        // View port 1
        visualization::PointCloudColorHandlerCustom<PointT> expected_cloud_handler (e_view, 20, 230, 230); // Cyan
        viewer->addPointCloud (e_view, expected_cloud_handler, e_name, v1);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, e_name, v1);
        // View port 2
        visualization::PointCloudColorHandlerCustom<PointT> expected_cloud_handler2 (e_view, 20, 230, 230); // yan
        viewer->addPointCloud (e_view, expected_cloud_handler2, e_name2, v2);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, e_name2, v2);
        printf(ANSI_COLOR_CYAN "EXPECTED CLOUD (%lu)"  ANSI_COLOR_RESET "\n", e_view->size());
        cin.ignore();
        // Visible cloud
        // View port 1
        visualization::PointCloudColorHandlerCustom<PointT> visible_cloud_handler (v_view, 255, 0, 255); // Magenta
        viewer->addPointCloud (v_view, visible_cloud_handler, v_name, v1);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, v_name, v1);
        // View port 3
        visualization::PointCloudColorHandlerCustom<PointT> visible_cloud_handler2 (v_view, 255, 0, 255); // Magenta
        viewer->addPointCloud (v_view, visible_cloud_handler2, v_name2, v3);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, v_name, v3);
        printf(ANSI_COLOR_MAGENTA "VISIBLE CLOUD (%lu)"  ANSI_COLOR_RESET "\n", v_view->size());
        cin.ignore();
    }

    ROS_INFO("ActiveExploration::run_view_expected_clouds : finished");
    return;
}

bool ActiveExploration::next_best_view(int &next_best_index, const SIM_TYPE &sim, const vector<Eigen::Vector4f> &map_locations, const double &variance)
{
    ROS_INFO("ActiveExploration::next_best_view : starting");
    next_best_index = -1;

    if (map_locations.size() == 0)
    {
        ROS_ERROR("ActiveExploration::next_best_view : map locations is empty");
        return false;
    }
    // If the sim type is random then return a random index
    if (sim == RANDOM)
    {
        next_best_index = int_rand(0, map_locations.size());
        return true;
    }
    // If extracting the expected view and classifying it
    bool classify_view = false;
    if (sim == MIN_VIEW_CLASSIFICATION_ENTROPY || sim == MAX_VIEW_CLASSIFICATION_PROB)
        classify_view = true;
    // If need to choose the nearest view or use choose by weighting the controibution of all possible views
    bool all_model_views = true;
    if (sim == NEAREST_AREA || sim == NEAREST_MIN_ENTROPY || sim == NEAREST_MAX_CLASS_PROB)
        all_model_views = false;
    // Surface area, class entropy or recognition probability
    enum UTIL_TYPE {AREA = 0, ENTROPY = 1, PROBABILITY = 2};
    UTIL_TYPE u_type = AREA;
    if (sim == MIN_CLASS_ENTROPY || sim == MIN_CLASS_ENTROPY_UNOCCLUDED || sim == NEAREST_MIN_ENTROPY)
        u_type = ENTROPY;
    else if (sim == MAX_CLASS_PROB || sim == MAX_CLASS_PROB_UNOCCLUDED || sim == NEAREST_MAX_CLASS_PROB)
        u_type = PROBABILITY;
    // Occlusion or not
    bool unoccluded = false;
    if (sim == MAX_AREA_UNOCCLUDED || sim == MIN_CLASS_ENTROPY_UNOCCLUDED || sim == MAX_CLASS_PROB_UNOCCLUDED)
        unoccluded = true;

    // Otherwise compute the next best view by selecting the location in map_locations that has the highest utility
    // Ignore segments which are already very well classified
    vector<int> segs_for_planning;
    // Sum the entropies to get the normlization constant
    double segs_max_ent = -1;
    for (vector<vector<int> >::size_type i = 0; i < _segments.size(); ++i)
    {
        // Get the confidence of the most likely class
        if (_class_estimates[i].confidence[0] < _CONFIDENCE_THRESHOLD)
        {
            segs_for_planning.push_back (i);
            if (_entropies[i] > segs_max_ent)
                segs_max_ent = _entropies[i];
        }
    }
    // If segs for planning is empty then just plan for the objct with the  highest entropy
    if (segs_for_planning.size() == 0)
    {
        // If there are no rankings available
        if (_entropy_ranking.size() == 0)
        {
            ROS_ERROR("ActiveExploration::next_best_view : no available entropy rankings");
            return false;
        }
        // Otherwise
        segs_for_planning.push_back(_entropy_ranking[0]);
        segs_max_ent = _entropies[_entropy_ranking[0]];
    }
    // Scale the entropy values
    if (segs_max_ent <= 0)
    {
        ROS_WARN("ActiveExploration::next_best_view : scaling factor for segment entropy is %.4f", segs_max_ent);
        segs_max_ent = 1;
    }
    vector<double> uncertainty_weight;
    uncertainty_weight.resize(segs_for_planning.size());
    for (vector<int>::size_type i = 0; i < segs_for_planning.size(); ++i)
        uncertainty_weight[i] = _entropies[segs_for_planning[i]]/segs_max_ent;

    // If the sim type requires the views to be extracted and classified
    if (classify_view)
    {
        next_best_index = extracted_point_cloud_next_best_view(map_locations, segs_for_planning, uncertainty_weight, sim);
    }
    else
    {
        // Compute the utility for each segment from the surface area proportions or the class entropy values
        // High surface area proportion means high utility
        // Low entropy (low uncertainty) means high utility
        // Find the maximum value and scale everything by this value (it will then be 1 and everything less)
        // If using entropy hen let [utility = 1 - scaled_entropy] to set high utility to low entropy
        vector<vector<double> > model_views_max_score;
        model_views_max_score.resize(segs_for_planning.size());
        for (vector<int>::size_type i = 0; i < segs_for_planning.size(); ++i)
        {
            int sx = segs_for_planning[i];
            model_views_max_score[i].resize(_emaps[sx].size());
            for (vector<EntMap>::size_type j = 0; j < _emaps[sx].size(); ++j)
            {
                size_t max_iter;
                if (u_type == ENTROPY)
                    max_iter = _emaps[sx][j]._class_entropies.size();
                if (u_type == PROBABILITY)
                    max_iter = _emaps[sx][j]._recognition_probabilities.size();
                else
                    max_iter = _emaps[sx][j]._surface_areas.size();
                // Iterate through each view
                model_views_max_score[i][j] = -1;
                for (vector<double>::size_type k = 0; k < max_iter; ++k)
                {
                    double e;
                    if (u_type == ENTROPY)
                        e = _emaps[sx][j]._class_entropies[k];
                    if (u_type == PROBABILITY)
                        e = _emaps[sx][j]._recognition_probabilities[k];
                    else
                        e = _emaps[sx][j]._surface_areas[k];
                    // Replace the current value if the new value is larger
                    if (e > model_views_max_score[i][j])
                        model_views_max_score[i][j] = e;
                }
                if (model_views_max_score[i][j] <= 0)
                {
                    ROS_WARN("ActiveExploration::next_best_view : scaling factor for model view score is %.4f",
                             model_views_max_score[i][j]);
                    model_views_max_score[i][j] = 1;
                }
            }
        }
        // Compute the scaled utilities and also transform the views into the map frame
        vector<vector<vector<double> > > scaled_model_utilities;
        scaled_model_utilities.resize(segs_for_planning.size());
        vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > model_views;
        model_views.resize(segs_for_planning.size());
        for (vector<int>::size_type i = 0; i < segs_for_planning.size(); ++i)
        {
            // Segment index
            int sx = segs_for_planning[i];
            // View index from the instance directories
            int view_ix = atoi(_instance_directories[sx][0]._ix.c_str()); // most likely viewpoint
            // Resize the vectors
            scaled_model_utilities[i].resize(_emaps[sx].size());
            model_views[i].resize(_emaps[sx].size());
            for (vector<EntMap>::size_type j = 0; j < _emaps[sx].size(); ++j)
            {
                // Resize the model utility vector
                if (u_type == ENTROPY)
                    scaled_model_utilities[i][j].resize(_emaps[sx][j]._class_entropies.size());
                if (u_type == PROBABILITY)
                    scaled_model_utilities[i][j].resize(_emaps[sx][j]._recognition_probabilities.size());
                else
                    scaled_model_utilities[i][j].resize(_emaps[sx][j]._surface_areas.size());
                // Resize the model views vector;
                model_views[i][j].first.resize(_emaps[sx][j]._camera_poses.size());
                model_views[i][j].second.resize(_emaps[sx][j]._camera_poses.size());
                // Get the transform of the strongest matching viewpoint to the segment
                int vec_ix = 0; // vector index
                vector<int>::const_iterator it = find(_emaps[sx][j]._ixs.begin(), _emaps[sx][j]._ixs.end(), view_ix);
                if (it != _emaps[sx][j]._ixs.end())
                    vec_ix = distance<vector<int>::const_iterator>(_emaps[sx][j]._ixs.begin(), it);
                else
                    ROS_WARN("ActiveExploration::next_best_view : invalid index for transform %u", vec_ix);
                double max_scaled_val = 0;
                for (vector<double>::size_type k = 0; k < _emaps[sx][j]._surface_areas.size(); ++k)
                {
                    if (u_type == ENTROPY)
                        scaled_model_utilities[i][j][k] = 1 - (_emaps[sx][j]._class_entropies[k]/model_views_max_score[i][j]) + _EPS;
                    if (u_type == PROBABILITY)
                        scaled_model_utilities[i][j][k] = _emaps[sx][j]._recognition_probabilities[k]/model_views_max_score[i][j] + _EPS;
                    else
                        scaled_model_utilities[i][j][k] = _emaps[sx][j]._surface_areas[k]/model_views_max_score[i][j] + _EPS;
                    // Scale the utility value
                    if (scaled_model_utilities[i][j][k] > max_scaled_val)
                        max_scaled_val = scaled_model_utilities[i][j][k];
                    // Transform to model frame
                    // Transform = instance_to_map * inverse(segment_to_instance) * instance_view_to_model;
                    //Eigen::Matrix4f i_transform = _instances_to_map_tfs[sx][j] * _transforms_to_instances[sx][j].inverse() * _emaps[sx][j]._transforms[k];  // CHANGED
                    Eigen::Matrix4f i_transform = _instances_to_map_tfs[sx][j]._transform *
                                                  _emaps[sx][j]._transforms[vec_ix].inverse() *
                                                  _emaps[sx][j]._transforms[k];
                    PointCloud<PointT> tr_model_v_cloud;
                    transformPointCloud(_emaps[sx][j]._clouds[k], tr_model_v_cloud, i_transform);
                    PointCloud<PointT> pt;
                    pt.resize(1);
                    pt.points[0].x = _emaps[sx][j]._camera_poses[k][0];
                    pt.points[0].y = _emaps[sx][j]._camera_poses[k][1];
                    pt.points[0].z = _emaps[sx][j]._camera_poses[k][2];
                    transformPointCloud(pt, pt, i_transform);
                    model_views[i][j].first[k] = tr_model_v_cloud;
                    model_views[i][j].second[k] = Eigen::Vector4f(pt.points[0].x, pt.points[0].y, pt.points[0].z, 0);
                }
                // Re adjust the scaled utility values to the range (0,1)
                for (vector<double>::size_type k = 0; k < scaled_model_utilities[i][j].size(); ++k)
                    scaled_model_utilities[i][j][k] = scaled_model_utilities[i][j][k]/max_scaled_val;
            }
        }

        // Compute the utility for each location
        if (all_model_views)
            next_best_index = gaussian_weighted_next_best_view(map_locations, segs_for_planning, model_views, scaled_model_utilities,
                                                               uncertainty_weight, variance, unoccluded);
        else
            next_best_index = nearest_next_best_view(map_locations, segs_for_planning, model_views, scaled_model_utilities, uncertainty_weight);
    }
    // Check that the index is valid
    if (next_best_index < 0)
    {
        ROS_ERROR("ActiveExploration::next_best_view : invalid next best view");
        return false;
    }

    ROS_INFO("ActiveExploration::next_best_view : finished");

    return true;
}

int ActiveExploration::nearest_next_best_view(const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                                              const vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > &model_views,
                                              const vector<vector<vector<double> > > &scaled_model_utilities,
                                              const vector<double> &uncertainty_weight)
{
    // For each segment find the closest location in the map
    vector<pair<int,double> > nearest_indices;
    for (vector<int>::size_type i = 0; i < seg_indices.size(); ++i)
    {
        int sx = seg_indices[i];
        // For each class that this segment could be
        for (vector<EntMap>::size_type j = 0; j < _emaps[sx].size(); ++j)
        {
            // Get the index of the best view
            int best_ix = max_element(scaled_model_utilities[i][j].begin(), scaled_model_utilities[i][j].end())
                          - scaled_model_utilities[i][j].begin();
            // Get the utility
            double u = scaled_model_utilities[i][j][best_ix];
            // Scale the utility by the confidence of the class
            u *= _class_estimates[sx].confidence[j];
            // Scale the utility by the uncertainty of the segment
            u *= uncertainty_weight[i];
            // Get the map location
            Eigen::Vector4f max_area_location = model_views[i][j].second[best_ix];
            // Get the map location closest to this location
            int clos_ix = 0;
            double clos_dist = eigdistance3D(max_area_location, map_locations[0]);
            for (vector<Eigen::Vector4f>::size_type k = 0; k < map_locations.size(); ++k)
            {
                double d = eigdistance3D(max_area_location, map_locations[k]);
                if (d < clos_dist)
                {
                    clos_ix = k;
                    clos_dist = d;
                }
            }
            // Store in the nearest_indices vector
            if (clos_ix >= 0)
            {
                // Loop through the vector and if the index has already been entered then add this utility
                bool added_to_existing = false;
                for (vector<pair<int,double> >::size_type k = 0; k < nearest_indices.size(); ++k)
                {
                    // If this index is the same
                    if (nearest_indices[k].first == clos_ix)
                    {
                        added_to_existing = true;
                        // Get the previous utility
                        double u_prev = nearest_indices[k].second;
                        // Replace
                        nearest_indices[k] = make_pair(clos_ix, u + u_prev);
                        break;
                    }
                }
                // If the index was not found
                if (!added_to_existing)
                    nearest_indices.push_back(make_pair(clos_ix, u));
            }
        }
    }
    // If elements in the vector
    if (nearest_indices.size() > 0)
    {
        // Sort the vector of indices and utilities
        sort(nearest_indices.begin(), nearest_indices.end(), compare<double>);
        // Want the maximum utility, so return the last element
        return nearest_indices.back().first;
    }
    else
    {
        ROS_ERROR("ActiveExploration::next_best_view : did not add any locations to the nearest_indices vector");
        return -1;
    }
}

int ActiveExploration::gaussian_weighted_next_best_view(const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                                                        const vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > &model_views,
                                                        const vector<vector<vector<double> > > &scaled_model_utilities,
                                                        const vector<double> &uncertainty_weight, const double &variance, const bool &unoccluded)
{
    vector<double> utilities;
    utilities.resize(map_locations.size());
    // Do one visualization if do_visualize is set to true
    bool single_vis = false;
    // WARN : no visualization ever when commented out
    if (_visualization_on)
        single_vis = true;
    for (vector<Eigen::Vector4f>::size_type i = 0; i < map_locations.size(); ++i)
    {
        //cout << " * * * LOCATION " << i << endl;
        utilities[i] = 0;
        // Consider the contribution from each segment that needs to be viewed
        vector<vector<PointCloud<PointT> > > expected_clouds;
        expected_clouds.resize(seg_indices.size());
        vector<vector<vector<int> > > visible_indices;
        visible_indices.resize(seg_indices.size());
        for (vector<int>::size_type j = 0; j < seg_indices.size(); ++j)
        {
            int sx = seg_indices[j];
            //_entropies.push_back ((double)entropy(it->confidence));
            // Sum the values at this location for each class hypothesis
            double seg_utility = 0;
            for (vector<EntMap>::size_type k = 0; k < _emaps[sx].size(); ++k)
            {
//                Eigen::Matrix4f map_to_model_tf = _transforms_to_instances[sx][k] * _instances_to_map_tfs[sx][k].inverse();
//                // Transform the test location into the model frame
//                Eigen::Vector4f map_loc_model_frame = transform_eigvec(map_locations[i], map_to_model_tf);
//                //cout << map_loc_model_frame[0] << " " << map_loc_model_frame[1] << " " << map_loc_model_frame[2] << endl;
//                // Transform the view points into the model frame
//                vector<Eigen::Vector4f> model_views;
//                model_views.resize(_emaps[sx][k]._camera_poses.size());
//                for (vector<Eigen::Vector4f>::size_type c = 0; c < _emaps[sx][k]._camera_poses.size(); ++c)
//                    model_views[c] = transform_eigvec(_emaps[sx][k]._camera_poses[c], _emaps[sx][k]._transforms[c]);

//                // Compute the utility
                double u = active_exploration_utils::location_utility(map_locations[i], model_views[j][k].second, scaled_model_utilities[j][k], variance);

//                if (single_vis)
//                {
//                    // Get the most likely view and use that to transform the model into
//                    int view_ix = atoi(_instance_directories[sx][0]._ix.c_str()); // most likely viewpoint
//                    int vec_ix = 0; // vector index
//                    vector<int>::const_iterator it = find(_emaps[sx][k]._ixs.begin(), _emaps[sx][k]._ixs.end(), view_ix);
//                    if (it != _emaps[sx][j]._ixs.end())
//                    {
//                        vec_ix = distance<vector<int>::const_iterator>(_emaps[sx][j]._ixs.begin(), it);
//                        Eigen::Matrix4f itf = _emaps[sx][j]._transforms[vec_ix];
//                        if (!visualize_in_model_frame(map_locations[i], _emaps[sx][k], _instances_to_map_tfs[sx][k]._transform, itf))
//                            ROS_WARN("ActiveExploration::gaussian_weighted_next_best_view : could not visualize location in model frame");
//                    }
//                    else
//                    {
//                        ROS_WARN("ActiveExploration::gaussian_weighted_next_best_view : invalid index for transform %u", vec_ix);
//                    }
//                    single_vis = false;
//                }

                // Reduce the utility by the percentage of how many points will actually be visible, when viewed from this location
                if (unoccluded)
                {
                    PointCloud<PointT> exp_pc;
                    vector<int> vis_ix;
                    double percent_visible = active_exploration_utils::percentage_visible_points(_tree, map_locations[i], _instance_directories[sx][k],
                                                                                                 _emaps[sx][k], _instances_to_map_tfs[sx][k]._transform,
                                                                                                 _segment_octree_keys[sx], exp_pc, vis_ix);
                    u *= percent_visible;
                    expected_clouds[j].push_back(exp_pc);
                    visible_indices[j].push_back(vis_ix);
                }
                // Reduce the utility by the confidence of this class
                //cout << "Hypothesis utility " << u << endl;
                u *= _class_estimates[sx].confidence[k];
                //cout << "Class confidence " << _class_estimates[sx].confidence[k] << endl;
                //cout << "Scaled hypothesis utility " << u << endl;
                seg_utility += u;
            }
            // Weight the utility of the segment by its uncertainty
            // If the segment is very uncertaint then maximum weight, otherwise reduce the weight if it is well known
            //cout << "Seg utility " << seg_utility << endl;
            seg_utility *= uncertainty_weight[j];
            //cout << "Uncertainty weight " << uncertainty_weight[j] << endl;
            //cout << "Scaled seg utility " << seg_utility << endl;
            // Add to the list of utilities
            utilities[i] += seg_utility;
        }
//        if (unoccluded && _visualization_on)
//        {
//            if (!visualize_expected_clouds(map_locations[i], seg_indices, expected_clouds, visible_indices))
//                ROS_WARN("ActiveExploration::gaussian_weighted_next_best_view : could not visualize expected point clouds");
//        }
    }
    // Determine the location with the maximum utility
    return distance(utilities.begin(), max_element(utilities.begin(), utilities.end()));
}

int ActiveExploration::extracted_point_cloud_next_best_view(const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                                                            const vector<double> &uncertainty_weight, const SIM_TYPE &sim)
{
    return 0;
}
