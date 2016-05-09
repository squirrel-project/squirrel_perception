#include <squirrel_object_perception_msgs/CoveragePlan.h>
#include <squirrel_object_perception_msgs/ActiveExplorationNBV.h>

#include "squirrel_active_exploration/active_exploration_utils.h"
#include "squirrel_active_exploration/octomap_utils.h"

#define _DEFAULT_VARIANCE 0.5
#define _DEFAULT_ROBOT_HEIGHT 0.7
#define _DEFAULT_ROBOT_RADIUS 0.22
#define _DEFAULT_DISTANCE_FROM_CENTER 2.0
#define _DEFAULT_NUM_LOCATIONS 10
#define _TREE_SEARCH_DEPTH 14
#define _VISUALIZATION 0

using namespace std;
using namespace pcl;
using namespace octomap;

class ActiveExplorationServer
{
public:
    ActiveExplorationServer()
    {}

    ~ActiveExplorationServer()
    {
        if (_n)
            delete _n;
    }

    void initialize(int argc, char **argv)
    {
        // Initialize the node
        ros::init (argc, argv ,"squirrel_active_exploration_server");
        ros::NodeHandle *_n (new ros::NodeHandle("~"));

        // Visualization is always FALSE
        _visualization = false;

        // Subscribe to the entropy map client
        _em_client = _n->serviceClient<squirrel_object_perception_msgs::EntropyMap>("/squirrel_entropy_map");

        // Create the service
        _service = _n->advertiseService("/squirrel_active_exploration", &ActiveExplorationServer::next_best_view, this);

        ROS_INFO("squirrel_active_exploration_server : ready to receive service calls...");
    }

private:
    /* === VARIABLES === */
    ros::NodeHandle *_n;  // ros node handle
    ros::ServiceServer _service;  // offer the service to determine the next best view
    ros::ServiceClient _em_client;  // service client for entropy map
    Hypothesis _hyp;  // structure to store hypothesis information
    SIM_TYPE _plan_type;  // the method for planning the next best view
    Eigen::Vector4f _camera_pose;  // the current pose of the camera (sensor on robot)
    vector<Eigen::Vector4f> _map_locations;  // the locations in the map to evaluate
    double _variance;  // variance in the utility function
    double _robot_height;  // height of the robot (and therefore sensor)
    double _robot_radius;  // radius of the robot (for occupancy checking)
    double _distance_from_center;  // the distance from the center of an object to put a viewpoint
    int _num_locations;  // the number of locations around each object to evaluate
    bool _visualization;  // boolean variable to specify if visualization is on/off

    vector<vector<int> > _segments;  // vector of indices into the point cloud, each element is a segment
    vector<vector<OcTreeKey> > _segment_octree_keys;  // vector of octree keys into the octree for each segment
    vector<squirrel_object_perception_msgs::Classification> _class_estimates;  // vector of classification estimates for each segment
    vector<Pose> _poses;  // vector of poses for each segment
    vector<vector<InstLookUp> > _instance_directories;  // vector of directories for the matched instances in the classification
    vector<vector<InstToMapTF> > _instances_to_map_tfs;  // vector of transformations and scores from the instances to the map
    vector<vector<EntMap> > _emaps;  // vector of the entropy maps for each of the matched classes
    vector<double> _entropies;  // vector of entropies of the classification estimate for each segment
    vector<int> _entropy_ranking;  // vector of integers which rank the segment entropies from smallest to largest

    /* === FUNCTIONS === */

    /*
     * Next best view service function.
     * Determine the next best view by computing the utility value of each given location and selecting the location with
     * the highest utility.
     * If no locations are given, then generate the locations in the map as specified by the parameters in the request.
     *
     * Return true on success or false on failure.
     */
    bool next_best_view(squirrel_object_perception_msgs::ActiveExplorationNBV::Request &request,
                        squirrel_object_perception_msgs::ActiveExplorationNBV::Response &response)
    {
        // Print out the input
        ROS_INFO("squirrel_active_exploration_server : input to service");
        ROS_INFO("Pose = %.2f %.2f %.2f", request.camera_pose.position.x, request.camera_pose.position.y,
                 request.camera_pose.position.z);
        ROS_INFO("Point cloud size = %lu", request.cloud.data.size());
        ROS_INFO("Number of clusters = %lu", request.clusters_indices.size());
        ROS_INFO("Number of classification results = %lu", request.class_results.size());
        if (request.class_results.size() > 0)
            ROS_INFO("Number of candidate locations = %lu", request.locations.size());
        else
            ROS_WARN("Number of candidate locations = %lu", request.locations.size());

        // Set the camera pose
        _camera_pose[0] = request.camera_pose.position.x;
        _camera_pose[1] = request.camera_pose.position.y;
        _camera_pose[2] = request.camera_pose.position.z;
        _camera_pose[3] = 0;

        // Read the variance
        if (request.variance > 0)
            _variance = request.variance;
        else
            ROS_WARN("squirrel_active_exploration_server : variance %.2f is invalid, using default value %.2f",
                     request.variance, static_cast<double>(_DEFAULT_VARIANCE));

        // Convert the input cloud to a pcl point cloud
        PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(request.cloud, pcl_pc2);
        PointCloud<PointT> cloud;
        fromPCLPointCloud2(pcl_pc2, cloud);

        // Plan type is MIN_CLASS_ENTROPY_UNOCCLUDED or MIN_CLASS_ENTROPY
        _plan_type = MIN_CLASS_ENTROPY_UNOCCLUDED;
        if (!request.occlusions)
        {
            ROS_WARN("squirrel_active_exploration_server : occlusions set to FALSE");
            _plan_type = MIN_CLASS_ENTROPY;
        }

        // Return error if the cloud is empty
        if (cloud.size() == 0)
        {
            ROS_ERROR("squirrel_active_exploration_server : input cloud is empty");
            response.nbv_ix = -1;
            return false;
        }

        // Return error if the number of segments does not match the number of class results
        if (request.clusters_indices.size() != request.class_results.size())
        {
            ROS_ERROR("squirrel_active_exploration_server : number of clusters %lu does not match number of classification results %lu",
                      request.clusters_indices.size(), request.class_results.size());
            response.nbv_ix = -1;
            return false;
        }

        // Set the segments array
        _segments.clear();
        _segments.resize(request.clusters_indices.size());
        for (size_t i = 0; i < request.clusters_indices.size(); ++i)
            _segments[i] = request.clusters_indices[i].data;
        // If successful segmentation
        if (_segments.size() == 0)
        {
            ROS_ERROR("squirrel_active_exploration_server : cloud has zero segments");
            response.nbv_ix = -1;
            return false;
        }

        // Convert the input map ros topic to an octomap
        // Get the octree
        OcTree *tree_ptr;
        // If the header specifies it is binary
        if (request.map.binary)
        {
            // Convert the message to a map
            tree_ptr = octomap_msgs::binaryMsgToMap(request.map);
        }
        else
        {
            ROS_ERROR("squirrel_active_exploration_server : map must be in binary format");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
//             cout << "full conversion" << endl;
//             // Convert the message to a map
//             AbstractOcTree *abstract_tree_ptr = octomap_msgs::fullMsgToMap(request.map);
//             // Cast to regular octree
//             tree_ptr = dynamic_cast<OcTree*>(abstract_tree_ptr);
        }
        OcTree tree = *tree_ptr;

        // Check that the map has data
        if (_plan_type == MIN_CLASS_ENTROPY_UNOCCLUDED && (request.map.data.size() == 0 || tree.size() == 0))
        {
            ROS_WARN("squirrel_active_exploration_server : input map is empty, ignoring occlusions in the map");
            // Ignore occlusions because there is no map information
            _plan_type = MIN_CLASS_ENTROPY;
        }

        // Extract the octree keys
        _segment_octree_keys.clear();
        if (_plan_type == MIN_CLASS_ENTROPY_UNOCCLUDED)
        {
            if (!active_exploration_utils::extract_segment_octree_keys(tree, cloud, _segments, _segment_octree_keys))
            {
                ROS_ERROR("squirrel_active_exploration_server : coud not extract segment octree keys");
                response.nbv_ix = -1;
                free(tree_ptr);
                return false;
            }
        }

        // Set the class estimates and replace the double back slashes in the file paths
        _class_estimates = active_exploration_utils::fix_path_names(request.class_results);

        // Compute the poses
        if (!active_exploration_utils::estimate_pose(cloud, _segments, _poses))
        {
            ROS_ERROR("squirrel_active_exploration_server : error trying to estimate the poses of the segments");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }

        // Extract the instance directories
        // Read the best instance and extract the pose transform file for each of the object class result
        if (!active_exploration_utils::extract_instance_directories(_class_estimates, _instance_directories))
        {
            ROS_ERROR("squirrel_active_exploration_server : could not extract the instance directories");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }

        // Extract the instance to map transforms
        // Get the transformations of the instances to the maps
        if (!active_exploration_utils::transform_instances_to_map(cloud, _segments, _instance_directories, _instances_to_map_tfs))
        {
            ROS_ERROR("squirrel_active_exploration_server : could not compute the transforms to the map");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }

        // Determine the entropy maps
        if (!active_exploration_utils::retrieve_entropy_maps(_segments, _instance_directories, _em_client, _emaps))
        {
            ROS_ERROR("squirrel_active_exploration_server : could not retrive entropy maps");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }

        // Compute the entropies
        if (!active_exploration_utils::compute_entropy(_class_estimates, _entropies))
        {
            ROS_ERROR("squirrel_active_exploration_server : could not compute the entropies");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }

        // Rank the entropy values
        if (!active_exploration_utils::rank_entropy(_entropies, _entropy_ranking))
        {
            ROS_ERROR("squirrel_active_exploration_server : could not rank the entropies");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }


        // Set the structure for the next_best_view function
        _hyp._segments = _segments;
        _hyp._octree_keys = _segment_octree_keys;
        _hyp._class_estimates = _class_estimates;
        _hyp._poses = _poses;
        _hyp._instance_directories = _instance_directories;
        _hyp._transforms = _instances_to_map_tfs;
        _hyp._emaps = _emaps;
        _hyp._entropies = _entropies;
        _hyp._entropy_ranking = _entropy_ranking;

        // Convert the input locations
        _map_locations.clear();
        // If locations are specified
        if (request.locations.size() > 0)
        {
            for (size_t i = 0; i < request.locations.size(); ++i)
            {
                Eigen::Vector4f loc;
                loc[0] = request.locations[i].x;
                loc[1] = request.locations[i].y;
                loc[2] = request.locations[i].z;
                loc[3] = 0;
                _map_locations.push_back(loc);
            }
            // Set the locations in the response
            response.generated_locations = request.locations;
        }
        // Otherwise, create locations in the environment
        else
        {
            ROS_WARN("squirrel_active_exploration_server : no map locations specified, must generate them in the map");
            // Get the parameters from the message
            _robot_height = _DEFAULT_ROBOT_HEIGHT;
            if (request.robot_height > 0 && request.robot_height < 2.0)
                _robot_height = request.robot_height;
            else
                ROS_WARN("squirrel_active_exploration_server : robot height %.2f is invalid, using default value %.2f",
                         request.robot_height, static_cast<double>(_DEFAULT_ROBOT_HEIGHT));

            _robot_radius = _DEFAULT_ROBOT_RADIUS;
            if (request.robot_radius > 0 && request.robot_radius < 2.0)
                _robot_radius = request.robot_radius;
            else
                ROS_WARN("squirrel_active_exploration_server : robot radius %.2f is invalid, using default value %.2f",
                         request.robot_radius, static_cast<double>(_DEFAULT_ROBOT_RADIUS));

            _distance_from_center = _DEFAULT_DISTANCE_FROM_CENTER;
            if (request.distance_from_center > 0 && request.distance_from_center < 6.0)
                _distance_from_center = request.distance_from_center;
            else
                ROS_WARN("squirrel_active_exploration_server : distance from center %.2f is invalid, using default value %.2f",
                         request.distance_from_center, static_cast<double>(_DEFAULT_DISTANCE_FROM_CENTER));

            _num_locations = _DEFAULT_NUM_LOCATIONS;
            if (request.num_locations > 0 && request.num_locations < 30)
                _num_locations = request.num_locations;
            else
                ROS_WARN("squirrel_active_exploration_server : number of locations %i is invalid, using default value %i",
                         request.num_locations, static_cast<int>(_DEFAULT_NUM_LOCATIONS));

            if (!generate_map_locations(tree, _poses))
            {
                ROS_ERROR("squirrel_active_exploration_server : could not generate locations in the map");
                response.nbv_ix = -1;
                free(tree_ptr);
                return false;
            }
            ROS_INFO("squirrel_active_exploration_server : generated %lu locations", _map_locations.size());
            // Set the locations in the response
            response.generated_locations.clear();
            for (size_t i = 0; i < _map_locations.size(); ++i)
            {
                geometry_msgs::Point p;
                p.x = _map_locations[i][0];
                p.y = _map_locations[i][1];
                p.z = _map_locations[i][2];
                response.generated_locations.push_back(p);
            }
        }

        int nbv;
        vector<double> utilities;
        if (!active_exploration_utils::next_best_view(nbv, utilities, tree, _hyp, _plan_type, _map_locations,
                                                      _variance, _visualization))
        {
            ROS_ERROR("squirrel_active_exploration_server : could not find next best view");
            response.nbv_ix = -1;
            free(tree_ptr);
            return false;
        }

        // If successful, set the next best view index
        response.nbv_ix = nbv;
        // Set the utilities
        response.utilities.clear();
        for (size_t i = 0; i < utilities.size(); ++i)
            response.utilities.push_back(utilities[i]);

        // Free the memory
        free(tree_ptr);

        return true;
    }

    /*
     * Generate locations in a map.
     * Determine a set of valid observation locations by generating locations around each identified segment.
     * Segment centers are specified by the poses vector.
     * Locations are removed if they are occupied as determined by the octree. If this is an empty tree, then
     * occupancy checking does not occur.
     * The locations are returned in the vector map_locations.
     *
     * Return true on success or false on failure.
     */
    bool generate_map_locations(const OcTree &tree, const vector<Pose> &poses)
    {
        // Generate locations around each segment centre at _distance_from_center from the centre
        vector<vector<Eigen::Vector4f> > circle_poses;
        for (size_t i = 0; i < poses.size(); ++i)
        {
            // Get the surrounding locations (they will have the same z value as the original pose)
            vector<Eigen::Vector4f> p = poses[i].get_surrounding_locations(_distance_from_center,
                                                                           _num_locations,
                                                                           _camera_pose[2]);
            // Add to the set of locations
            circle_poses.push_back(p);
        }

//        // Write locations to a file
//        string fname = "/home/tpat8946/locations.txt";
//        ofstream myfile (fname.c_str());
//        if (myfile.is_open())
//        {
//            // Write segment centroids
//            for (size_t i = 0; i < poses.size(); ++i)
//                myfile << poses[i].get_centroid()[0] << " " << poses[i].get_centroid()[1] << " " << poses[i].get_centroid()[2] << " ";
//            myfile << "\n";
//            // Write original locations
//            for (size_t i = 0; i < circle_poses.size(); ++i)
//            {
//                for (size_t j = 0; j < circle_poses[i].size(); ++j)
//                    myfile << circle_poses[i][j][0] << " " << circle_poses[i][j][1] << " " << circle_poses[i][j][2] << " ";
//            }
//            myfile << "\n";
//            myfile.close();
//        }

        // Remove locations that are too near object centers
        if (tree.size() > 0)
        {
            if (!remove_occupied_locations(tree, _robot_height, _robot_radius, circle_poses))
            {
                ROS_WARN("squirrel_active_exploration_server : could not remove occupied locations");
                return false;
            }
        }

//        myfile.open(fname.c_str(), ios::app);
//        if (myfile.is_open())
//        {
//            // Write ground removed locations
//            for (size_t i = 0; i < circle_poses.size(); ++i)
//            {
//                for (size_t j = 0; j < circle_poses[i].size(); ++j)
//                    myfile << circle_poses[i][j][0] << " " << circle_poses[i][j][1] << " " << circle_poses[i][j][2] << " ";
//            }
//            myfile << "\n";
//            myfile.close();
//        }

        // Merge locations near each other (greedily)
        double min_dist = 2.0 * M_PI * _distance_from_center / static_cast<double>(_num_locations);  // circumference divided by number of intervals
        // Loop through the sets of locations and merge locations that are less than the min dist
        vector<Eigen::Vector4f> merged_locs;
        for (size_t i = 0; i < circle_poses.size(); ++i)
        {
            // If this is the first set, then always add
            if (i == 0)
            {
                merged_locs.insert(merged_locs.end(), circle_poses[i].begin(), circle_poses[i].end());
            }
            // Otherwise iterate through the already added poses and merrge if necessary
            else
            {
                // For each location around the object
                for (size_t j = 0; j < circle_poses[i].size(); ++j)
                {
                    int merge_ix = -1;
                    // For each location already added to the list
                    for (size_t k = 0; k < merged_locs.size(); ++k)
                    {
                        // If the distance is less than min_dist
                        if (eigdistance3D(circle_poses[i][j], merged_locs[k]) < min_dist)
                        {
                            merge_ix = k;
                            break;
                        }
                    }
                    // If the location needs to be merged
                    if (merge_ix >= 0 && merge_ix < merged_locs.size())
                    {
                        Eigen::Vector4f p;
                        p[0] = (circle_poses[i][j][0] + merged_locs[merge_ix][0]) / 2;
                        p[1] = (circle_poses[i][j][1] + merged_locs[merge_ix][1]) / 2;
                        p[2] = circle_poses[i][j][2];
                        p[3] = 0;
                        merged_locs[merge_ix] = p;
                    }
                    // Otherwise add to the list
                    else
                    {
                        merged_locs.push_back(circle_poses[i][j]);
                    }
                }
            }
        }

//        myfile.open(fname.c_str(), ios::app);
//        if (myfile.is_open())
//        {
//            // Write merged locations
//            for (size_t i = 0; i < merged_locs.size(); ++i)
//                myfile << merged_locs[i][0] << " " << merged_locs[i][1] << " " << merged_locs[i][2] << " ";
//            myfile << "\n";
//            myfile.close();
//        }

        // Now pass through all the locations and remove any locations that are too near to each other
        _map_locations.clear();
        for (size_t i = 0; i < merged_locs.size(); ++i)
        {
            // If have not added to the list yet
            if (_map_locations.size() == 0)
            {
                _map_locations.push_back(merged_locs[i]);
            }
            // Otherwise, check this new location should be added or not
            else
            {
                bool add_to_list = true;
                for (size_t j = 0; j < _map_locations.size(); ++j)
                {
                    if (eigdistance3D(merged_locs[i], _map_locations[j]) < min_dist)
                    {
                        add_to_list = false;
                        break;
                    }
                }
                // If passed all checks
                if (add_to_list)
                    _map_locations.push_back(merged_locs[i]);
            }
        }

//        myfile.open(fname.c_str(), ios::app);
//        if (myfile.is_open())
//        {
//            // Write merged locations
//            for (size_t i = 0; i < _map_locations.size(); ++i)
//                myfile << _map_locations[i][0] << " " << _map_locations[i][1] << " " << _map_locations[i][2] << " ";
//            myfile << "\n";
//            myfile.close();
//        }

        return true;
    }

    /*
     * Remove locations from a list that are occupied in an octree.
     * Compare each location to the occupied cells in an octree. If the cell is occupied then remove it from
     * the list of locations.
     * The locations are returned by the locations vector.
     *
     * Return true on success or false on failure.
     */
    bool remove_occupied_locations(const OcTree &tree, const double &robot_height, const double robot_radius,
                                   vector<vector<Eigen::Vector4f> > &locations)
    {
        // Construct a point cloud representation of the tree
        PointCloud<PointT> occupied_cloud = octree_to_cloud(tree);
        // Determine the height of the ground and the minimum z values from the tree
        double zground = octree_ground_height(tree, _TREE_SEARCH_DEPTH);
        double zmin = zground  + _GROUND_THRESH;
        //double zmin = zground + _robot_height/2;
        double zmax = zground + robot_height;
        double zheight = zmin + tree.getResolution();

        // Get the points in the point cloud that are within the z bounds
        PointCloud<PointT> inrange_cloud;
        inrange_cloud.resize(occupied_cloud.size());
        int n_size = 0;
        for (size_t i = 0; i < occupied_cloud.size(); ++i)
        {
            if (occupied_cloud.points[i].z >= zmin && occupied_cloud.points[i].z <= zmax)
            {
                inrange_cloud.points[n_size].x = occupied_cloud.points[i].x;
                inrange_cloud.points[n_size].y = occupied_cloud.points[i].y;
                inrange_cloud.points[n_size].z = occupied_cloud.points[i].z;

                ++n_size;
            }
        }
        inrange_cloud.resize(n_size);
        PointT minr, maxr;
        getMinMax3D (inrange_cloud, minr, maxr);

        // Iterate through the locations and only retain the locations that are not occupied
        vector<vector<Eigen::Vector4f> > keep_locations;
        double z = zmax;
        bool check_floor = false;  // don't check the floor, just remove locations too near the robot
        for (size_t i = 0; i < locations.size(); ++i)
        {
            vector<Eigen::Vector4f> kp;
            for (size_t j = 0; j < locations[i].size(); ++j)
            {
                point3d p (locations[i][j][0], locations[i][j][1], z);  // z or locations[i][2] ??
                // Check if valid on ground
                bool valid;
                valid_on_ground(tree, robot_radius, zmin, zmax, zheight, p, valid, check_floor);
                if (valid)
                {
                    kp.push_back(locations[i][j]);
                }
            }
            // If kp has elements
            if (kp.size() > 0)
                keep_locations.push_back(kp);
        }

        // Set the locations to the locations that were valid
        locations = keep_locations;

        // Return success
        return true;
    }
};


/*
 * Main function
 */
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING SQUIRREL_ACTIVE_EXPLORATION_SERVER ***");

    ActiveExplorationServer server;
    server.initialize(argc, argv);
    ros::spin();

    ROS_INFO("*** FINISH SQUIRREL_ACTIVE_EXPLORATION_SERVER ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}
