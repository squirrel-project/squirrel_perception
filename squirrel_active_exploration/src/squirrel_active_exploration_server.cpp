#include <squirrel_object_perception_msgs/CoveragePlan.h>
#include <squirrel_object_perception_msgs/ActiveExplorationNBV.h>

#include "squirrel_active_exploration/active_exploration_utils.h"
#include "squirrel_active_exploration/octomap_utils.h"

#define _DEFAULT_VARIANCE 0.5

using namespace std;
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

        // Read the parameters
        if (!_n->getParam("variance", _variance))
        {
            _variance = _DEFAULT_VARIANCE;
            ROS_WARN("squirrel_active_exploration_server : no variance specified, using default value %.2f",
                     static_cast<double>(_DEFAULT_VARIANCE));
        }
        // Plan type is always MIN_CLASS_ENTROPY
        _plan_type = MIN_CLASS_ENTROPY;

        // Create the services
        _service = _n->advertiseService("/squirrel_active_exploration", &ActiveExplorationServer::next_best_view, this);

        ROS_INFO("squirrel_active_exploration_server : ready to receive service calls...");
    }

private:
    /* === VARIABLES === */
    ros::NodeHandle *_n;  // ros node handle
    ros::ServiceServer _service;  // offer the service to determine the next best view
    Hypothesis _hyp;  // structure to store hypothesis information
    SIM_TYPE _plan_type;  // the method for planning the next best view
    double _variance;  // variance in the utility function

    /* === FUNCTIONS === */

    bool next_best_view(squirrel_object_perception_msgs::ActiveExplorationNBV::Request &req,
                        squirrel_object_perception_msgs::ActiveExplorationNBV::Response &response)
    {
        // Print out the input
        ROS_INFO("squirrel_active_exploration_server : input to service");
        ROS_INFO("Point cloud size = %lu", req.cloud.data.size());
        ROS_INFO("Number of clusters = %lu", req.clusters_indices.size());
        ROS_INFO("Number of classification results = %lu", req.class_results.size());
        if (req.class_results.size() > 0)
            ROS_INFO("Number of candidate locations = %lu", req.locations.size());
        else
            ROS_WARN("Number of candidate locations = %lu", req.locations.size());

        // Return error if the number of segments does not match the number of class results
        if (req.clusters_indices.size() != req.class_results.size())
        {
            ROS_ERROR("squirrel_active_exploration_server : number of clusters %lu does not match number of classification results %lu",
                      req.clusters_indices.size(), req.class_results.size());
            response.nbv_ix = -1;
            return false;
        }

        // Set the input for the next_best_view function


//        _hyp._segments = _segments;
//        _hyp._octree_keys = _segment_octree_keys;
//        _hyp._class_estimates = _class_estimates;
//        _hyp._poses = _poses;
//        _hyp._instance_directories = _instance_directories;
//        _hyp._transforms = _instances_to_map_tfs;
//        _hyp._emaps = _emaps;
//        _hyp._entropies = _entropies;
//        _hyp._entropy_ranking = _entropy_ranking;

//          // next_best_view(int &next_best_index, const OcTree &tree, const Hypothesis &hypothesis, const SIM_TYPE &sim,
//          //const vector<Eigen::Vector4f> &map_locations, const double &variance, const bool &do_visualize)
//        if (!active_exploration_utils::next_best_view(next_best_index, _tree, hyp, sim, map_locations, variance, _visualization_on))
//        {
//            ROS_ERROR("ActiveExploration::plan : Could not find next best view");
//            return false;
//        }





//        // Get the octree
//        OcTree *tree_ptr;
//        // If the header specifies it is binary
//        if (req.map.binary)
//        {
//            // Convert the message to a map
//            tree_ptr = octomap_msgs::binaryMsgToMap(req.map);
//        }
//        else
//        {
//             // Convert the message to a map
//             AbstractOcTree *abstract_tree_ptr = octomap_msgs::fullMsgToMap(req.map);
//             // Cast to regular octree
//             tree_ptr = dynamic_cast<OcTree*>(abstract_tree_ptr);
//        }
//        OcTree tree = *tree_ptr;

//        // Set the robot parameters
//        robot_parameters robot;
//        robot.height = req.robot_height;
//        robot.outer_range = req.robot_outer_range;
//        robot.inner_range = req.robot_inner_range;
//        robot.radius = req.robot_radius;

//        if (req.tree_depth <= 0 || req.tree_depth > tree.getTreeDepth())
//        {
//            ROS_ERROR("squirrel_active_exploration_server : input depth of %u is invalid", req.tree_depth);
//            return false;
//        }

//        // No save directory
//        string dir = "";

//        // Get the locations
//        vector<point3d> locations = get_coverage_locations(tree, dir, robot, req.step_size,
//                                                           req.maximum_iterations, (unsigned int)req.tree_depth);
//        // If visualization is on
//        if (req.visualize_on)
//            visualize_coverage(tree, robot, req.step_size, locations, (unsigned int)req.tree_depth);

//        // Now return the response
//        vector<geometry_msgs::Point> return_locations;
//        for (vector<point3d>::const_iterator it = locations.begin(); it != locations.end(); ++it)
//        {
//            geometry_msgs::Point p;
//            p.x = it->x();
//            p.y = it->y();
//            p.z = it->z();
//            return_locations.push_back(p);

//        }
//        response.positions = return_locations;

//        if (tree_ptr)
//            delete tree_ptr;

        return true;
    }

//    bool extract_exploration_locations_from_file(squirrel_object_perception_msgs::CoveragePlanFile::Request &req,
//                                                 squirrel_object_perception_msgs::CoveragePlanFile::Response &response)
//    {
//        // Print out the parameters
//        ROS_INFO("squirrel_active_exploration_server : input parameters");
//        ROS_INFO("Filename = %s", req.filename.c_str());
//        ROS_INFO("Tree depth = %u", req.tree_depth);
//        ROS_INFO("Robot height = %.2f", req.robot_height);
//        ROS_INFO("Robot outer range = %.2f", req.robot_outer_range);
//        ROS_INFO("Robot inner range = %.2f", req.robot_inner_range);
//        ROS_INFO("Robot radius = %.2f", req.robot_radius);
//        ROS_INFO("Step = %.2f", req.step_size);
//        ROS_INFO("Maximum iterations = %u", req.maximum_iterations);
//        if (req.visualize_on)
//            ROS_WARN("squirrel_active_exploration_server : visualization = ON");
//        else
//            ROS_WARN("squirrel_active_exploration_server : visualization = OFF");

//        // Get the octree
//        bool is_directory;
//        OcTree tree = get_tree_from_filename(req.filename, is_directory);
//        ROS_INFO("squirrel_active_exploration_server : tree has size %lu", (long int)tree.size());
//        if (tree.size() == 0)
//        {
//            ROS_ERROR("squirrel_active_exploration_server : could not find a valid tree file from %s", req.filename.c_str());
//            return false;
//        }

//        // Set the robot parameters
//        robot_parameters robot;
//        robot.height = req.robot_height;
//        robot.outer_range = req.robot_outer_range;
//        robot.inner_range = req.robot_inner_range;
//        robot.radius = req.robot_radius;

//        if (req.tree_depth <= 0 || req.tree_depth > tree.getTreeDepth())
//        {
//            ROS_ERROR("squirrel_active_exploration_server : input depth of %u is invalid", req.tree_depth);
//            return false;
//        }

//        // If the filename is a directory then can lookup/save the results
//        string dir = "";
//        if (is_directory)
//            dir = req.filename;

//        // Get the locations
//        vector<point3d> locations = get_coverage_locations(tree, dir, robot, req.step_size,
//                                                           req.maximum_iterations, (unsigned int)req.tree_depth);
//        // If visualization is on
//        if (req.visualize_on)
//            visualize_coverage(tree, robot, req.step_size, locations, (unsigned int)req.tree_depth);

//        // Now return the response
//        vector<geometry_msgs::Point> return_locations;
//        for (vector<point3d>::const_iterator it = locations.begin(); it != locations.end(); ++it)
//        {
//            geometry_msgs::Point p;
//            p.x = it->x();
//            p.y = it->y();
//            p.z = it->z();
//            return_locations.push_back(p);

//        }
//        response.positions = return_locations;

//        return true;
//    }
};


// Run Main
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
