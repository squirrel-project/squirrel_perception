#include <squirrel_active_exploration/CoveragePlan.h>
#include <squirrel_active_exploration/CoveragePlanFile.h>

#include "squirrel_active_exploration/octomap_utils.h"

using namespace std;
using namespace octomap;

class CoverageServer
{
public:
    CoverageServer()
    {}

    ~CoverageServer()
    {
        if (_n)
            delete _n;
    }

    void initialize(int argc, char **argv)
    {
        // Initialize the node
        ros::init (argc, argv ,"squirrel_map_coverage_server");
        ros::NodeHandle *_n (new ros::NodeHandle("~"));

        // Create the services
        _service_coverage_plan = _n->advertiseService("/squirrel_map_coverage", &CoverageServer::extract_coverage_locations, this);
        _service_coverage_plan_file = _n->advertiseService("/squirrel_map_coverage_file", &CoverageServer::extract_coverage_locations_from_file, this);

        ROS_INFO("squirrel_map_coverage_server : ready to receive service calls...");
    }

private:
    /* === VARIABLES === */
    ros::NodeHandle *_n;  // ros node handle
    ros::ServiceServer _service_coverage_plan;       // offer the service to extract the coverage plan
    ros::ServiceServer _service_coverage_plan_file;  // offer the service to extract the coverage plan from a file

    /* === FUNCTIONS === */

    bool extract_coverage_locations(squirrel_active_exploration::CoveragePlan::Request &req,
                                    squirrel_active_exploration::CoveragePlan::Response &response)
    {
        // Print out the parameters
        ROS_INFO("squirrel_map_coverage_server : input parameters");
        ROS_INFO("Tree depth = %u", req.tree_depth);
        ROS_INFO("Robot height = %.2f", req.robot_height);
        ROS_INFO("Robot outer range = %.2f", req.robot_outer_range);
        ROS_INFO("Robot inner range = %.2f", req.robot_inner_range);
        ROS_INFO("Robot radius = %.2f", req.robot_radius);
        ROS_INFO("Step = %.2f", req.step_size);
        ROS_INFO("Maximum iterations = %u", req.maximum_iterations);
        if (req.visualize_on)
            ROS_WARN("squirrel_map_coverage_server : visualization = ON");
        else
            ROS_WARN("squirrel_map_coverage_server : visualization = OFF");

        // Get the octree
        OcTree *tree_ptr;
        // If the header specifies it is binary
        if (req.map.binary)
        {
            // Convert the message to a map
            tree_ptr = octomap_msgs::binaryMsgToMap(req.map);
        }
        else
        {
             // Convert the message to a map
             AbstractOcTree *abstract_tree_ptr = octomap_msgs::fullMsgToMap(req.map);
             // Cast to regular octree
             tree_ptr = dynamic_cast<OcTree*>(abstract_tree_ptr);
        }
        OcTree tree = *tree_ptr;

        // Set the robot parameters
        robot_parameters robot;
        robot.height = req.robot_height;
        robot.outer_range = req.robot_outer_range;
        robot.inner_range = req.robot_inner_range;
        robot.radius = req.robot_radius;

        if (req.tree_depth <= 0 || req.tree_depth > tree.getTreeDepth())
        {
            ROS_ERROR("squirrel_map_coverage_server : input depth of %u is invalid", req.tree_depth);
            return false;
        }

        // No save directory
        string dir = "";

        // Get the locations
        vector<point3d> locations = get_coverage_locations(tree, dir, robot, req.step_size,
                                                           req.maximum_iterations, (unsigned int)req.tree_depth);
        // If visualization is on
        if (req.visualize_on)
            visualize_coverage(tree, robot, req.step_size, locations, (unsigned int)req.tree_depth);

        // Now return the response
        vector<geometry_msgs::Point> return_locations;
        for (vector<point3d>::const_iterator it = locations.begin(); it != locations.end(); ++it)
        {
            geometry_msgs::Point p;
            p.x = it->x();
            p.y = it->y();
            p.z = it->z();
            return_locations.push_back(p);

        }
        response.positions = return_locations;

        if (tree_ptr)
            delete tree_ptr;

        return true;
    }

    bool extract_coverage_locations_from_file(squirrel_active_exploration::CoveragePlanFile::Request &req,
                                              squirrel_active_exploration::CoveragePlanFile::Response &response)
    {
        // Print out the parameters
        ROS_INFO("squirrel_map_coverage_server : input parameters");
        ROS_INFO("Filename = %s", req.filename.c_str());
        ROS_INFO("Tree depth = %u", req.tree_depth);
        ROS_INFO("Robot height = %.2f", req.robot_height);
        ROS_INFO("Robot outer range = %.2f", req.robot_outer_range);
        ROS_INFO("Robot inner range = %.2f", req.robot_inner_range);
        ROS_INFO("Robot radius = %.2f", req.robot_radius);
        ROS_INFO("Step = %.2f", req.step_size);
        ROS_INFO("Maximum iterations = %u", req.maximum_iterations);
        if (req.visualize_on)
            ROS_WARN("squirrel_map_coverage_server : visualization = ON");
        else
            ROS_WARN("squirrel_map_coverage_server : visualization = OFF");

        // Get the octree
        bool is_directory;
        OcTree tree = get_tree_from_filename(req.filename, is_directory);
        ROS_INFO("squirrel_map_coverage_server : tree has size %lu", (long int)tree.size());
        if (tree.size() == 0)
        {
            ROS_ERROR("squirrel_map_coverage_server : could not find a valid tree file from %s", req.filename.c_str());
            return false;
        }

        // Set the robot parameters
        robot_parameters robot;
        robot.height = req.robot_height;
        robot.outer_range = req.robot_outer_range;
        robot.inner_range = req.robot_inner_range;
        robot.radius = req.robot_radius;

        if (req.tree_depth <= 0 || req.tree_depth > tree.getTreeDepth())
        {
            ROS_ERROR("squirrel_map_coverage_server : input depth of %u is invalid", req.tree_depth);
            return false;
        }

        // If the filename is a directory then can lookup/save the results
        string dir = "";
        if (is_directory)
            dir = req.filename;

        // Get the locations
        vector<point3d> locations = get_coverage_locations(tree, dir, robot, req.step_size,
                                                           req.maximum_iterations, (unsigned int)req.tree_depth);
        // If visualization is on
        if (req.visualize_on)
            visualize_coverage(tree, robot, req.step_size, locations, (unsigned int)req.tree_depth);

        // Now return the response
        vector<geometry_msgs::Point> return_locations;
        for (vector<point3d>::const_iterator it = locations.begin(); it != locations.end(); ++it)
        {
            geometry_msgs::Point p;
            p.x = it->x();
            p.y = it->y();
            p.z = it->z();
            return_locations.push_back(p);

        }
        response.positions = return_locations;

        return true;
    }
};


// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING SQUIRREL_MAP_COVERAGE_SERVER ***");

    CoverageServer server;
    server.initialize(argc, argv);
    ros::spin();

    ROS_INFO("*** FINISH SQUIRREL_MAP_COVERAGE_SERVER ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}
