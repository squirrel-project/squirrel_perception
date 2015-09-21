#include "squirrel_active_exploration/octomap_utils.h"

using namespace std;
using namespace octomap;

// Default definitions, should be replaced by program arguments
#define _FILENAME "/home/tpat8946/Data/TUW/maps/fr_078_tidyup/"
#define _TREE_DEPTH 14
#define _ROBOT_HEIGHT 0.7
#define _ROBOT_OUTER_RANGE 2.0
#define _ROBOT_INNER_RANGE 1.0
#define _ROBOT_RADIUS 0.5
#define _STEP 0.5
#define _MAX_ITERS 25

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING SQUIRREL_MAP_COVERAGE ***");

    ros::init (argc, argv ,"squirrel_map_coverage");
    ros::NodeHandle n("~");

    // Get the parameters
    string filename = _FILENAME;
    int tree_depth = _TREE_DEPTH;
    double robot_height = _ROBOT_HEIGHT;
    double robot_outer_range = _ROBOT_OUTER_RANGE;
    double robot_inner_range = _ROBOT_INNER_RANGE;
    double robot_radius = _ROBOT_RADIUS;
    double step = _STEP;
    int max_iters = _MAX_ITERS;
    bool visualize = false;
    // Read the input if it exists
    n.getParam ("data_filename", filename);
    n.getParam ("tree_depth", tree_depth);
    n.getParam ("robot_height", robot_height);
    n.getParam ("robot_outer_range", robot_outer_range);
    n.getParam ("robot_inner_range", robot_inner_range);
    n.getParam ("robot_radius", robot_radius);
    n.getParam ("grid_step", step);
    n.getParam ("maximum_iterations", max_iters);
    n.getParam ("visualize", visualize);
    // Print out the input
    ROS_INFO("TEST_octomap_utils::main : input parameters");
    ROS_INFO("Filename = %s", filename.c_str());
    ROS_INFO("Tree depth = %u", tree_depth);
    ROS_INFO("Robot height = %.2f", robot_height);
    ROS_INFO("Robot outer range = %.2f", robot_outer_range);
    ROS_INFO("Robot inner range = %.2f", robot_inner_range);
    ROS_INFO("Robot radius = %.2f", robot_radius);
    ROS_INFO("Step = %.2f", step);
    ROS_INFO("Maximum iterations = %u", max_iters);
    if (visualize)
        ROS_INFO("Visualization = ON");
    else
        ROS_INFO("Visualization = OFF");

    // Get the octree
    bool is_directory;
    OcTree tree = get_tree_from_filename(filename, is_directory);
    ROS_INFO("squirrel_map_coverage::main : tree has size %lu", (long int)tree.size());
    if (tree.size() == 0)
    {
        ROS_ERROR("squirrel_map_coverage::main : could not find a valid tree file from %s", filename.c_str());
        return EXIT_FAILURE;
    }

    // Set the robot parameters
    robot_parameters robot;
    robot.height = robot_height;
    robot.outer_range = robot_outer_range;
    robot.inner_range = robot_inner_range;
    robot.radius = robot_radius;

    if (tree_depth <= 0 || tree_depth > tree.getTreeDepth())
    {
        ROS_ERROR("squirrel_map_coverage::main : input depth of %u is invalid", tree_depth);
        return EXIT_FAILURE;
    }

    // If the filename is a directory then can lookup/save the results
    string dir = "";
    if (is_directory)
        dir = filename;

    // Get the locations
    vector<point3d> locations = get_coverage_locations(tree, dir, robot, step, max_iters, (unsigned int)tree_depth);

    // If visualization is on
    if (visualize)
        visualize_coverage(tree, robot, step, locations, (unsigned int)tree_depth);

    ROS_INFO("*** FINISH SQUIRREL_MAP_COVERAGE ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}
