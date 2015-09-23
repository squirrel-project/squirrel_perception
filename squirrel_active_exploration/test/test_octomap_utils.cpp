#include "squirrel_active_exploration/octomap_utils.h"

using namespace std;
using namespace octomap;

// Default definitions, should be replaced by program arguments
#define _FILENAME "/home/tpat8946/Data/TUW/maps/fr_078_tidyup/"
#define _TREE_DEPTH 14  // 14 => resolution of 0.1m, initial tree depth is 16 => 0.025m
#define _ROBOT_HEIGHT 0.75
#define _ROBOT_OUTER_RANGE 2.0
#define _ROBOT_INNER_RANGE 1.0
#define _ROBOT_RADIUS 0.22
#define _STEP 0.5
#define _MAX_ITERS 20

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    ros::init (argc, argv ,"test_octomap");
    ros::NodeHandle n("~");

    // Get the parameters
    string filename = _FILENAME;
    //string dir = _FILENAME;
    unsigned int tree_depth = _TREE_DEPTH;
    double robot_height = _ROBOT_HEIGHT;
    double robot_outer_range = _ROBOT_OUTER_RANGE;
    double robot_inner_range = _ROBOT_INNER_RANGE;
    double robot_radius = _ROBOT_RADIUS;
    double step = _STEP;
    int max_iters = _MAX_ITERS;
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

    // Get the octree
    bool is_directory;
    OcTree tree = get_tree_from_filename(filename, is_directory);
    if (tree.size() == 0)
    {
        ROS_ERROR("TEST_octomap_utils : could not find a tree file in the directory %s", filename.c_str());
        return EXIT_FAILURE;
    }
    // Set the robot parameters
    robot_parameters robot;
    robot.height = robot_height;
    robot.outer_range = robot_outer_range;
    robot.inner_range = robot_inner_range;
    robot.radius = robot_radius;

    string save_dir = "";
    if (is_directory)
        save_dir = filename;

    // Run the visualizer
    //octree_visualize(tree, robot, tree_depth);
    //octree_visualize_grid(tree, robot, step, tree_depth);
    //octree_visualize_location(tree, robot, tree_depth);
    octree_visualize_minimal_overlap(tree, save_dir, robot, step, max_iters, tree_depth);

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}
