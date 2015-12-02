#include <squirrel_object_perception_msgs/CoveragePlan.h>
#include <squirrel_object_perception_msgs/CoveragePlanFile.h>

#include "squirrel_active_exploration/octomap_utils.h"

using namespace std;
using namespace octomap;

// Default definitions, should be replaced by program arguments
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
    ros::init (argc, argv ,"test_map_coverage_server");
    ros::NodeHandle n("~");

    // Get the parameters
    string filename = "";
    int tree_depth = _TREE_DEPTH;
    double robot_height = _ROBOT_HEIGHT;
    double robot_outer_range = _ROBOT_OUTER_RANGE;
    double robot_inner_range = _ROBOT_INNER_RANGE;
    double robot_radius = _ROBOT_RADIUS;
    double step = _STEP;
    int max_iters = _MAX_ITERS;
    bool visualize = false;
    // Read the input if it exists
    if (!n.getParam ("data_filename", filename))
    {
        ROS_ERROR("TEST_map_coverage_server::main : you must enter a filename");
        return EXIT_FAILURE;
    }
    n.getParam ("tree_depth", tree_depth);
    n.getParam ("robot_height", robot_height);
    n.getParam ("robot_outer_range", robot_outer_range);
    n.getParam ("robot_inner_range", robot_inner_range);
    n.getParam ("robot_radius", robot_radius);
    n.getParam ("grid_step", step);
    n.getParam ("maximum_iterations", max_iters);
    n.getParam ("visualize", visualize);
    // Print out the input
    ROS_INFO("TEST_map_coverage_server::main : input parameters");
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

//    ros::ServiceClient mp_client = n.serviceClient<squirrel_object_perception_msgs::CoveragePlan>("/squirrel_map_coverage");
//    squirrel_object_perception_msgs::CoveragePlan mp_srv;
//    // TODO ...

    ros::ServiceClient mp_file_client = n.serviceClient<squirrel_object_perception_msgs::CoveragePlanFile>("/squirrel_map_coverage_file");
    squirrel_object_perception_msgs::CoveragePlanFile mp_file_srv;
    mp_file_srv.request.filename = filename;
    mp_file_srv.request.tree_depth = tree_depth;
    mp_file_srv.request.robot_height = robot_height;
    mp_file_srv.request.robot_outer_range = robot_outer_range;
    mp_file_srv.request.robot_inner_range = robot_inner_range;
    mp_file_srv.request.robot_radius = robot_radius;
    mp_file_srv.request.step_size = step;
    mp_file_srv.request.maximum_iterations = max_iters;
    mp_file_srv.request.visualize_on = visualize;
    // Call the service
    ROS_INFO("TEST_map_coverage_server::main : calling service /squirrel_map_coverage_file");
    if (!mp_file_client.call(mp_file_srv))
    {
        ROS_ERROR("TEST_map_coverage_server::main : could not call the entropy map service");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("TEST_map_coverage_server::main: successfully computed plan with %lu locations", mp_file_srv.response.positions.size());
        // Print out the locations
        vector<geometry_msgs::Point> positions = mp_file_srv.response.positions;
        for (vector<geometry_msgs::Point>::size_type i = 0; i < positions.size(); ++i)
            cout << i << " : " << positions[i].x << " " << positions[i].y << " " << positions[i].z << endl;
    }

    ros::shutdown();
    return EXIT_SUCCESS;
}
