#include "squirrel_active_exploration/active_exploration.h"
#include "squirrel_active_exploration/robot_controller.h"
#include <sensor_msgs/PointCloud2.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace pcl;

#define _LOCATIONS_ORDER_FILENAME "experiment_locations.txt"
#define _MAP_LOCATIONS_FILE "/home/tpat8946/Data/TUW/TUW_GH30_dataset/GH30_set_003_locations.txt"
#define _STORE_POINTS_DIR "/home/tpat8946/Data/"
#define _VARIANCE 0.5
#define _PLAN_TYPE "max_area"
#define _EXPECTED_NUMBER_OBJECTS -1
#define _EXPECTED_NUMBER_CLASSES -1
#define _SCENE_CENTER 0
#define _KINECT_HEIGHT 0.625
#define _VISUALIZE_FLAG 0
#define _SAVE_FLAG 0
#define _CLOUD_PREFIX "cloud_"
#define _TRANSFORMATION_PREFIX "transformation_"
#define _VERIFICATION_PREFIX "verification_cloud_"

bool parse_input(const ros::NodeHandle &n, string &map_locations_file, string &stored_points_dir, double &variance, SIM_TYPE &sim,
                 int &expected_number_objects, int &expected_number_classes, double &scene_center_x, double &scene_center_y, double &kinect_height,
                 bool &do_visualize, bool &do_saving)
{
    // Default values
    map_locations_file = _MAP_LOCATIONS_FILE;
    stored_points_dir = _STORE_POINTS_DIR;
    variance = _VARIANCE;
    expected_number_objects = _EXPECTED_NUMBER_OBJECTS;
    expected_number_classes = _EXPECTED_NUMBER_CLASSES;
    scene_center_x = _SCENE_CENTER;
    scene_center_y = _SCENE_CENTER;
    kinect_height = _KINECT_HEIGHT;
    do_visualize = _VISUALIZE_FLAG;
    do_saving = _SAVE_FLAG;
    // Read the arguments
    n.getParam("map_locations_file", map_locations_file);
    n.getParam("stored_points_directory", stored_points_dir);
    n.getParam("variance", variance);
    n.getParam("expected_number_objects", expected_number_objects);
    n.getParam("expected_number_classes", expected_number_classes);
    n.getParam("scene_center_x", scene_center_x);
    n.getParam("scene_center_y", scene_center_y);
    n.getParam("kinect_height", kinect_height);
    n.getParam("visualize", do_visualize);
    n.getParam("save", do_saving);
    // Get plan type and convert to correct format
    string plan_type = _PLAN_TYPE;
    n.getParam("plan_type", plan_type);
    sim = MAX_AREA;
    if (boost::iequals(plan_type,"nearest_location_area") || boost::iequals(plan_type,"nearest_area"))
        sim = NEAREST_AREA;
    else if (boost::iequals(plan_type,"nearest_location_min_entropy") || boost::iequals(plan_type,"nearest_min_entropy") ||
             boost::iequals(plan_type,"nearest_location_entropy") || boost::iequals(plan_type,"nearest_entropy"))
        sim = NEAREST_MIN_ENTROPY;
    else if (boost::iequals(plan_type,"nearest_location_max_probability") || boost::iequals(plan_type,"nearest_max_probability") ||
             boost::iequals(plan_type,"nearest_location_probability") || boost::iequals(plan_type,"nearest_probability") ||
             boost::iequals(plan_type,"nearest_location_max_prob") || boost::iequals(plan_type,"nearest_max_prob") ||
             boost::iequals(plan_type,"nearest_location_prob") || boost::iequals(plan_type,"nearest_prob") )
        sim = NEAREST_MAX_CLASS_PROB;
    else if (boost::iequals(plan_type, "random") || boost::iequals(plan_type,"rand"))
        sim = RANDOM;
    else if (boost::iequals(plan_type,"max_area") || boost::iequals(plan_type,"area"))
        sim = MAX_AREA;
    else if (boost::iequals(plan_type,"max_area_unoccluded") || boost::iequals(plan_type,"area_unoccluded"))
        sim = MAX_AREA_UNOCCLUDED;
    else if (boost::iequals(plan_type,"min_class_entropy") || boost::iequals(plan_type,"min_class") || boost::iequals(plan_type,"entropy"))
        sim = MIN_CLASS_ENTROPY;
    else if (boost::iequals(plan_type,"min_class_entropy_unoccluded") || boost::iequals(plan_type,"min_class_unoccluded") ||
             boost::iequals(plan_type,"min_entropy_unoccluded") || boost::iequals(plan_type,"entropy_unoccluded"))
        sim = MIN_CLASS_ENTROPY_UNOCCLUDED;
    else if (boost::iequals(plan_type,"max_class_probability") || boost::iequals(plan_type,"max_class") ||
             boost::iequals(plan_type,"probability") || boost::iequals(plan_type,"max_class_prob") || boost::iequals(plan_type,"prob"))
        sim = MAX_CLASS_PROB;
    else if (boost::iequals(plan_type,"max_class_probability_unoccluded") || boost::iequals(plan_type,"max_class_unoccluded") ||
             boost::iequals(plan_type,"max_probability_unoccluded") || boost::iequals(plan_type,"probability_unoccluded") ||
             boost::iequals(plan_type,"max_class_prob_unoccluded") || boost::iequals(plan_type,"max_prob_unoccluded") ||
             boost::iequals(plan_type,"prob_unoccluded"))
        sim = MAX_CLASS_PROB_UNOCCLUDED;
    else if (boost::iequals(plan_type,"min_view_classification_entropy"))
        sim = MIN_VIEW_CLASSIFICATION_ENTROPY;
    else if (boost::iequals(plan_type,"max_view_classification_probability") || boost::iequals(plan_type,"max_view_classification_prob"))
        sim = MAX_VIEW_CLASSIFICATION_PROB;
    else
    {
        ROS_ERROR("squirrel_run_planner::parse_input : could not interpret plan type input %s", plan_type.c_str());
        return false;
    }
    // Print the input to console
    ROS_INFO(" -- PARAMETERS --");
    ROS_INFO("  Map locations file = %s", map_locations_file.c_str());
    ROS_INFO("  Stored points directory = %s", stored_points_dir.c_str());
    ROS_INFO("  Variance = %.4f", variance);
    ROS_INFO("  Simulation type = %s", plan_type.c_str());
    ROS_INFO("  Expected number objects = %i", expected_number_objects);
    ROS_INFO("  Expected number classes = %i", expected_number_classes);
    ROS_INFO("  Scene center = %.4f , %.4f", scene_center_x, scene_center_y);
    ROS_INFO("  Kinect height = %.4f", kinect_height);
    if (do_visualize)
        ROS_INFO("  Visualization = ON");
    else
        ROS_INFO("  Visualization = OFF");
    if (do_saving)
        ROS_INFO("  Saving = ON");
    else
        ROS_INFO("  Saving = OFF");
    ROS_INFO(" ----------------");

    return true;
}

bool load_map_locations(const string &map_locations_file, const double &height, vector<Eigen::Vector4f> &map_locations,
                        vector<pair<Eigen::Vector4f,int> > &locations_with_ix, vector<Eigen::Vector4f> &road_map, sensor_msgs::Image &in_image)
{
    map_locations.clear();
    locations_with_ix.clear();
    road_map.clear();
    // Open the map locations file
    ifstream myfile (map_locations_file.c_str());
    if (myfile.is_open())
    {
        // Read each line
        string line;
        int count = 0;
        while (getline(myfile, line))
        {
            stringstream linestream (line);
            double x, y;
            linestream >> x >> y;
            // Store in the vector
            Eigen::Vector4f loc;
            loc[0] = x;
            loc[1] = y;
            loc[2] = height;
            loc[3] = 0.0;
            map_locations.push_back(loc);
            locations_with_ix.push_back(make_pair(loc,count));
            ++count;
            road_map.push_back(loc);
        }
        myfile.close();
    }
    else
    {
        ROS_ERROR("squirrel_run_robot_experiment::load_map_locations : could not open %s", map_locations_file.c_str());
        return false;
    }

    int num_locs = map_locations.size();
    if (num_locs == 0)
    {
        ROS_ERROR("squirrel_run_robot_experiment::load_map_locations : map locations is empty");
        return false;
    }
    if (locations_with_ix.size() != num_locs || road_map.size() != num_locs)
    {
        ROS_ERROR("squirrel_run_robot_experiment::load_map_locations : vector mismatch, locations has %lu, indices has %lu, road map has %lu",
                  map_locations.size(), locations_with_ix.size(), road_map.size());
        return false;
    }

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
    cv_ptr->toImageMsg(in_image);

    return true;
}

bool save_cloud_to_file(ros::NodeHandle &n, const string &stored_points_dir, const int &count)
{
    // Point cloud
    sensor_msgs::PointCloud2ConstPtr scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", n, ros::Duration(50));
    PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*scene,pcl_pc2);
    PointCloud<PointT> cloud;
    fromPCLPointCloud2(pcl_pc2, cloud);
    ROS_INFO("squirrel_run_robot_experiment::save_cloud_to_file : read %lu points", cloud.size());
    string f = add_backslash(stored_points_dir) + _CLOUD_PREFIX + boost::lexical_cast<string>(count) + ".pcd";
    io::savePCDFileBinary (f, cloud);
    ROS_INFO("squirrel_run_robot_experiment::save_cloud_to_file : point cloud saved");

    // Transform
    tf::StampedTransform transform;
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("/kinect_depth_optical_frame", "/map", ros::Time(0), ros::Duration(5.0));
    tf_listener.lookupTransform ("/kinect_depth_optical_frame", "/map", ros::Time(0), transform);
    tf::Vector3 t = transform.getOrigin();
    tf::Matrix3x3 r = transform.getBasis();
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf(0,0) = r[0][0];
    tf(0,1) = r[0][1];
    tf(0,2) = r[0][2];
    tf(0,3) = t[0];
    tf(1,0) = r[1][0];
    tf(1,1) = r[1][1];
    tf(1,2) = r[1][2];
    tf(1,3) = t[1];
    tf(2,0) = r[2][0];
    tf(2,1) = r[2][1];
    tf(2,2) = r[2][2];
    tf(2,3) = t[2];
    tf = tf.inverse();  // inverse the transform
    ofstream out;
    f = add_backslash(stored_points_dir) + _TRANSFORMATION_PREFIX + boost::lexical_cast<string>(count) + ".txt";
    out.open(f.c_str());
    // Write results
    if (out.is_open())
    {
        out << tf(0,0) << " " << tf(0,1) << " " << tf(0,2) << " " << tf(0,3) << " "
            << tf(1,0) << " " << tf(1,1) << " " << tf(1,2) << " " << tf(1,3) << " "
            << tf(2,0) << " " << tf(2,1) << " " << tf(2,2) << " " << tf(2,3) << " "
            << tf(3,0) << " " << tf(3,1) << " " << tf(3,2) << " " << tf(3,3) << endl;
        out.close();
        ROS_INFO("squirrel_run_robot_experiment::save_cloud_to_file : transformation saved");
        cout << tf << endl;
    }
    else
    {
        ROS_ERROR("squirrel_run_robot_experiment::main : could not open file to save transform %s", f.c_str());
        return false;
    }
    // Save a transformed point cloud for verification
    PointCloud<PointT> transformed_cloud;
    transformPointCloud (cloud, transformed_cloud, tf);
    f = add_backslash(stored_points_dir) + _VERIFICATION_PREFIX + boost::lexical_cast<string>(count) + ".pcd";
    io::savePCDFileBinary (f, transformed_cloud);
    return true;
}

bool verify_point_cloud()
{
    cout << "Verify the point cloud is aligned correctly" << endl;
    string input;
    cout << "Is the point cloud usable [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        return true;
    }
    // If "n" input
    else if (input.compare("n") == 0 || input.compare("no") == 0 ||
             input.compare("N") == 0 || input.compare("NO") == 0)
    {
        return false;
    }
}

bool load_current_cloud_and_transform(const string &stored_points_dir, const int &count,
                                      sensor_msgs::PointCloud2 &current_cloud, Eigen::Matrix4f &current_transform)
{
    // Load the point cloud
    PointCloud<PointT> in_cloud;
    string f = add_backslash(stored_points_dir) + _CLOUD_PREFIX + boost::lexical_cast<string>(count) + ".pcd";
    io::loadPCDFile(f, in_cloud);
    toROSMsg(in_cloud, current_cloud);

    // Load the transform
    current_transform = Eigen::Matrix4f::Identity();
    f = add_backslash(stored_points_dir) + _TRANSFORMATION_PREFIX + boost::lexical_cast<string>(count) + ".txt";
    if (!read_tf_file(f, current_transform))
    {
        ROS_ERROR("squirrel_run_robot_experiment::load_current_cloud_and_transform : error reading transform file %s", f.c_str());
        return false;
    }
    return true;
}

bool move_to_next_location(RobotController *rc, const vector<Eigen::Vector4f> &road_map, const int &from_ix, const int &to_ix)
{
    ROS_INFO("squirrel_run_robot_experiment::move_to_next_location : requesting path from %i to %i", from_ix, to_ix);
    ROS_INFO("squirrel_run_robot_experiment::move_to_next_location : road map has size %lu", road_map.size());
    if (from_ix < 0 || from_ix >= road_map.size())
    {
        ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : invalid _from_ index %u when road map has %lu elements",
                  from_ix, road_map.size());
        return false;
    }
    if (to_ix < 0 || to_ix >= road_map.size())
    {
        ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : invalid _to_ index %u when road map has %lu elements",
                  to_ix, road_map.size());
        return false;
    }
    // Find the shortest path forwards or backwards from from_ix to to_ix through the road_map
    // Forwards
    int i = from_ix;
    Eigen::Vector4f previous_loc;
    vector<Eigen::Vector4f> forward_path;
    double forward_dist = 0;
    if (i >= 0 && i < road_map.size())
    {
        previous_loc = road_map[i];
        while (i != to_ix)
        {
            // Increment to the next location in the road map
            ++i;
            // Go to beginning of vector if reached the end
            if (i >= road_map.size())
                i = 0;
            // Compute the distance from the previous location to this location
            forward_dist += distance3D(previous_loc[0], previous_loc[1], 0, road_map[i][0], road_map[i][1], 0);
            forward_path.push_back(road_map[i]);
            cout << "forward path " << road_map[i][0] << " " << road_map[i][1] << " (" << i << ")" << endl;
            // If this index is the one being searched for then it will exit
        }
    }
    else
    {
        forward_dist = numeric_limits<double>::infinity();
    }
    cout << "forward path distance " << forward_dist << endl;
    cout << "forward path vector has " << forward_path.size() << " elements" << endl;
    // Backward path
    i = from_ix;
    vector<Eigen::Vector4f> backward_path;
    double backward_dist = 0;
    if (i >= 0 && i < road_map.size())
    {
        previous_loc = road_map[i];
        while (i != to_ix)
        {
            // Decrement to the next location in the road map
            --i;
            // Go to beginning of vector if reached the end
            if (i < 0)
                i = road_map.size() - 1;
            // Compute the distance from the previous location to this location
            backward_dist += distance3D(previous_loc[0], previous_loc[1], 0, road_map[i][0], road_map[i][1], 0);
            backward_path.push_back(road_map[i]);
            cout << "backward path " << road_map[i][0] << " " << road_map[i][1] << " (" << i << ")" << endl;
            // If this index is the one being searched for then it will exit
        }
    }
    else
    {
        backward_dist = numeric_limits<double>::infinity();
    }
    cout << "backward path distance " << backward_dist << endl;
    cout << "backward path vector has " << backward_path.size() << " elements" << endl;

    // If both are invalid then exit
    if (forward_dist == numeric_limits<double>::infinity() && backward_dist == numeric_limits<double>::infinity())
    {
        ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : both paths have infinity length!");
        return false;
    }
    if (forward_path.size() == 0 && backward_path.size() == 0)
    {
        ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : both paths have zero elements!");
        return false;
    }

    // Choose the shorter path
    vector<Eigen::Vector4f> shortest_path;
    if (forward_dist < backward_dist)
        shortest_path = forward_path;
    else
        shortest_path = backward_path;

    // Send move commands
    double x, y;
    if (shortest_path.size() == 0)
    {
        ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : shortest path has zero elements!");
        return false;
    }
    else
    {
        if (shortest_path.size() > 1)
        {
            for (vector<Eigen::Vector4f>::size_type j = 0; j < shortest_path.size()-1; ++j)
            {
                if (j == 1 || j == 3 || j == 5 || j == 7 || j == 9 || j == 11)
                {
                    // Move to the next location
                    x = shortest_path[j][0];
                    y = shortest_path[j][1];
                    if (!rc->move_to_waypoint(x, y))
                    {
                        ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : could not move to waypoint %lu in path of length %lu",
                                  j, shortest_path.size());
                        return false;
                    }
                }
            }
        }
        // Move slowly to the very last waypoint
        x = shortest_path.back()[0];
        y = shortest_path.back()[1];
        if (!rc->move_to_waypoint_slow(x, y))
        {
            ROS_ERROR("squirrel_run_robot_experiment::move_to_next_location : could not move to final waypoint in path");
            return false;
        }
    }

    return true;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    // Seed the random number generator
    srand(time(NULL));

    // Create and initialize the active exploration object
    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);
    // Create and initialize the robot controller object
    RobotController *rc (new RobotController());
    rc->initialize(argc, argv);

    // Get the input
    string map_locations_file;
    string stored_points_dir;
    double variance;
    SIM_TYPE sim;
    int expected_number_objects;
    int expected_number_classes;
    double scene_center_x;
    double scene_center_y;
    double kinect_height;
    bool do_visualize;
    bool do_saving;
    if (!parse_input(*exp->get_ros_node_handle(), map_locations_file, stored_points_dir, variance, sim,
                     expected_number_objects, expected_number_classes, scene_center_x, scene_center_y, kinect_height,
                     do_visualize, do_saving))
    {
        ROS_ERROR("squirrel_run_robot_experiment::main : could not parse the input");
        return EXIT_FAILURE;
    }
    string locations_save_file = add_backslash(stored_points_dir) + _LOCATIONS_ORDER_FILENAME;

    // Load the map locations
    vector<Eigen::Vector4f> map_locations;
    vector<pair<Eigen::Vector4f,int> > map_locations_with_indices;
    vector<Eigen::Vector4f> road_map;
    sensor_msgs::Image in_image;
    if (!load_map_locations(map_locations_file, kinect_height, map_locations, map_locations_with_indices, road_map, in_image))
    {
        ROS_ERROR("squirrel_run_robot_experiment::main : could not load the data");
        if (exp)
            delete exp;
        if (rc)
            delete rc;
        return EXIT_FAILURE;
    }

    // Turn on visualization
    if (do_visualize)
        exp->turn_on_visualization();
    // Turn on saving
    if (do_saving)
        exp->turn_on_saving(expected_number_objects, expected_number_classes);

    // Get the current location of the robot
    Eigen::Vector4f current_location;
    int robot_position_loop = 0;
    while (true)
    {
        if (rc->robot_position_in_map(current_location))
        {
            break;
        }
        else
        {
            ++robot_position_loop;
            if (robot_position_loop >= 5)
            {
                ROS_ERROR("squirrel_run_robot_experiment::main : could not get the current robot location");
                if (exp)
                    delete exp;
                if (rc)
                    delete rc;
                return EXIT_FAILURE;
            }
        }
    }

    // Find the closest location in map_locations
    int closest_map_location = -1;
    double dist = numeric_limits<double>::infinity();
    for (vector<Eigen::Vector4f>::size_type i = 0; i < map_locations.size(); ++i)
    {
        double d = sqrt(pow(current_location[0]-map_locations[i][0],2) + pow(current_location[1]-map_locations[i][1],2));
        if (d < dist)
        {
            dist = d;
            closest_map_location = i;
        }
    }
    ROS_INFO("squirrel_run_robot_experiment::main : nearest to location %i with distance %.2f", closest_map_location, dist);
    // Save to file
    if (do_saving)
    {
        ofstream out;
        out.open(locations_save_file.c_str(), ios::app);
        // Write results
        if (out.is_open())
        {
            // If the distance is sufficiently small then this location is to be saved then removed from the list
            if (closest_map_location >= 0 && dist < 0.25)
            {
                // Write the index
                out << closest_map_location << " ";
                // Write the map location
                out << current_location[0] << " " << current_location[1] << " " << current_location[2] << endl;
            }
            else
            {
                // Write null index and fake location
                out << "-1 0 0 0 " << endl;
            }
            out.close();
        }
        else
        {
            ROS_WARN("squirrel_run_robot_experiment::main : could not open file to save entropy %s", locations_save_file.c_str());
        }
    }
    if (closest_map_location >= 0 && dist < 0.25)
    {
        ROS_INFO("squirrel_run_robot_experiment::main : currently at location [%.2f %2f]", current_location[0], current_location[1]);
        ROS_INFO("squirrel_run_robot_experiment::main : this is location %i in road map", map_locations_with_indices[closest_map_location].second);
        // Remove from list
        map_locations.erase(map_locations.begin() + closest_map_location);
        map_locations_with_indices.erase(map_locations_with_indices.begin() + closest_map_location);
    }

    // Loop through the map locations
    sensor_msgs::PointCloud2 in_cloud;
    Eigen::Matrix4f in_transform;
    int previous_road_map_index = closest_map_location;
    int count = 0;
    while (ros::ok())
    {
//        // Get the current cloud
//        cout << "Press enter to capture new point cloud to index " << count << endl;
//        cin.ignore();
//        cout << "Capturing point cloud ..." << endl;
//        if (!save_cloud_to_file(*exp->get_ros_node_handle(), stored_points_dir, count))
//        {
//            ROS_ERROR("squirrel_run_robot_experiment::main : error capturing point cloud");
//            if (exp)
//                delete exp;
//            if (rc)
//                delete rc;
//            return EXIT_FAILURE;
//        }
//        // Need to verify the point cloud before proceeding
//        if (count > 0)
//        {
//            while (!verify_point_cloud())
//            {
//                cout << "Press enter to save new point cloud to index " << count << endl;
//                cin.ignore();
//                cout << "Capturing point cloud ..." << endl;
//                if (!save_cloud_to_file(*exp->get_ros_node_handle(), stored_points_dir, count))
//                {
//                    ROS_ERROR("squirrel_run_robot_experiment::main : error capturing point cloud");
//                    if (exp)
//                        delete exp;
//                    if (rc)
//                        delete rc;
//                    return EXIT_FAILURE;
//                }
//            }
//        }

        bool point_cloud_captured = false;
        string ready_input;
        while (!point_cloud_captured)
        {
            cout << "Type ready when point cloud is successfully captured ... ";
            cin >> ready_input;
            if (ready_input.compare("ready") == 0 || ready_input.compare("READY") == 0)
                point_cloud_captured = true;
        }

        // Load the point cloud and transform from file
        if (!load_current_cloud_and_transform(stored_points_dir, count, in_cloud, in_transform))
        {
            ROS_ERROR("squirrel_run_robot_experiment::main : error in loading the current cloud and transform for index %u", count);
            if (exp)
                delete exp;
            if (rc)
                delete rc;
            return EXIT_FAILURE;
        }

        // Process the point cloud
        exp->set_data(in_cloud, in_image, in_transform);
        // Initialize visualizer
        if (!exp->initialize_visualization())
        {
            ROS_ERROR("squirrel_run_robot_experiment::main : error in initializing visualization");
            if (exp)
                delete exp;
            if (rc)
                delete rc;
            return EXIT_FAILURE;
        }
        // Update the hypotheses
        ROS_INFO("squirrel_run_robot_experiment::main : updating hypotheses");
        if (!exp->update_hypotheses(in_cloud, in_image, in_transform))
        {
            ROS_ERROR("squirrel_run_robot_experiment::main : error when updating hypotheses point cloud");
            if (exp)
                delete exp;
            if (rc)
                delete rc;
            return EXIT_FAILURE;
        }

        if (do_visualize)
        {
            if (!exp->visualize())
            {
                ROS_ERROR("squirrel_run_robot_experiment::main : error when visualizing the point cloud segments");
                if (exp)
                    delete exp;
                if (rc)
                    delete rc;
                return EXIT_FAILURE;
            }
        }

        if (map_locations.size() == 0)
        {
            ROS_WARN("squirrel_run_robot_experiment::main : map locations is empty");
            break;
        }

        // Plan
        // It will return the index of the best location in transformed locations
        ROS_INFO("squirrel_run_robot_experiment::main : planning");
        int next_best_index = -1;
        if (!exp->plan(next_best_index, sim, variance, map_locations))
        {
            ROS_ERROR("squirrel_run_robot_experiment::main : error when planning");
            if (exp)
                delete exp;
            if (rc)
                delete rc;
            return EXIT_FAILURE;
        }

        // If it is an invalid index
        if (next_best_index < 0 || next_best_index >= map_locations.size())
        {
            ROS_WARN("squirrel_run_robot_experiment::main : next best view index %u is invalid", next_best_index);
            break;
        }

        // Get the next view
        current_location = map_locations[next_best_index];
        // Write to the locations file
        if (do_saving)
        {
            ofstream out;
            out.open(locations_save_file.c_str(), ios::app);
            // Write results
            if (out.is_open())
            {
                // Write the index
                out << map_locations_with_indices[next_best_index].second << " ";
                // Write the map location
                out << current_location[0] << " " << current_location[1] << " " << current_location[2] << endl;
                out.close();
            }
            else
            {
                ROS_WARN("squirrel_run_robot_experiment::main : could not open file to save entropy %s", locations_save_file.c_str());
            }
        }

        // Move to the next best view
        int road_map_index = map_locations_with_indices[next_best_index].second;
        ROS_INFO("squirrel_run_robot_experiment::main : previous road map index %i", previous_road_map_index);
        ROS_INFO("squirrel_run_robot_experiment::main : next road map index %i", road_map_index);
        if (!move_to_next_location(rc, road_map, previous_road_map_index, road_map_index))
        {
            ROS_ERROR("squirrel_run_robot_experiment::main : could not move to the next location");
            if (exp)
                delete exp;
            if (rc)
                delete rc;
            return EXIT_FAILURE;
        }
        // Rotate to focus area
        if (!rc->rotate_slow(scene_center_x, scene_center_y))
        {
            ROS_ERROR("squirrel_run_robot_experiment::main : could not rotate to the scene center");
            if (exp)
                delete exp;
            if (rc)
                delete rc;
            return EXIT_FAILURE;
        }
        previous_road_map_index = road_map_index;

        // Remove from list
        map_locations.erase(map_locations.begin() + next_best_index);
        map_locations_with_indices.erase(map_locations_with_indices.begin() + next_best_index);

        // Increment the count
        ++count;
    }

    if (exp)
        delete exp;
    if (rc)
        delete rc;

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

