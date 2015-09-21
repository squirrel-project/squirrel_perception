#include "squirrel_active_exploration/robot_controller.h"
#include "squirrel_active_exploration/active_exploration.h"

using namespace std;
using namespace pcl;

bool get_waypoint(double &x, double &y)
{
    cout << "x: ";
    cin >> x;
    if (!cin)
    {
        ROS_ERROR("TEST_robot_controller::get_waypoint : invalid input");
        return false;
    }
    cout << "y: ";
    cin >> y;
    if (!cin)
    {
        ROS_ERROR("TEST_robot_controller::get_waypoint : invalid input");
        return false;
    }
    return true;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");
    string dir = "/home/tpat8946/Data/TUW/TUW_dynamic_dataset_icra15/test_set/set_00001/";
    string pc_file = "00000.pcd";

    // Start Robot Controller
    RobotController *rc (new RobotController());
    rc->initialize(argc, argv);
//    // Load a point cloud
//    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>());
//    PointCloud<PointT>::Ptr robot (new PointCloud<PointT>());
//    robot->resize(1);
//    robot->points[0].x = 0;
//    robot->points[0].y = 0;
//    robot->points[0].z = 0;
//    string cloud_filename = add_backslash(dir) + pc_file;
//    if (io::loadPCDFile<PointT> (cloud_filename.c_str(), *cloud) == -1)
//    {
//        ROS_ERROR("test_robot_controller::main : could not read point cloud file %s", cloud_filename.c_str());
//        exit_code = EXIT_FAILURE;
//    }
//    else
//    {
//        PointCloud<PointT>::Ptr cloud_mp (new PointCloud<PointT>());
//        PointCloud<PointT>::Ptr robot_mp (new PointCloud<PointT>());
//        // If successfully found transform
//        if (rc->transforms_available())
//        {
//            rc->pointcloud_to_map(*cloud, *cloud_mp);
//            rc->pointcloud_to_map(*robot, *robot_mp);
//        }
//        // Otherwise look for transform file
//        else
//        {
//            transform_cloud_from_file(dir, pc_file, *cloud, *cloud_mp, *robot_mp);
//        }
//        // Visualize the original point cloud
//        vector<PointCloud<PointT>::Ptr> original_clouds;
//        original_clouds.push_back (cloud);
//        original_clouds.push_back (robot);
//        visualize_point_cloud(original_clouds);
//        // Visualize the point cloud in map coordinates
//        vector<PointCloud<PointT>::Ptr> map_clouds;
//        map_clouds.push_back (cloud_mp);
//        map_clouds.push_back (robot_mp);
//        visualize_point_cloud(map_clouds);
//        // Visualize the point clouds together
//        vector<PointCloud<PointT>::Ptr> all_clouds;
//        all_clouds.push_back (cloud);
//        all_clouds.push_back (cloud_mp);
//        all_clouds.push_back (robot);
//        all_clouds.push_back (robot_mp);
//        visualize_point_cloud(all_clouds);
//    }

    // Testing create an active exploration object
    ActiveExploration *exp = new ActiveExploration();
    exp->initialize(argc, argv);

    // Get the scene center from user
    double center_x, center_y;
    cout << "Enter scene center" << endl;
    if (!get_waypoint(center_x, center_y))
    {
        ROS_ERROR("TEST_robot_controller::main : could not get valid input from user");
        return EXIT_FAILURE;
    }
//    center_x = 1.58;
//    center_y = -1.24;

    // Get waypoint from user
    double x, y;
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f inv_tf = Eigen::Matrix4f::Identity();
    PointCloud<PointT> current_cloud;
    PointCloud<PointT> transformed_cloud;
    string f;
    int counter = 0;
    while (true)
    {
        cout << "Enter waypoint" << endl;
        if (!get_waypoint(x, y))
        {
            ROS_ERROR("TEST_robot_controller::main : could not get valid input from user");
            return EXIT_FAILURE;
        }
        // Add waypoint to list
        if (!rc->move_to_waypoint(x, y))
        {
            ROS_ERROR("TEST_robot_controller::main : could not add the waypoint");
            return EXIT_FAILURE;
        }
        // Rotate to focus area
        if (!rc->rotate_slow(center_x, center_y))
        {
            ROS_ERROR("TEST_robot_controller::main : could not rotate to the scene center");
            return EXIT_FAILURE;
        }
        // Clear the sensor input
        exp->clear_sensor_input();
        // Get the transform
        tf = rc->pointcloud_to_map_tf();
        // Get the data from the sensor
        exp->data_from_sensor(tf);
        // Get the point cloud
        current_cloud = exp->get_cloud();
        f = "/home/tpat8946/raw_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
        io::savePCDFileBinary(f, current_cloud);
//        // Transform the cloud here
//        transformPointCloud(current_cloud, transformed_cloud, tf);
//        f = "/home/tpat8946/transformed_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
//        io::savePCDFileBinary(f, transformed_cloud);
//        // Inverse transform the cloud here
//        inv_tf = tf.inverse();
//        transformPointCloud(current_cloud, transformed_cloud, inv_tf);
//        f = "/home/tpat8946/inverse_transformed_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
//        io::savePCDFileBinary(f, transformed_cloud);
//        // Transform the cloud in robot controller
//        rc->pointcloud_to_map(current_cloud, transformed_cloud);
//        f = "/home/tpat8946/rc_transformed_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
//        io::savePCDFileBinary(f, transformed_cloud);
//        // Get the transformed cloud from the active exploration object
//        transformed_cloud = exp->get_transformed_cloud();
//        f = "/home/tpat8946/ae_transformed_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
//        io::savePCDFileBinary(f, transformed_cloud);

        tf::StampedTransform transform;
        tf::TransformListener tf_listener;
        tf_listener.waitForTransform("/kinect_depth_optical_frame", "/map", ros::Time(0), ros::Duration(5.0));
        tf_listener.lookupTransform ("/kinect_depth_optical_frame", "/map", ros::Time(0), transform);

        tf::Vector3 t = transform.getOrigin();
        tf::Matrix3x3 r = transform.getBasis();
        tf = Eigen::Matrix4f::Identity();
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
        // Transform the cloud here
        transformPointCloud(current_cloud, transformed_cloud, tf);
        f = "/home/tpat8946/transformed_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
        io::savePCDFileBinary(f, transformed_cloud);
        // Inverse transform the cloud here
        inv_tf = tf.inverse();
        transformPointCloud(current_cloud, transformed_cloud, inv_tf);
        f = "/home/tpat8946/inverse_transformed_cloud_" + boost::lexical_cast<string>(counter) + ".pcd";
        io::savePCDFileBinary(f, transformed_cloud);

        // Increment counter
        ++counter;
    }

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}
