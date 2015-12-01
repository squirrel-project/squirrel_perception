#include "squirrel_active_exploration/math_utils.h"
#include "squirrel_active_exploration/transform_utils.h"
#include "squirrel_active_exploration/io_utils.h"
#include <sensor_msgs/PointCloud2.h>
#include <boost/algorithm/string.hpp>
#include <cstdlib>

using namespace std;
using namespace pcl;

#define _CLOUD_PREFIX "cloud_"
#define _TRANSFORMATION_PREFIX "transformation_"
#define _ADJUSTMENT_PREFIX "adjusted_cloud_"
#define _NEW_TRANSFORMATION_PREFIX "new_transformation_"

bool save_transform_to_file(const string &dir, const int &id, const Eigen::Matrix4f &tf)
{
    ofstream out;
    string f = add_backslash(dir) + _NEW_TRANSFORMATION_PREFIX + boost::lexical_cast<string>(id) + ".txt";
    out.open(f.c_str());
    // Write results
    if (out.is_open())
    {
        out << tf(0,0) << " " << tf(0,1) << " " << tf(0,2) << " " << tf(0,3) << " "
            << tf(1,0) << " " << tf(1,1) << " " << tf(1,2) << " " << tf(1,3) << " "
            << tf(2,0) << " " << tf(2,1) << " " << tf(2,2) << " " << tf(2,3) << " "
            << tf(3,0) << " " << tf(3,1) << " " << tf(3,2) << " " << tf(3,3) << endl;
        out.close();
        ROS_INFO("manual_adjust_point_cloud::save_transform_to_file : transformation saved");
        cout << tf << endl;
    }
    else
    {
        ROS_ERROR("manual_adjust_point_cloud::save_transform_to_file : could not open file to save transform %s", f.c_str());
        return false;
    }
    return true;
}

bool verify_point_cloud(bool &use_transform)
{
    use_transform = false;
    cout << "Verify the point cloud is aligned correctly" << endl;
    string input;
    cout << "Is the point cloud usable [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        use_transform = true;
        return true;
    }
    // If "n" input
    else if (input.compare("n") == 0 || input.compare("no") == 0 ||
             input.compare("N") == 0 || input.compare("NO") == 0)
    {
        use_transform = false;
        return true;
    }
    else
    {
        ROS_WARN("manual_adjust_point_cloud::verify_point_cloud : invalid input %s", input.c_str());
        return false;
    }
}

bool load_current_cloud_and_transform(const string &dir, const int &id,
                                      sensor_msgs::PointCloud2 &current_cloud, Eigen::Matrix4f &current_transform)
{
    // Load the point cloud
    PointCloud<PointT> in_cloud;
    string f = add_backslash(dir) + _CLOUD_PREFIX + boost::lexical_cast<string>(id) + ".pcd";
    io::loadPCDFile(f, in_cloud);
    toROSMsg(in_cloud, current_cloud);

    // Load the transform
    current_transform = Eigen::Matrix4f::Identity();
    f = add_backslash(dir) + _TRANSFORMATION_PREFIX + boost::lexical_cast<string>(id) + ".txt";
    if (!read_tf_file(f, current_transform))
    {
        ROS_ERROR("manual_adjust_point_cloud::load_current_cloud_and_transform : error reading transform file %s", f.c_str());
        return false;
    }
    return true;
}

bool get_input(bool &quit, bool &inverse, double &x, double &y)
{
    quit = false;
    inverse = false;
    x = 0;
    y = 0;
    cout << "Enter the x and y adjustments (or i for inverse or q for quit)" << endl;

    string num1, num2;

    cin >> num1;
    if (num1.find_first_not_of("1234567890.-") != string::npos)
    {
        // Check if the input is "i" or "q"
        if (num1.compare("i") == 0 || num1.compare("inverse") == 0 || num1.compare("I") == 0 || num1.compare("INVERSE") == 0)
        {
            ROS_INFO("manual_adjust_point_cloud::get_input : requested inverse transform");
            inverse = true;
            return true;
        }
        else if (num1.compare("q") == 0 || num1.compare("quit") == 0 || num1.compare("Q") == 0 || num1.compare("QUIT") == 0)
         {
             ROS_INFO("manual_adjust_point_cloud::get_input : quitting");
             quit = true;
             return true;
         }
        else
        {
            ROS_ERROR("manual_adjust_point_cloud::get_input : invalid input %s", num1.c_str());
            return false;
        }
    }
    else
    {
        x = atof(num1.c_str());
    }

    cin >> num2;
    if (num2.find_first_not_of("1234567890.-") != string::npos)
    {
        ROS_ERROR("manual_adjust_point_cloud::get_input : invalid input %s", num2.c_str());
        return false;
    }
    else
    {
        y = atof(num2.c_str());
    }
    return true;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    ros::init(argc, argv, "manual_adjust_point_cloud");
    ros::NodeHandle n("~");

    // Get the directory
    string dir = "";
    n.getParam("directory", dir);
    if (dir.size() == 0)
    {
        ROS_ERROR("manual_adjust_point_cloud::main : you must enter a directory!");
        return EXIT_FAILURE;
    }
    // Get the id
    int id = -1;
    n.getParam("id", id);
    if (id < 0)
    {
        ROS_ERROR("manual_adjust_point_cloud::main : you must enter a cloud id!");
        return EXIT_FAILURE;
    }

    // Load the cloud
    sensor_msgs::PointCloud2 current_cloud_pcl;
    Eigen::Matrix4f current_transform = Eigen::Matrix4f::Identity();
    if (!load_current_cloud_and_transform(dir, id, current_cloud_pcl, current_transform))
    {
        ROS_WARN("manual_adjust_point_cloud::main : could not load cloud and transform");
        return EXIT_SUCCESS;
    }
    PointCloud<PointT> current_cloud;
    pcl::fromROSMsg(current_cloud_pcl, current_cloud);
    ROS_INFO("manual_adjust_point_cloud::main : current transform");
    cout << current_transform << endl;

    double x, y;
    bool inverse, quit, use_transform;
    PointCloud<PointT> transformed_cloud;
    string f = add_backslash(dir) + _ADJUSTMENT_PREFIX + boost::lexical_cast<string>(id) + ".pcd";
    while (true)
    {
        Eigen::Matrix4f tf = current_transform;
        // Get the adjustment parameters
        if (!get_input(quit, inverse, x, y))
        {
            ROS_ERROR("manual_adjust_point_cloud::main : could not get valid input from terminal");
            return EXIT_FAILURE;
        }
        // If quit
        if (quit)
            break;
        // If inverse
        if (inverse)
        {
            tf = tf.inverse();
        }
        else
        {
            // Apply the adjustment parameters to the transformation
            tf(0,3) = tf(0,3) + x;
            tf(1,3) = tf(1,3) + y;
        }

        // Transform the cloud
        transformPointCloud(current_cloud, transformed_cloud, tf);
        // Save to file
        io::savePCDFileBinary(f, transformed_cloud);

        // Verify the cloud
        if (!verify_point_cloud(use_transform))
        {
            ROS_ERROR("manual_adjust_point_cloud::main : could not get valid input from terminal for cloud verification");
            return EXIT_FAILURE;
        }

        // If use the transform then save it
        if (use_transform)
        {
            if (!save_transform_to_file(dir, id, tf))
            {
                ROS_ERROR("manual_adjust_point_cloud::main : could not save the new transformation");
                return EXIT_FAILURE;
            }
            break;
        }
        // Otherwise continue the loop again
    }

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}

