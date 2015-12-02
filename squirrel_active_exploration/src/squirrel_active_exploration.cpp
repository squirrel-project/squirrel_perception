#include "squirrel_active_exploration/active_exploration.h"

using namespace std;

// Get input
bool get_input(ActiveExploration *exp, int &exit_code, bool replace = true)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Get input from sensor [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if (!exp->data_from_sensor())
        {
            ROS_ERROR("squirrel_active_exploration::get_input : error when getting input from the sensor");
            exit_code = EXIT_FAILURE;
            return false;
        }
        return true;
    }
    // If "n" input
    else if (input.compare("n") == 0 || input.compare("no") == 0 ||
             input.compare("N") == 0 || input.compare("NO") == 0)
    {
        ROS_WARN("squirrel_active_exploration::get_input : skipping getting input");
        return true;
    }
    else
    {
        string cloud_name;
        cout << "Enter point cloud filename : ";
        cin >> cloud_name;
        string transform_name;
        cout << "Enter transform filename : ";
        cin >> transform_name;
        string image_name;
        cout << "Enter image filename : ";
        cin >> image_name;
        if (!exp->read_input(cloud_name, transform_name, image_name))
        {
            ROS_ERROR("squirrel_active_exploration::get_input : error when reading input from file");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
}

// Check process
bool check_process(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Process the point cloud [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->process())
        {
            ROS_ERROR("squirrel_active_exploration::check_process : error when processing the point cloud");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_process : skipping processing");
        return false;
    }
    return true;
}

// Check segmentation
bool check_segmentation(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Segment point cloud [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->segment())
        {
            ROS_ERROR("squirrel_active_exploration::check_segmentation : error when segmenting the point cloud");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_segmentation : skipping segmentation");
    }
    return true;
}

// Check filter segmentation
bool check_filter_segmentation(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Filter the segments [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->filter_segments())
        {
            ROS_ERROR("squirrel_active_exploration::check_filter_segmentation : error when filtering segmentation");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_filter_segmentation : skipping filtering segmentation");
    }
    return true;
}

// Check estimate pose
bool check_estimate_pose(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Estimate poses [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->estimate_pose())
        {
            ROS_ERROR("squirrel_active_exploration::check_estimate_pose : error when estimating the poses");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_estimate_pose : skipping estimating pose");
    }
    return true;
}

// Check classification
bool check_classification(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Classify the segments [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->classify())
        {
            ROS_ERROR("squirrel_active_exploration::check_classification : error when classifying");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_classification : skipping classification");
    }
    return true;
}

// Check compute entropy
bool check_compute_entropy(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Compute entropy [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->compute_entropy())
        {
            ROS_ERROR("squirrel_active_exploration::check_compute_entropy : error when computing the entropy");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_compute_entropy : skipping computing entropy");
    }
    return true;
}

// Check rank entropy
bool check_rank_entropy(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Rank the entropies [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->rank_entropy())
        {
            ROS_ERROR("squirrel_active_exploration::check_rank_entropy : error when ranking the entropies");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_rank_entropy : skipping ranking entropies");
    }
    return true;
}

// Check visualization
bool check_visualization(ActiveExploration *exp, int &exit_code)
{
    exit_code = EXIT_SUCCESS;
    string input;
    cout << "Visualize segments [y/n]? ";
    cin >> input;
    // If "y" input
    if (input.compare("y") == 0 || input.compare("yes") == 0 ||
        input.compare("Y") == 0 || input.compare("YES") == 0)
    {
        if(!exp->visualize())
        {
            ROS_ERROR("squirrel_active_exploration::check_visualization : error when visualizing the point cloud segments");
            exit_code = EXIT_FAILURE;
            return false;
        }
    }
    // Otherwise skip
    else
    {
        ROS_WARN("squirrel_active_exploration::check_visualization : skipping visualization");
    }
    return true;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING SQUIRREL_ACTIVE_EXPLORATION ***");
    int exit_code = EXIT_SUCCESS;

    ActiveExploration *exp (new ActiveExploration());
    exp->initialize(argc, argv);
    get_input(exp,exit_code);

    // Loop for ever ...
    while (ros::ok() && exit_code == EXIT_SUCCESS)
    {
        // Process the point cloud
        if (!check_process(exp,exit_code))
        {
            if (exit_code == EXIT_FAILURE)
            {
                break;
            }
            // Segment
            if (!check_segmentation(exp,exit_code))
            {
                break;
            }
            // Filter segmentation
            if (!check_filter_segmentation(exp,exit_code))
            {
                break;
            }
            // Estimate pose
            if (!check_estimate_pose(exp,exit_code))
            {
                break;
            }
            // Classify
            if (!check_classification(exp,exit_code))
            {
                break;
            }
            // Compute entropy
            if (!check_compute_entropy(exp,exit_code))
            {
                break;
            }
            // Rank entropy
            if (!check_rank_entropy(exp,exit_code))
            {
                break;
            }
        }
        // Visualize
        if (!check_visualization(exp,exit_code))
        {
            break;
        }
        // Get new input
        if (!get_input(exp,exit_code))
        {
            break;
        }
    }

    ROS_INFO("*** FINISH SQUIRREL_ACTIVE_EXPLORATION ***");
    ros::shutdown();
    return exit_code;
}
