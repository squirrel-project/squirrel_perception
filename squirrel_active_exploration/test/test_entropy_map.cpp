#include "ros/ros.h"
#include <squirrel_active_exploration/EntropyMap.h>
#include <squirrel_active_exploration/EntropyMapViz.h>

#include "squirrel_active_exploration/pcl_conversions.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "squirrel_active_exploration/math_utils.h"
#include "squirrel_active_exploration/visualization_utils.h"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

bool compare_int_double_pair(const pair<int,double> &lhs, const pair<int,double> &rhs)
{
    return lhs.second < rhs.second;
}

// Run Main
int main(int argc, char **argv)
{
    ROS_INFO("*** STARTING TEST ***");

    ros::init(argc, argv, "test_entropy_map");
    ros::NodeHandle n("~");

    string class_type = "apple";
    string instance_name = "3a92a256ad1e060ec048697b91f69d2";
    int view_index = -1;
    string score_type = "area";
    //string class_type = "book";
    //string instance_name = "bacef3777b91b02f29c5890b07f3a65";

    // Read the input parameters
    n.getParam("class_type", class_type);
    n.getParam("instance_name", instance_name);
    n.getParam("view_index", view_index);
    n.getParam("score_type", score_type);

    if (view_index >= 50)
        view_index = -1;

    ROS_INFO("TEST_entropy_map::main : parameters");
    ROS_INFO("  Class Type = %s", class_type.c_str());
    ROS_INFO("  Instance Name = %s", instance_name.c_str());
    ROS_INFO("  View Index = %i", view_index);
    ROS_INFO("  Score Type = %s", score_type.c_str());

    vector<PointCloud<PointT>::Ptr> clouds;

    ros::ServiceClient em_client = n.serviceClient<squirrel_active_exploration::EntropyMap>("/squirrel_entropy_map");
    squirrel_active_exploration::EntropyMap em_srv;
    ros::ServiceClient em_viz_client = n.serviceClient<squirrel_active_exploration::EntropyMapViz>("/squirrel_entropy_map_visualize");
    squirrel_active_exploration::EntropyMapViz em_vis_srv;
    // Call the services
    em_srv.request.class_type.data = class_type;
    em_srv.request.instance_name.data = instance_name;
    if (!em_client.call(em_srv))
    {
        ROS_ERROR("TEST_entropy_map::main : could not call the entropy map service");
        return EXIT_FAILURE;
    }
    else
    {
        ROS_INFO("TEST_entropy_map::main : successfully extracted entropy map with %lu elements and instance cloud size %lu",
                 em_srv.response.entropy_map.size(), em_srv.response.cloud.data.size());
        ROS_INFO("TEST_entropy_map::main : octree file %s", em_srv.response.octree_file.data.c_str());
        // Print out the details of each instance
        for (size_t i = 0; i < em_srv.response.entropy_map.size(); ++i)
        {
            ROS_INFO(" - view %s :", em_srv.response.entropy_map[i].view_name.data.c_str());
            ROS_INFO("  points %lu", em_srv.response.entropy_map[i].cloud.data.size());
            std_msgs::Float64MultiArray transform = em_srv.response.entropy_map[i].transform;
            ROS_INFO("  transform %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*0 + transform.layout.dim[2].stride*0],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*0 + transform.layout.dim[2].stride*1],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*0 + transform.layout.dim[2].stride*2],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*0 + transform.layout.dim[2].stride*3],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*1 + transform.layout.dim[2].stride*0],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*1 + transform.layout.dim[2].stride*1],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*1 + transform.layout.dim[2].stride*2],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*1 + transform.layout.dim[2].stride*3],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*2 + transform.layout.dim[2].stride*0],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*2 + transform.layout.dim[2].stride*1],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*2 + transform.layout.dim[2].stride*2],
                     transform.data[transform.layout.data_offset + transform.layout.dim[1].stride*2 + transform.layout.dim[2].stride*3]);
            ROS_INFO("  centroid %.1f %.1f %.1f",
                     em_srv.response.entropy_map[i].centroid[0],
                     em_srv.response.entropy_map[i].centroid[1],
                     em_srv.response.entropy_map[i].centroid[2]);
            geometry_msgs::Point pose = em_srv.response.entropy_map[i].camera_pose.position;
            ROS_INFO("  pose %.1f %.1f %.1f", pose.x, pose.y, pose.z);
            ROS_INFO("  surface area %.2f", em_srv.response.entropy_map[i].surface_area_proportion);
            ROS_INFO("  class entropy %.2f", em_srv.response.entropy_map[i].class_entropy);
            ROS_INFO("  recognition %.2f", em_srv.response.entropy_map[i].recognition_probability);

            // Transform to the model
            std_msgs::Float64MultiArray in_t = em_srv.response.entropy_map[i].transform;
            Eigen::Matrix4f out_t = Eigen::Matrix4f::Identity();
            int r = 0, c = 0;
            for (int m = 0; m < in_t.data.size(); ++m)
            {
                out_t(r,c) = in_t.data[m];
                ++c;
                if (c == 4)
                {
                    ++r;
                    c = 0;
                }
            }

            PointCloud<PointT>::Ptr pos (new PointCloud<PointT>());
            pos->push_back(PointT(pose.x, pose.y, pose.z));
            transformPointCloud(*pos, *pos, out_t);
            clouds.push_back(pos);
        }

        // Print out the comparison of visible entropy and the number of visible points
        vector<pair<int,double> > surface_areas;
        vector<pair<int,double> > points_percentage;
        for (size_t i = 0; i < em_srv.response.entropy_map.size(); ++i)
        {
            surface_areas.push_back(make_pair(i,em_srv.response.entropy_map[i].surface_area_proportion));
            points_percentage.push_back(make_pair(i,(double)em_srv.response.entropy_map[i].cloud.data.size()));
        }
        double max_area = max_element(surface_areas.begin(), surface_areas.end(), compare_int_double_pair)->second;
        double max_perc = max_element(points_percentage.begin(), points_percentage.end(), compare_int_double_pair)->second;
        for (size_t i = 0; i < surface_areas.size(); ++i)
        {
            cout << i << " area " << surface_areas[i].second << " (" << (surface_areas[i].second/max_area)*100.0 << "%)"
                 << " -> pts " << points_percentage[i].second*100.0 << " (" << (points_percentage[i].second/max_perc)*100.0 << "%)" << endl;
        }
        // Rank the surface areas and the points percentage
        sort(surface_areas.begin(), surface_areas.end(), compare_int_double_pair);
        sort(points_percentage.begin(), points_percentage.end(), compare_int_double_pair);
        cout << "SORTED" << endl;
        for (size_t i = 0; i < surface_areas.size(); ++i)
        {
            cout << surface_areas[i].first << " " << surface_areas[i].second << " "
                 << points_percentage[i].first << " " << points_percentage[i].second << endl;
        }
        cout << "REVERSED" << endl;
        reverse(points_percentage.begin(), points_percentage.end());
        for (size_t i = 0; i < surface_areas.size(); ++i)
        {
            cout << surface_areas[i].first << " " << surface_areas[i].second << " "
                 << points_percentage[i].first << " " << points_percentage[i].second << endl;
        }


        PointCloud<PointT>::Ptr total_cloud (new PointCloud<PointT>());
        pcl::fromROSMsg(em_srv.response.cloud, *total_cloud);
        clouds.push_back(total_cloud);
        // Visualize the entropy map
        if (em_srv.response.entropy_map.size() > 0)
        {
            em_vis_srv.request.class_type.data = class_type;
            em_vis_srv.request.instance_name.data = instance_name;
            em_vis_srv.request.view_index = view_index;
            em_vis_srv.request.score_type.data = score_type;
            if (!em_viz_client.call(em_vis_srv))
            {
                ROS_ERROR("TEST_entropy_map::main : could not call the entropy map visualization service");
                return EXIT_FAILURE;
            }
        }
        else
        {
            ROS_WARN("TEST_entropy_map::main : requested entropy map has zero elements");
        }

    }

//    // Visualize the point clouds here
//    visualize_point_cloud (clouds);

    // Display the point clouds
    visualization::PCLVisualizer viewer("Point clouds");
    viewer.setBackgroundColor(1.0, 1.0, 1.0, 0);
    string f = "cloud";  // point cloud name
    for (int c = 0; c < clouds.size(); ++c)
    {
        f = "cloud_" + boost::lexical_cast<string>(c);
        int s = 1;
        if (clouds[c]->size() == 1)
        {
            s = 10;
            visualization::PointCloudColorHandlerCustom<PointT> cloud_handler (clouds[c], 0, 0, 255);
            viewer.addPointCloud (clouds[c], cloud_handler, f);
        }
        else
        {
            visualization::PointCloudColorHandlerCustom<PointT> cloud_handler (clouds[c], 255, 255, 0);
            viewer.addPointCloud (clouds[c], cloud_handler, f);
        }
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, s, f);
    }

    // Wait for the viewer to close before exiting
    while (!viewer.wasStopped ())
        viewer.spinOnce ();

    ROS_INFO("*** FINISH TEST ***");
    ros::shutdown();
    return EXIT_SUCCESS;
}
