#ifndef VISUALIZATION_UTILS_H
#define VISUALIZATION_UTILS_H

#include <stdlib.h>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

typedef pcl::PointXYZRGB PointT;

/* === COLOR DATA STRUCTURE AND FUNCTIONS === */

namespace color
{
    struct HSV
    {
        float h;
        float s;
        float v;
    };

    struct RGB
    {
        float r;
        float g;
        float b;
    };
}

std::vector<color::HSV> generate_HSV(const int n, const bool &print_results = true);

std::vector<color::RGB> generate_RGB(const int n, const bool &print_results = true);

// http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
color::RGB hsv_to_rgb(const color::HSV &hsv);

color::HSV rgb_to_hsv(const color::RGB &rgb);

/* === VISUALIZATION === */

namespace visualization_u
{
    void visualize_point_cloud(const std::vector<pcl::PointCloud<PointT>::Ptr> &in_clouds, const int &max = -1);

    void visualize_point_cloud(const std::vector<pcl::PointCloud<PointT> > &in_clouds, const int &max = -1);

    void visualize_point_cloud(const pcl::PointCloud<PointT>::Ptr in_cloud, const int &max = -1);

    void visualize_point_cloud(const pcl::PointCloud<PointT> &in_cloud, const int &max = -1);

    void run_viewer(pcl::visualization::PCLVisualizer *viewer, bool *exit_status);
}

#endif // VISUALIZATION_UTILS_H
