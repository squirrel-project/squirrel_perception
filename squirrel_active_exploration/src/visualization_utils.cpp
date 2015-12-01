#include "squirrel_active_exploration/visualization_utils.h"

using namespace std;
using namespace pcl;

/* === COLOR DATA STRUCTURE AND FUNCTIONS === */

vector<color::HSV> generate_HSV(const int n, const bool &print_results)
{
    vector<color::HSV> colors;
    // If the input is less than 0
    if (n <= 0)
    {
        printf(ANSI_COLOR_RED  "ERROR color_utils::generate_HSV : input must be larger than 0!"  ANSI_COLOR_RESET "\n");
        return colors;
    }
    // If requesting 1 color then return white
    // If requesting 2 colors then return white and red
    // If requesting 3 colors then return white, red and green
    // If requesting 4 colors then return white, red, green and blue
    else if (n <= 4)
    {
        vector<color::RGB> rgb_colors = generate_RGB (n, false);
        // Convert to HSV
        colors.resize(rgb_colors.size());
        for (vector<color::RGB>::size_type i = 0; i < rgb_colors.size(); ++i)
            colors[i] = rgb_to_hsv(rgb_colors[i]);
    }
    // Otherwise generate evenly distributed colors in HSV space
    else
    {
        for (int i = 0; i < 360; i += 360 / n)
        {
            color::HSV c;
            c.h = i;
            //c.s = 0.9 + ((float)rand()/(float)RAND_MAX) / 10;
            //c.v = 0.5 + ((float)rand()/(float)RAND_MAX) / 10;
            c.s = 1.0;
            c.v = 1.0;
            colors.push_back (c);
        }
    }
    // Check the size
    if (colors.size() < n)
    {
        printf(ANSI_COLOR_YELLOW  "WARN color_utils::generate_HSV : could not generate the requested number of colors!"  ANSI_COLOR_RESET "\n");
        printf(ANSI_COLOR_YELLOW  "WARN color_utils::generate_HSV : requested %u but generated %lu"  ANSI_COLOR_RESET "\n", n, colors.size());
    }
    // Print out the hsv values
    if (print_results)
    {
        printf("*** GENERATED %lu HSV COLORS ***\n", colors.size());
        for (vector<color::HSV>::size_type i = 0; i < colors.size(); ++i)
        {
            // Cast to ints for formatted output
            float h = colors[i].h;
            float s = colors[i].s;
            float v = colors[i].v;
            printf("  %04lu : %3.2f %3.2f %3.2f\n", i, h, s, v);
        }
        printf("******************************\n");
    }
    return colors;
}

vector<color::RGB> generate_RGB(const int n, const bool &print_results)
{
    vector<color::RGB> colors;
    // If the input is less than 0
    if (n <= 0)
    {
        printf(ANSI_COLOR_RED  "ERROR color_utils::generate_RGB : input must be larger than 0!"  ANSI_COLOR_RESET "\n");
        return colors;
    }
    if (n == 1)
    {
        printf(ANSI_COLOR_YELLOW  "WARN color_utils::generate_RGB : requesting one color!"  ANSI_COLOR_RESET "\n");
    }

    // Generate the colors
    if (n <= 4)
    {
        // If requesting 1 color then return white
        color::RGB white;
        white.r = 255;
        white.g = 255;
        white.b = 255;
        colors.push_back(white);
        // If requesting 2 colors then return white and red
        if (n > 1)
        {
            color::RGB red;
            red.r = 255;
            red.g = 0;
            red.b = 0;
            colors.push_back(red);
        }
        // If requesting 3 colors then return white, red and green
        if (n > 2)
        {
            color::RGB green;
            green.r = 0;
            green.g = 255;
            green.b = 0;
            colors.push_back(green);
        }
        // If requesting 4 colors then return white, red, green and blue
        if (n > 3)
        {
            color::RGB blue;
            blue.r = 0;
            blue.g = 0;
            blue.b = 255;
            colors.push_back(blue);
        }
    }
    // Otherwise generate in HSV space and convert back to RGB
    else
    {
        // Generate colors in HSV
        vector<color::HSV> hsv = generate_HSV(n, false);
        colors.resize(hsv.size());
            printf("Colors %lu\n", hsv.size());
        for (vector<color::HSV>::size_type i = 0; i < hsv.size(); ++i)
            colors[i] = hsv_to_rgb (hsv[i]);
    }
    // Print out the hsv values
    if (print_results)
    {
        printf("*** GENERATED %lu RGB COLORS ***\n", colors.size());
        for (vector<color::HSV>::size_type i = 0; i < colors.size(); ++i)
        {
            // Cast to ints for formatted output
            float r = colors[i].r;
            float g = colors[i].g;
            float b = colors[i].b;
            printf("  %04lu : %3.0f %3.0f %3.0f \n", i, r, g, b);
        }
        printf("******************************\n");
    }
    return colors;
}

color::RGB hsv_to_rgb(const color::HSV &hsv)
{
    double hh, p, q, t, ff;
    long i;
    color::RGB rgb;

    if (hsv.s <= 0.0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }
    hh = hsv.h;
    if (hh >= 360.0)
        hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = hsv.v * (1.0 - hsv.s);
    q = hsv.v * (1.0 - (hsv.s * ff));
    t = hsv.v * (1.0 - (hsv.s * (1.0 - ff)));

    switch(i)
    {
    case 0:
        rgb.r = hsv.v;
        rgb.g = t;
        rgb.b = p;
        break;
    case 1:
        rgb.r = q;
        rgb.g = hsv.v;
        rgb.b = p;
        break;
    case 2:
        rgb.r = p;
        rgb.g = hsv.v;
        rgb.b = t;
        break;
    case 3:
        rgb.r = p;
        rgb.g = q;
        rgb.b = hsv.v;
        break;
    case 4:
        rgb.r = t;
        rgb.g = p;
        rgb.b = hsv.v;
        break;
    case 5:
    default:
        rgb.r = hsv.v;
        rgb.g = p;
        rgb.b = q;
        break;
    }

    // Scale to 255
    rgb.r *= 255;
    rgb.g *= 255;
    rgb.b *= 255;
    return rgb;
}

color::HSV rgb_to_hsv(const color::RGB &rgb)
{
    color::HSV hsv;

//    // Scale to (0,1)
//    rgb.r /= 255;
//    rgb.g /= 255;
//    rgb.b /= 255;

    double min = rgb.r < rgb.g ? rgb.r : rgb.g;
    min = min  < rgb.b ? min  : rgb.b;
    double max = rgb.r > rgb.g ? rgb.r : rgb.g;
    max = max  > rgb.b ? max  : rgb.b;

    hsv.v = max;                                // v
    double delta = max - min;
    if (max > 0.0) // NOTE: if Max is == 0, this divide would cause a crash
    {
        hsv.s = (delta / max);                  // s
    }
    else
    {
        // if max is 0, then r = g = b = 0
            // s = 0, v is undefined
        hsv.s = 0.0;
        hsv.h = NAN;                            // its now undefined
        return hsv;
    }
    if (rgb.r >= max)                           // > is bogus, just keeps compilor happy
        hsv.h = (rgb.g - rgb.b) / delta;        // between yellow & magenta
    else
    {
        if (rgb.g >= max)
            hsv.h = 2.0 + (rgb.b - rgb.r) / delta;  // between cyan & yellow
        else
            hsv.h = 4.0 + (rgb.r - rgb.g) / delta;  // between magenta & cyan
    }

    hsv.h *= 60.0;                              // degrees

    if (hsv.h < 0.0)
        hsv.h += 360.0;

    return hsv;
}

/* === VISUALIZATION === */

void visualization_u::visualize_point_cloud(const vector<PointCloud<PointT>::Ptr> &in_clouds, const int &max)
{
    if (in_clouds.size() == 0)
    {
        printf(ANSI_COLOR_RED  "ERROR utils::visualize_point_cloud : empty vector"  ANSI_COLOR_RESET "\n");
        return;
    }
    // Generate colors for the number of input clouds
    vector<color::RGB> colors;
    if (in_clouds.size() == 1)
    {
        color::RGB white;
        white.r = 255;
        white.g = 255;
        white.b = 255;
        colors.push_back (white);
    }
    else
    {
        colors = generate_RGB(in_clouds.size());
    }
    // Display the point clouds
    visualization::PCLVisualizer viewer("Point clouds");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    string f = "cloud";  // point cloud name
    color::RGB c = colors[0];  // point cloud color
    int s = 2;  // point cloud size
    int max_clouds = in_clouds.size();
    if (max > 0 && max <= in_clouds.size())
        max_clouds = max-1;
    //cout << "max = " << max << ", in_clouds.size() = " << in_clouds.size() << ", max_clouds = " << max_clouds << endl;
    for (vector<PointCloud<PointT>::Ptr>::size_type i = 0; i < max_clouds; ++i)
    {
        if (in_clouds[i]->size() > 0)
        {
            // If many points then this is an observation
            if (in_clouds[i]->size() > 1)
            {
                f = "cloud" + boost::lexical_cast<string>(i);
                c = colors[i];
                s = 2;
            }
            // Otherwise this is a robot location
            else
            {
                f = "robot" + boost::lexical_cast<string>(i);
                // Robot is always blue
                c.r = 0;
                c.g = 0;
                c.b = 255;
                // Robot is larger
                s = 10;
            }
            // Define R,G,B colors for the point cloud
            visualization::PointCloudColorHandlerCustom<PointT> cloud_handler (in_clouds[i], c.r, c.g, c.b);
            // Add the point cloud to the viewer and pass the color handler
            viewer.addPointCloud (in_clouds[i], cloud_handler, f);
            // Set the point cloud properties
            viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, s, f);
        }
        else
        {
            printf(ANSI_COLOR_YELLOW  "WARN utils::visualize_point_cloud : point cloud %lu is empty!"  ANSI_COLOR_RESET "\n", i);
        }
    }

    // Wait for the viewer to close before exiting
    while (!viewer.wasStopped ())
        viewer.spinOnce ();
}

void visualization_u::visualize_point_cloud(const vector<PointCloud<PointT> > &in_clouds, const int &max)
{
    // Create a pointer to each point cloud
    vector<PointCloud<PointT>::Ptr> ptr_clouds;
    for (vector<PointCloud<PointT> >::size_type i = 0; i < in_clouds.size(); ++i)
    {
        PointCloud<PointT>::Ptr ptr_cloud (new PointCloud<PointT>(in_clouds[i]));
        ptr_clouds.push_back (ptr_cloud);
    }
    visualize_point_cloud (ptr_clouds, max);
}

void visualization_u::visualize_point_cloud(const PointCloud<PointT>::Ptr in_cloud, const int &max)
{
    // Create a vector with one elemet
    vector<PointCloud<PointT>::Ptr> clouds;
    PointCloud<PointT>::Ptr non_const_cloud (new PointCloud<PointT>(*in_cloud));
    clouds.push_back (non_const_cloud);
    visualize_point_cloud (clouds, max);
}

void visualization_u::visualize_point_cloud(const PointCloud<PointT> &in_cloud, const int &max)
{
    // Create a pointer to the point cloud
    PointCloud<PointT>::Ptr ptr_cloud (new PointCloud<PointT>(in_cloud));
    visualize_point_cloud (ptr_cloud, max);
}

void visualization_u::run_viewer(visualization::PCLVisualizer *viewer, bool *exit_status)
{
    printf("visualization_utils::run_viewer : starting \n");
    // Display the visualiser until 'q' key is pressed
    while (!viewer->wasStopped ())
        viewer->spinOnce ();
    printf("visualizarion_utils::run_viewer : finished \n");
    *exit_status = true;
}
