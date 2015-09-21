#include <cstdlib>
#include <iostream>
#include <string>
#include "squirrel_active_exploration/pcl_conversions.h"
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <dirent.h>

#include "squirrel_active_exploration/math_utils.h"
#include "squirrel_active_exploration/file_utils.h"

#define _STAMPED_POSE_PREFIX "stampedPose_"
#define _TRANSFORMATION_PREFIX "transformation_"

using namespace std;
using namespace pcl;

int main(int argc, char **argv)
{
    // Initalize ros
    ros::init(argc, argv, "cloud2pcd");
    ros::NodeHandle nh( "~" );
    // Read the directory name
    string dir = "/home/tpat8946/Data/TUW/TUW_GH30_dataset/set_001";
    nh.getParam("directory", dir);
    ROS_INFO("pose_to_transform::main : converting files in directory %s", dir.c_str());

    DIR *dp;
    struct dirent *dirp;
    if ((dp  = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR("pose_to_transform::main : could not open %s", dir.c_str());
        return EXIT_FAILURE;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            //cout << f << endl;
            // Get the number
            size_t underscore = f.find_last_of('_');
            size_t dot = f.find_last_of('.');
            if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
            {
                int str_len = dot - underscore;
                string str_ix = f.substr(underscore+1,str_len-1);
                int ix = atoi(str_ix.c_str());
                string filename = add_backslash(dir) + f;

                if (strncmp(f.c_str(),_STAMPED_POSE_PREFIX, sizeof(_STAMPED_POSE_PREFIX)-1) == 0)
                {
                    // Open the file
                    ifstream myfile (filename.c_str());
                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    bool valid_transform = false;
                    if (myfile.is_open())
                    {
                        int word_count = 0;
                        string line;
                        while (getline(myfile, line))
                        {
                            stringstream linestream (line);
                            if (word_count == 0)
                            {
                                double x, y, z;
                                linestream >> x >> y >> z;
                                transform(0,3) = x;
                                transform(1,3) = y;
                                transform(2,3) = z;
                                ++word_count;
                            }
                            else if (word_count == 1)
                            {
                                double r11, r12, r13, r21, r22, r23, r31, r32, r33;
                                linestream >> r11 >> r12 >> r13 >> r21 >> r22 >> r23 >> r31 >> r32 >> r33;
                                transform(0,0) = r11;
                                transform(0,1) = r12;
                                transform(0,2) = r13;
                                transform(1,0) = r21;
                                transform(1,1) = r22;
                                transform(1,2) = r23;
                                transform(2,0) = r31;
                                transform(2,1) = r32;
                                transform(2,2) = r33;
                                valid_transform = true;
                            }
                        }
                        myfile.close();
                    }
                    else
                    {
                        ROS_ERROR("pose_to_transform::main : could not open file %s", filename.c_str());
                    }
                    // Write the transform to file
                    if (valid_transform)
                    {
                        transform = transform.inverse();
                        string out_str_ix = boost::lexical_cast<string>(ix);
                        while (out_str_ix.size() < str_ix.size())
                            out_str_ix = "0" + out_str_ix;
                        filename = add_backslash(dir) + _TRANSFORMATION_PREFIX + out_str_ix + ".txt";
                        ofstream myfile2 (filename.c_str());
                        if (myfile2.is_open())
                        {
                            myfile2 << transform(0,0) << " " << transform(0,1) << " " << transform(0,2) << " " << transform(0,3) << " "
                                    << transform(1,0) << " " << transform(1,1) << " " << transform(1,2) << " " << transform(1,3) << " "
                                    << transform(2,0) << " " << transform(2,1) << " " << transform(2,2) << " " << transform(2,3) << " "
                                    << transform(3,0) << " " << transform(3,1) << " " << transform(3,2) << " " << transform(3,3) << endl;
                            myfile2.close();
                        }
                    }
                }
            }
        }
    }
    closedir(dp);

    return EXIT_SUCCESS;
}

