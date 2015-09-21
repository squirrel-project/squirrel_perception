#include <ros/ros.h>
#include "squirrel_active_exploration/math_utils.h"

using namespace std;

// Run Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "active_exploration");
    ros::NodeHandle node("~");

//    string f = "/home/tpat//hello//k//l//";
//    string from = "//";
//    string to = "/";
//    replace_substr(f,from,to);
//    cout << f << endl;

//    // Test Gaussian function
//    double variance = 0.5;
//    vector<double> means;
//    means.push_back(0.786489);
//    means.push_back(1.06442);
//    means.push_back(1.40764);
//    means.push_back(1.2599);
//    means.push_back(0.540742);
//    means.push_back(0.705379);
//    means.push_back(1.24813);
//    means.push_back(1.02777);

//    for (size_t i = 0; i < means.size(); ++i)
//        cout << means[i] << " -> " << gaussian_func(means[i], variance) << endl;

    // Test the random number generator
    srand(time(NULL));
    for (int i = 20; i < 50; ++i)
    {
        cout << "---" << endl;
        cout << "N = " << i << ": rand = " << int_rand(0,i);
        sleep(0.5);
        cout << " " << int_rand(0,i);
        sleep(0.5);
        cout << " " << int_rand(0,i);
        sleep(0.5);
        cout << " " << int_rand(0,i);
        sleep(0.5);
        cout << " " << int_rand(0,i) << endl;
        cout << "---" << endl;
    }

    ros::shutdown();
    return EXIT_SUCCESS;
}
