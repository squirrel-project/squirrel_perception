#include "squirrel_active_exploration/math_utils.h"

using namespace std;

/* === MATHEMATICS === */

// http://stackoverflow.com/questions/5891811/generate-random-number-between-1-and-3-in-c
int int_rand(const int &min, const int &max)
{
    // x is in [0,1[
   double x = rand()/static_cast<double>(RAND_MAX);
   // [0,1[ * (max - min) + min is in [min,max[
   int that = min + static_cast<int>(x*(max-min));

   return that;
}

double distance3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2)
{
    return sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2));
}

double eigdistance3D(const Eigen::Vector4f &eigvec1, const Eigen::Vector4f &eigvec2)
{
    return distance3D(eigvec1[0], eigvec1[1], eigvec1[2], eigvec2[0], eigvec2[1], eigvec2[2]);
}

pair<double, double> angle3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2,
                             const double &radius)
{
    pair<double,double> res;
    res.first = atan2(y2-y1, x2-x1);
    res.second = acos((z2-z1) / radius);
    return res;
}

pair<double,double> angle3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2)
{
    return angle3D(x1,y1,z1,x2,y2,z2,distance3D(x1,y1,z1,x2,y2,z2));
}

Eigen::Vector4f polar_coordinate(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2)
{
    Eigen::Vector4f res;
    double radius = distance3D(x1,y1,z1,x2,y2,z2);
    pair<double,double> angles = angle3D(x1,y1,z1,x2,y2,z2,radius);
    res[0] = radius;
    res[1] = angles.first;
    res[2] = angles.second;
    return res;
}

double gaussian_func(const double &mean, const double &variance)
{
    // Defined as 1/[sigma*sqrt(2pi)]*e^[-1/2((x-mu)/sigma)^2]
    return ( exp(-pow(mean/variance,2)/2) / (variance*sqrt(2*M_PI)) );
}

double entropy(const vector<double> &vec)
{
    // Sum the entropy from each element
    double res = 0;
    for (vector<double>::const_iterator it = vec.begin(); it != vec.end(); ++it)
        res -= (*it) * log(*it);

    return res;
}

double entropy(const vector<float> &vec)
{
    vector<double> vecd;
    vecd.resize(vec.size());
    for (vector<double>::size_type i = 0; i < vec.size(); ++i)
        vecd[i] = (double)vec[i];
    return entropy(vecd);
}

vector<double> entropy(const vector<vector<double> > &vec)
{
    vector<double> res;
    for (vector<vector<double> >::const_iterator it = vec.begin(); it != vec.end(); ++it)
        res.push_back (entropy(*it));
    return res;
}

vector<double> entropy(const vector<vector<float> > &vec)
{
    vector<double> res;
    for (vector<vector<float> >::const_iterator it = vec.begin(); it != vec.end(); ++it)
        res.push_back (entropy(*it));
    return res;
}

vector<int> rank(const vector<double> &vec)
{
    // Create a vector of pairs where pair = (ix, value)
    vector<pair<int,double> > paired_vec;
    paired_vec.resize(vec.size());
    int i = 0;
    for (vector<double>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        paired_vec[i] = make_pair (i,*it);
        ++i;
    }
    // Sort the vector using the comparator function
    sort(paired_vec.begin(), paired_vec.end(), compare<double>);
    // Extract the indices from the sorted vector
    vector<int> res;
    for (vector<pair<int,double> >::const_iterator it = paired_vec.begin(); it != paired_vec.end(); ++it)
        res.push_back(it->first);
    return res;
}

vector<int> rank(const vector<float> &vec)
{
    vector<double> vecd;
    vecd.resize(vec.size());
    for (vector<double>::size_type i = 0; i < vec.size(); ++i)
        vecd[i] = (double)vec[i];
    return rank(vecd);
}

///* === TEMPLATE DEFINITIONS === */
//template double distance3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);
//template float distance3D(const float &x1, const float &y1, const float &z1, const float &x2, const float &y2, const float &z2);
//template double eigdistance3D(const Eigen::Vector4d &eigvec1, const Eigen::Vector4d &eigvec2);
//template double eigdistance3D(const Eigen::Vector4f &eigvec1, const Eigen::Vector4f &eigvec2);
//template pair<double,double> angle3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2, const double &radius);
//template pair<float,float> angle3D(const float &x1, const float &y1, const float &z1, const float &x2, const float &y2, const float &z2, const float &radius);
//template pair<double,double> angle3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);
//template pair<float,float> angle3D(const float &x1, const float &y1, const float &z1, const float &x2, const float &y2, const float &z2);
//template Eigen::Vector4f polar_coordinate(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);
//template Eigen::Vector4f polar_coordinate(const float &x1, const float &y1, const float &z1, const float &x2, const float &y2, const float &z2);
//template double gaussian_func(const double &mean, const double &variance);
//template float gaussian_func(const float &mean, const float &variance);
//template double entropy(const vector<double> &vec);
//template float entropy(const vector<float> &vec);
//template vector<double> entropy(const vector<vector<double> > &vec);
//template vector<float> entropy(const vector<vector<float> > &vec);
//template vector<int> rank(const vector<double> &vec);
//template vector<int> rank(const vector<float> &vec);
