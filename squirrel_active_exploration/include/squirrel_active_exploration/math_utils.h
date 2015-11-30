#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

/* === COMPARISONS === */

// Comparator structure for using Eigen::Vector4f as a map key
struct compare_eig_vec4f
{
    bool operator()(const Eigen::Vector4f& lhs, const Eigen::Vector4f& rhs) const
    {
        if (lhs[0] < rhs[0]) return true;
        if (rhs[0] < lhs[0]) return false;
        // Otherwise, lhs[0] == rhs[0]
        if (lhs[1] < rhs[1]) return true;
        if (rhs[1] < lhs[1]) return false;
        // Otherwise, lhs[1] == rhs[1]
        if (lhs[2] < rhs[2]) return true;
        if (rhs[2] < lhs[2]) return false;
        // Otherwise, lhs[3] == rhs[3]
        if (lhs[3] < rhs[3]) return true;
        if (rhs[3] < lhs[3]) return false;
    }
};

template<typename T>
bool compare(const std::pair<int,T> &lhs, const std::pair<int,T> &rhs)
{
    return (lhs.second < rhs.second);
}

/* === MATHEMATICS === */

int int_rand(const int &min, const int &max);

double distance3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);

double eigdistance3D(const Eigen::Vector4f &eigvec1, const Eigen::Vector4f &eigvec2);

std::pair<double,double> angle3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2,
                                 const double &radius);

std::pair<double,double> angle3D(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);

Eigen::Vector4f polar_coordinate(const double &x1, const double &y1, const double &z1, const double &x2, const double &y2, const double &z2);

double gaussian_func(const double &mean, const double &variance);

double entropy(const std::vector<double> &vec);

double entropy(const std::vector<float> &vec);

std::vector<double> entropy(const std::vector<std::vector<double> > &vec);

std::vector<double> entropy(const std::vector<std::vector<float> > &vec);

std::vector<int> rank(const std::vector<double> &vec);

std::vector<int> rank(const std::vector<float> &vec);


#endif // UTILS_H
