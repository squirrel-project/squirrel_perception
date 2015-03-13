/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CONVERT_NORMALS_HPP
#define KP_CONVERT_NORMALS_HPP

#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "v4r/KeypointTools/DataMatrix2D.hpp"
#include "v4r/KeypointTools/PointTypes.hpp"


namespace kp 
{


inline void convertNormals(const kp::DataMatrix2D<Eigen::Vector3f> &kp_normals, pcl::PointCloud<pcl::Normal> &pcl_normals)
{
  pcl_normals.points.resize(kp_normals.data.size());
  pcl_normals.width = kp_normals.cols;
  pcl_normals.height = kp_normals.rows;
  pcl_normals.is_dense = false;

  for (unsigned i=0; i<pcl_normals.points.size(); i++)
  {
    pcl_normals.points[i].getNormalVector3fMap() = kp_normals.data[i];
  }
}


} //--END--

#endif

