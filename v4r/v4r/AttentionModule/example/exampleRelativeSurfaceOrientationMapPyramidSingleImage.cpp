#include <opencv2/opencv.hpp>

#include "v4r/AttentionModule/AttentionModule.hpp"
#include "v4r/EPUtils/EPUtils.hpp"

int main(int argc, char** argv)
{
  
  // read image
  std::string image_name(argv[1]);
  cv::Mat image = cv::imread(image_name,-1);
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  std::string cloud_name(argv[2]);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_name,*cloud) == -1)
  {
    std::cerr << "[ERROR] Couldn't read point cloud." << std::endl;
    return -1;
  }
  
  // start creating parameters
  AttentionModule::RelativeSurfaceOrientationMapParameters parameters;
  parameters.width = image.cols;
  parameters.height = image.rows;
  // set camera parameters
  parameters.cameraParametrs.clear();
  parameters.cameraParametrs.resize(4);
  parameters.cameraParametrs.at(0) = 525.0f;
  parameters.cameraParametrs.at(1) = 525.0f;
  parameters.cameraParametrs.at(2) = 319.5f;
  parameters.cameraParametrs.at(3) = 239.5f;
  
  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZ>(cloud,indices))
    return(pclAddOns::FILTER);
  
  // segment plane
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
  pcl::PointIndices::Ptr objects_indices(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  if(!pclAddOns::SegmentPlane<pcl::PointXYZ>(cloud,indices,plane_indices,objects_indices,coefficients))
  {
    return(pclAddOns::SEGMENT);
  }
  
  parameters.cloud = cloud;
  parameters.indices = objects_indices;
  
  parameters.normal.normal[0] = coefficients->values[0];
  parameters.normal.normal[1] = coefficients->values[1];
  parameters.normal.normal[2] = coefficients->values[2];
  
  CalculateRelativeSurfaceOrientationMapPyramid(parameters);
  
  cv::imshow("map",parameters.map);
  cv::waitKey();
    
  return(0);
}