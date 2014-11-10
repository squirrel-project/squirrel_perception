// Bring in my package's API, which is what I'm testing
#include <squirrel_attention_location.hpp>
// Bring in gtest
#include <gtest/gtest.h>

class ClientAttentionLocation : public testing::Test 
{
  
protected:
  // Remember that SetUp() is run immediately before a test starts.
  virtual void SetUp() 
  {
    n_ = new ros::NodeHandle ("~");
    n_->getParam ( "cloud_name", cloud_name_);
    readCloud ();
    
    n_->getParam ( "ground_truth", ground_truth_);
    readGroundTruth();
    
  }

  // TearDown() is invoked immediately after a test finishes.
  virtual void TearDown() 
  {
    if(n_)
      delete n_;
  }

  //memebers
  typedef pcl::PointXYZRGB PointT;
  std::string cloud_name_;
  pcl::PointCloud<PointT>::Ptr cloud_;
  sensor_msgs::PointCloud2 msg;
  
  std::string ground_truth_;
  cv::Mat groundTruth;
  
  //int service_calls_;
  //ros::NodeHandle n_;
  ros::NodeHandle *n_;
  
  void readCloud ()
  {
    cloud_.reset(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (cloud_name_.c_str(), *cloud_) == -1) //* load the file
    {
      ROS_ERROR ("Couldn't read file!\n");
      return;
    }
    
    pcl::toROSMsg (*cloud_,msg);
    ROS_INFO("Cloud is up and running!");
  }
  
  void readGroundTruth ()
  {
    groundTruth= cv::imread(ground_truth_,-1);
    
    ROS_INFO("Ground Truth is up and running!");
  }

public:

};

TEST_F(ClientAttentionLocation, testClientAttentionLocation_1) 
{
  std::cout << "going to call service..." << std::endl;
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::get_saliency_location>("/squirrel_attention_location");
  squirrel_object_perception_msgs::get_saliency_location srv;
  srv.request.cloud = msg;
  srv.request.location.data = 0;
  
  EXPECT_TRUE(client.call(srv));
  
  //msg->cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat saliency_map = cv_ptr->image;
  
  for(int i = 0; i < saliency_map.rows; ++i)
  {
    for(int j = 0; j < saliency_map.cols; ++j)
    {
      EXPECT_EQ(saliency_map.at<uchar>(i,j),groundTruth.at<uchar>(i,j));
    }
  }
}

int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "squirrel_attention_location_test");
  return RUN_ALL_TESTS();
}