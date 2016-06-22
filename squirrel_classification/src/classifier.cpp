/*
 * Just a wrapper around the v4r_ros_wrappers wrapper for the v4r classifier.
 * Yes, we are gangsta wrappers.
 *
 *  Created on: Sep 7, 2013
 *      Author: Michael Zillich, zillich@acin.tuwien.ac.at
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <classifier_srv_definitions/classify.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/Classification.h>

class Classifier
{
  private:
    ros::NodeHandle n_;
    ros::ServiceServer classify_service_;

    /**
     * Returns index of most confident class.
     * NOTE: Actually not needed at the moment, but let's keep it, just in case.
     */
    int find_best_class(object_perception_msgs::classification &classification)
    {
      int best_idx = -1;
      float best_conf = 0.;
      assert(classification.class_type.size() == classification.confidence.size());
      for(size_t i = 0; i < classification.class_type.size(); i++)
      {
        if(classification.confidence[i] > best_conf)
        {
          best_conf = classification.confidence[i];
          best_idx = (int)i;
        }
      }
      return best_idx;
    }

    /**
     * The actual classification method, calling the v4r classification library.
     */
    bool classify(squirrel_object_perception_msgs::Classify::Request & req,
                  squirrel_object_perception_msgs::Classify::Response & response)
    {
      ROS_INFO("Classifying %d objects\n", (int)req.clusters_indices.size());
      classifier_srv_definitions::classify srv;
      srv.request.cloud = req.cloud;
      srv.request.clusters_indices = req.clusters_indices;
      ros::ServiceClient client = n_.serviceClient<classifier_srv_definitions::classify>("/classify");
      if(client.call(srv))
      {
        for(size_t i = 0; i < srv.response.class_results.size(); i++)
        {
          squirrel_object_perception_msgs::Classification class_tmp;
          for(size_t j = 0; j < srv.response.class_results[i].class_type.size(); j++)
          {
            std::cout << srv.response.class_results[i].class_type[j] << " with confidence " <<
              srv.response.class_results[i].confidence[j] << std::endl;
            //std_msgs::String str_tmp;
            //str_tmp.data = srv.response.class_results[i].class_type[j];
            class_tmp.class_type.push_back(srv.response.class_results[i].class_type[j]);
            class_tmp.confidence.push_back(srv.response.class_results[i].confidence[j]);
            // from Tim Patten: this is present in squirrel_object_perception_msgs::Classify,
            // but not in classifier_srv_definitions::classify:
            // std_msgs::String pose_tmp;
            // pose_tmp.data = poses_[kk];
            // class_tmp.pose.push_back(pose_tmp);
          }
          // NOTE: these are present in classifier_srv_definitions::classify, but not in 
          // squirrel_object_perception_msgs::Classify (because these are not actually
          // the result of classification). We simply ignore these.
          // geometry_msgs/Point32[] centroid
          // object_perception_msgs/BBox[] bbox
          // sensor_msgs/PointCloud2[] cloud
          response.class_results.push_back(class_tmp);
        }
        return true;
      }
      else
      {
        ROS_ERROR("failed to call v4r classification service");
        return false;
      }
    }

  public:
    Classifier()
    : n_("~")
    {
    }

    void initialize()
    {
      classify_service_ = n_.advertiseService("/squirrel_classify", &Classifier::classify, this);
      ROS_INFO("%s: Ready to receive service calls.", ros::this_node::getName().c_str());
    }
};

int main (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_object_classifier");
  Classifier m;
  m.initialize();
  ros::spin();

  return 0;
}
