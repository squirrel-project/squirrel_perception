/**
 * The meta tracker calls the object tracker of lump tracker, depending on
 * whether we have a model for the object to be tracked.
 *
 * @author Michael Zillich
 * @date 2016-02
 */

#include <squirrel_object_perception_msgs/StartInstanceTracking.h>
#include <squirrel_object_perception_msgs/StopInstanceTracking.h>
#include <squirrel_object_perception_msgs/StartLumpTracking.h>
#include <squirrel_object_perception_msgs/StopLumpTracking.h>
#include <squirrel_tracking/meta_tracker.hpp>

using namespace std;

MetaTracker::MetaTracker()
{
  n_ = 0;
  messageStore = 0;
  startedTracking = false;
}

MetaTracker::~MetaTracker()
{
  delete messageStore;
  delete n_;
}

void MetaTracker::initialize(int argc, char ** argv)
{
  ROS_INFO("initialize");
  n_ = new ros::NodeHandle("~");
  messageStore = new mongodb_store::MessageStoreProxy(*n_);
  startTrackingService_ = n_->advertiseService("/squirrel_start_object_tracking", &MetaTracker::startTracking, this);
  stopTrackingService_ = n_->advertiseService("/squirrel_stop_object_tracking", &MetaTracker::stopTracking, this);
  startInstanceClient = n_->serviceClient<squirrel_object_perception_msgs::StartInstanceTracking>("/squirrel_start_instance_tracking");
  stopInstanceClient = n_->serviceClient<squirrel_object_perception_msgs::StopInstanceTracking>("/squirrel_stop_instance_tracking");
  startLumpClient = n_->serviceClient<squirrel_object_perception_msgs::StartLumpTracking>("/squirrel_start_lump_tracking");
  stopLumpClient = n_->serviceClient<squirrel_object_perception_msgs::StopLumpTracking>("/squirrel_stop_lump_tracking");

  ROS_INFO("MetaTracker: Ready to receive service calls.");

  ros::spin();
}

bool MetaTracker::startTracking(squirrel_object_perception_msgs::StartObjectTracking::Request &req, squirrel_object_perception_msgs::StartObjectTracking::Response &response)
{
  bool ret = false;

  if(!startedTracking)
  {
    // Note that the object_id is just a unique identifier. What the tracker needs is essentially
    // the file name of the model to be loaded. This, in scene database terminology, is the object class.
    // So, first we have to get the class from the scene database.
    trackedObjectClass = "";
    trackedObjectId = req.object_id.data;
    std::vector< boost::shared_ptr<std_msgs::String> > results;
    if(messageStore->queryNamed<std_msgs::String>(req.object_id.data, results))
    {
      if(results.size() == 1)
        trackedObjectClass = results[0]->data;
      else
        ROS_ERROR("MetaTracker::startTracking: multiple objects with same ID '%s'", trackedObjectId.c_str());
    }
    else
    {
      ROS_INFO("MetaTracker::startTracking: have no class/model for object '%s', using default tracker", trackedObjectId.c_str());
      trackedObjectClass = "unknown";
    }

    if(!trackedObjectClass.empty())
    {
      if(trackedObjectClass == "unknown")
      {
        ROS_INFO("MetaTracker: starting lump tracker for object '%s'", trackedObjectId.c_str());
        squirrel_object_perception_msgs::StartLumpTracking srv;
        if(startLumpClient.call(srv))
          startedTracking = true;
        else
          ROS_ERROR("MetaTrackere::startTracking: Failed to call lump tracking service");
      }
      else
      {
        ROS_INFO("MetaTracker: starting instance tracker '%s' for object '%s'", trackedObjectClass.c_str(), trackedObjectId.c_str());
        squirrel_object_perception_msgs::StartInstanceTracking srv;
        if(startInstanceClient.call(srv))
          startedTracking = true;
        else
          ROS_ERROR("MetaTrackere::startTracking: Failed to call instance tracking service");
      }
    }
    ret = startedTracking;
  }
  else
  {
    ROS_ERROR("MetaTracker::startTracking: I am already tracking an object, can only do one at a time.");
  }
  return ret;
}

bool MetaTracker::stopTracking(squirrel_object_perception_msgs::StopObjectTracking::Request &req, squirrel_object_perception_msgs::StopObjectTracking::Response &response)
{
  if(startedTracking)
  {
    if(trackedObjectClass == "unknown")
    {
      squirrel_object_perception_msgs::StopLumpTracking srv;
      if(!stopLumpClient.call(srv))
        ROS_ERROR("MetaTracker: Failed to call lump tracking service");
    }
    else
    {
      squirrel_object_perception_msgs::StopInstanceTracking srv;
      if(!stopInstanceClient.call(srv))
        ROS_ERROR("MetaTracker: Failed to call instance tracking service");
    }
    trackedObjectId = "";
    trackedObjectClass = "";
    startedTracking = false;
    ROS_INFO("MetaTracker::stopTracking: stopped");
  }
  else
  {
    ROS_ERROR("MetaTracker::stopTracking: currently not tracking an object");
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "meta_tracker");
  MetaTracker tracker;
  tracker.initialize(argc, argv);

  return 0;
}
