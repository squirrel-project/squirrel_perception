/**
 * The meta tracker calls the object tracker of lump tracker, depending on
 * whether we have a model for the object to be tracked.
 *
 * @author Michael Zillich
 * @date 2016-02
 */


#ifndef SQUIRREL_META_TRACKER_HPP
#define SQUIRREL_META_TRACKER_HPP

#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <squirrel_object_perception_msgs/StartObjectTracking.h>
#include <squirrel_object_perception_msgs/StopObjectTracking.h>

class MetaTracker
{
private:
  ros::NodeHandle *n_;
  ros::ServiceServer startTrackingService_;
  ros::ServiceServer stopTrackingService_;
  ros::ServiceClient startInstanceClient;
  ros::ServiceClient stopInstanceClient;
  ros::ServiceClient startLumpClient;
  ros::ServiceClient stopLumpClient;
  // Scene database
  mongodb_store::MessageStoreProxy *messageStore;
  bool startedTracking;
  std::string trackedObjectId;
  std::string trackedObjectClass;

  bool startTracking(squirrel_object_perception_msgs::StartObjectTracking::Request &req, squirrel_object_perception_msgs::StartObjectTracking::Response &response);
  bool stopTracking(squirrel_object_perception_msgs::StopObjectTracking::Request &req, squirrel_object_perception_msgs::StopObjectTracking::Response &response);

public:
  MetaTracker();
  ~MetaTracker();

  void initialize(int argc, char ** argv);
};

#endif

