#ifndef CHECKGRASPSUCCESS
#define CHECKGRASPSUCCESS

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <vector>

#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_view_controller_msgs/LookAtPanTilt.h>
#include <squirrel_view_controller_msgs/LookAtPosition.h>
#include <squirrel_object_perception_msgs/FindDynamicObjects.h>
#include <squirrel_object_perception_msgs/CheckGraspSuccess.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>


class CheckGraspSuccess {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer check_grasp_server;
    double max_pose_dist; //in meters
    double max_overlap; //in percentage
    double called_cam_service;
    sensor_msgs::PointCloud2 scene;
    tf::TransformListener tf_listener;

    bool checkGraspCB(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response);
    bool useOctomap(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response);
    bool useRecognizer(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response);
    bool useSegmentation(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response);
    void getPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    double computeDist(geometry_msgs::Pose obj1, geometry_msgs::Pose obj2);
    double doIntersect(double distance, double diam1, double diam2);
    bool moveCameraAtPosition(std::string frame_id, geometry_msgs::Pose pose);

public:
    CheckGraspSuccess(ros::NodeHandle *nodehandle);
    ~CheckGraspSuccess();

    void initialize (int argc, char ** argv);

};

#endif
