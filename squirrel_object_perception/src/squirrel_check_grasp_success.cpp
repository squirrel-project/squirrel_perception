#include "squirrel_check_grasp_success.h"

CheckGraspSuccess::CheckGraspSuccess(ros::NodeHandle* nodehandle):nh_(*nodehandle) {
    called_cam_service = false;
}

CheckGraspSuccess::~CheckGraspSuccess() {

}

void CheckGraspSuccess::initialize(int argc, char **argv) {
    check_grasp_server = nh_.advertiseService("/squirrel_check_grasp_success", &CheckGraspSuccess::checkGraspCB, this);
    if(!nh_.getParam("maxPoseDist", max_pose_dist)) {
        max_pose_dist = 0.1;
    }
    if(!nh_.getParam("maxOverlap", max_overlap)) {
        max_overlap = 0.3;
    }
}

bool CheckGraspSuccess::checkGraspCB (squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response) {
    if (request.check_type == 0) {          //octomap
        useOctomap(request, response);
    } else if (request.check_type == 1) {   //segmentation
        useSegmentation(request, response);
    } else if (request.check_type == 2) {   //recognition
        useRecognizer(request, response);
    } else {
        ROS_INFO("The type %d of checking the grasp success is not provided. Choose 0(octomap), 1(segmentation) or 2(recognition)", request.check_type);
        return false;
    }
}

bool CheckGraspSuccess::moveCameraAtPosition(std::string frame_id, geometry_msgs::Pose pose) {
    ros::ServiceClient client = nh_.serviceClient<squirrel_view_controller_msgs::LookAtPosition>("/squirrel_view_controller/look_at_position");
    squirrel_view_controller_msgs::LookAtPosition srv;
    srv.request.target.header.frame_id= frame_id;
    srv.request.target.pose.position.x = pose.position.x;
    srv.request.target.pose.position.y = pose.position.y;
    srv.request.target.pose.position.z = pose.position.z;
    srv.request.target.pose.orientation.x = 0.0;
    srv.request.target.pose.orientation.y = 0.0;
    srv.request.target.pose.orientation.z = 0.0;
    srv.request.target.pose.orientation.w = 1.0;
    if(client.call(srv)) {
        ROS_INFO("Moved camera to position (%f, %f, %f)", pose.position.x,pose.position.y, pose.position.z);
        return true;
    } else {
        ROS_ERROR("Did NOT move camera to position(%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
        return false;
    }
}

bool CheckGraspSuccess::useOctomap(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response) {
    //move camera to the pose of the object
    if (!moveCameraAtPosition(request.object.header.frame_id, request.object.pose)) {
        return false;
    }
    sleep(2); //give the octomap time to update

    ros::ServiceClient find_lumps_client = nh_.serviceClient<squirrel_object_perception_msgs::FindDynamicObjects>("/squirrel_find_dynamic_objects");
    squirrel_object_perception_msgs::FindDynamicObjects fdSrv;
    geometry_msgs::Point min, max;
    min.x = min.y = min.z = max.x = max.y = max.z = 0;
    fdSrv.request.max = max;
    fdSrv.request.min = min;
    if (!find_lumps_client.call(fdSrv)) {
        ROS_ERROR("Could not call service squirrel_find_dynamic_objects");
        return false;
    }
    //Check removed lumps if mongoDB is maintained
    if (!fdSrv.response.database_empty) {
        std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = fdSrv.response.dynamic_objects_removed.begin();
        for (; ci != fdSrv.response.dynamic_objects_removed.end(); ++ci) {
            squirrel_object_perception_msgs::SceneObject so = (*ci);
            if (request.object.id != "") {
                if (so.id == request.object.id) {   //check for the same id
                    response.success = true;
                    return true;
                } else if(request.object.bounding_cylinder.diameter == 0) { //check for same pose
                    double dist = computeDist(so.pose, request.object.pose);
                    if (dist < max_pose_dist) {
                        ROS_INFO("Found removed lump with a distance of %f. Therefore grasping DID succeed!", dist);
                        response.success = true;
                        return true;
                    } else {
                        response.success = false;
                    }
                } else {   //check for bounding box intersection
                    double dist = computeDist(so.pose, request.object.pose);
                    double intersectionPerc = doIntersect(dist, so.bounding_cylinder.diameter, request.object.bounding_cylinder.diameter);
                    if (intersectionPerc < max_overlap) {
                        ROS_INFO("Found removed lump with a bounding cylinder intersection of %f. Therefore grasping DID succeed!", intersectionPerc);
                        response.success = true;
                        return true;
                    } else {
                        response.success = false;
                    }
                }
            }
        }
        ROS_INFO("Did NOT find a removed lump that corresponds to the given object. Therefore grasping did NOT succeed!");
        return true;
    }

    //Otherwise check added lumps
    else {
        std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = fdSrv.response.dynamic_objects_added.begin();
        for (; ci != fdSrv.response.dynamic_objects_added.end(); ++ci) {
            squirrel_object_perception_msgs::SceneObject so = (*ci);
            if(request.object.bounding_cylinder.diameter == 0) { //check for same pose
                double dist = computeDist(so.pose, request.object.pose);
                if (dist > max_pose_dist) {
                    response.success = true;
                } else {
                    ROS_INFO("Found added lump with a distance of %f. Therefore grasping DID NOT succeed!", dist);
                    response.success = false;
                    return true;
                }
            } else {   //check for bounding box intersection
                double dist = computeDist(so.pose, request.object.pose);
                double intersectionPerc = doIntersect(dist, so.bounding_cylinder.diameter, request.object.bounding_cylinder.diameter);
                if (intersectionPerc > max_overlap) {
                    response.success = true;
                } else {
                    ROS_INFO("Found added lump with a bounding cylinder intersection of %f. Therefore grasping DID NOT succeed!", intersectionPerc);
                    response.success = false;
                    return true;
                }
            }
        }
        ROS_INFO("Did NOT find an added lump that corresponds to the given object. Therefore grasping DID succeed!");
        return true;
    }
}

void CheckGraspSuccess::getPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!called_cam_service) { //this is a little hack, to get the second point cloud without shutting down the subscriber in between
        sleep(1);
        std::cout << "Waiting for point cloud." << std::endl;
        scene = *ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_);
        std::cout << "Received point cloud." << std::endl;
        called_cam_service = true;
    }
}

bool CheckGraspSuccess::useSegmentation(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response) {
    ros::ServiceClient segm_init_client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
    ros::ServiceClient segm_once_client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");

    //move camera to the pose of the object
    if (!moveCameraAtPosition(request.object.header.frame_id, request.object.pose)) {
        return false;
    }

    //get point cloud
    //do the hacky trick to get second cloud of subscriber (otherwise wrongly colored cloud)
    ROS_INFO("Try to subscribe to point cloud");
    ros::Subscriber sub_pc = nh_.subscribe ("/kinect/depth_registered/points", 1, &CheckGraspSuccess::getPointCloud, this);
    ros::Rate loop_rate (1);
    // poll until we did receive a point cloud
    while(!called_cam_service)
    {
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    called_cam_service = false;
    sub_pc.shutdown();

    //Initilize segmentation
    squirrel_object_perception_msgs::SegmentInit srv_init;
    srv_init.request.cloud = scene;
    if (segm_init_client.call(srv_init)) {
        ROS_INFO("Called segmentation init");
    }
    else {
        ROS_ERROR("Failed to call segmentation init!");
        return false;
    }

    //Get closest object (attention it is in kinect frame!)
    if (!segm_once_client.exists()) {
        ROS_ERROR("Could not call service segmentation once.");
        return false;
    }
    squirrel_object_perception_msgs::SegmentOnce srv_once;
    while (segm_once_client.call(srv_once)) {
        ROS_INFO("Called segmentation once");
        squirrel_object_perception_msgs::SegmentOnce::Response segm_result = srv_once.response;
        //transform pose to map frame
        try
        {
            tf_listener.waitForTransform(segm_result.poses[0].header.frame_id, "/map", ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose("/map", segm_result.poses[0], segm_result.poses[0]);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }

        //check now the positions
        double dist = computeDist(segm_result.poses[0].pose, request.object.pose);
        if (dist < max_pose_dist) {
            ROS_INFO("Found segmentation cluster with a distance of %f. Therefore grasping did NOT succeed!", dist);
            response.success = false;
            return true;
        }
    }
    //this means not a single object was segmented or the segmented objects were not close
    ROS_INFO("Could not find an object close by.");
    response.success = true;
    return true;
}


bool CheckGraspSuccess::useRecognizer(squirrel_object_perception_msgs::CheckGraspSuccessRequest &request, squirrel_object_perception_msgs::CheckGraspSuccessResponse &response) {
    actionlib::SimpleActionClient<squirrel_object_perception_msgs::LookForObjectsAction> recognition_client("squirrel_recognize_objects", true);
    squirrel_object_perception_msgs::LookForObjectsGoal recognition_goal;

    while(!recognition_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the squirrel_recognize_objects action server to come up");
    }
    if (!recognition_client.isServerConnected()) {
        ROS_ERROR("Could not connect to squirrel_recognize_objects action server.");
        return false;
    }
    recognition_goal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
    //look at pose where the grasped object was
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = request.object.header;
    poseStamped.pose = request.object.pose;
    recognition_goal.look_at_pose = poseStamped;
    recognition_client.sendGoal(recognition_goal);
    recognition_client.waitForResult(ros::Duration(60.0));
    if (recognition_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("Failed to call squirrel_recognize_objects. %s", recognition_client.getResult()->result_status.c_str());
        return false;
    }
    std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = recognition_client.getResult()->objects_added.begin();
    for (; ci != recognition_client.getResult()->objects_added.end(); ++ci) {
        squirrel_object_perception_msgs::SceneObject so = (*ci);
        if (request.object.category != "") {
            if (so.category == request.object.category) {
                //There is an object of the same category. Should we check position as well?
                ROS_INFO("Found an object of the same category! Grasping failed!");
                response.success = false;
                return true;
            } else {
                //No category was given in the reqeust.
                //No bounding cylinder set. Check if a recognized object is close.
                if(so.bounding_cylinder.diameter == 0 || request.object.bounding_cylinder.diameter == 0) {
                    double dist = computeDist(so.pose, request.object.pose);
                    if (dist > max_pose_dist) {
                        response.success = true;
                    } else {
                        ROS_INFO("Recognized object with a distance of %f. Therefore grasping did NOT succeed!", dist);
                        response.success = false;
                        return true;
                    }
                } else {
                    double dist = computeDist(so.pose, request.object.pose);
                    double intersectionPerc = doIntersect(dist, so.bounding_cylinder.diameter, request.object.bounding_cylinder.diameter);
                    if (intersectionPerc > max_overlap) {
                        response.success = true;
                    } else {
                        ROS_INFO("Recognized object with a bounding cylinder intersection of %f. Therefore grasping did NOT succeed!", intersectionPerc);
                        response.success = false;
                        return true;
                    }
                }
            }
        }
    }
    ROS_INFO("Did not recognize any object that is close by. Therefore grasping succeeded!");
    return true;
}

double CheckGraspSuccess::computeDist(geometry_msgs::Pose obj1, geometry_msgs::Pose obj2) {
    double dist = std::sqrt((obj1.position.x-obj2.position.x)*(obj1.position.x-obj2.position.x) + (obj1.position.y-obj2.position.y)*(obj1.position.y-obj2.position.y));
    return dist;
}

//returns the amount of intersection in percent (x% of the area of c1 is covered by c2)
//http://mathworld.wolfram.com/Circle-CircleIntersection.html
double CheckGraspSuccess::doIntersect(double distance, double diam1, double diam2) {
    double intersectionPerc = 0.0;

    //not intersecting
    if (distance >= diam1+diam2) {
        intersectionPerc = 0.0;
        return intersectionPerc;
    }

    //one of the circles covers the other circle completely
    if (distance <= std::abs(diam1-diam2)) {
        intersectionPerc = 1.0;
        return intersectionPerc;
    }

    double overlappingArea;
    double distance2 = distance*distance;
    overlappingArea = diam1*diam1 * acos((diam1*diam1 - diam2*diam2 + distance2) / (2*distance*diam1)) +
            diam2*diam2 * acos((diam2*diam2 - diam1*diam1 + distance2) / (2*distance*diam2)) -
            0.5 * std::sqrt((-distance+diam1+diam2) * (distance + diam1-diam2) * (distance-diam1+diam2) * (distance+diam1+diam2));

    double areaC1 = diam1*diam1 * M_PI;
    double areaC2 = diam2*diam2 * M_PI;
    intersectionPerc = std::max(overlappingArea/areaC1, overlappingArea/areaC2);

    return intersectionPerc;
}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "squirrel_check_grasp_success");

    ros::NodeHandle n ("~");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    CheckGraspSuccess checkGrasp(&n);
    checkGrasp.initialize(argc, argv);
    ROS_INFO("%s: ready...", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
