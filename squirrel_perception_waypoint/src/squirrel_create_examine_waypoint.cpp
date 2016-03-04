#include "squirrel_create_examine_waypoint.h"

CreateExamineWaypoint::CreateExamineWaypoint() {

}

CreateExamineWaypoint::~CreateExamineWaypoint() {
    if (n_)
        delete n_;
}

void CreateExamineWaypoint::initialize(int argc, char **argv) {
    ros::init (argc, argv, "squirrel_perception_waypoints_server");
    n_ = new ros::NodeHandle ("~");

    examineWaypointsServer = n_->advertiseService ("/squirrel_perception_examine_waypoint", &CreateExamineWaypoint::createExamineWaypoints, this);

    markerPublisher = n_->advertise<visualization_msgs::MarkerArray>("marker_perc_waypoints", 0);
    posePublisher = n_->advertise<geometry_msgs::PoseArray>("pose_perc_waypoints", 0);

    ROS_INFO ("Ready to get service calls...");
    ros::spin ();
}

//bool CreateExamineWaypoint::classifyWaypoint(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
//    //TODO
//    //Tims code?
//}

bool CreateExamineWaypoint::createExamineWaypoints(squirrel_waypoint_msgs::ExamineWaypointRequest &req, squirrel_waypoint_msgs::ExamineWaypointResponse &resp) {
    //circle around lump
    tf::Transform tf;
    geometry_msgs::PoseStamped lump_pose = req.object_pose;

    float angle = 2*M_PI/(float)nr_of_waypoints;

    for(int i=0;i<nr_of_waypoints;i++)
    {
        geometry_msgs::PoseWithCovarianceStamped waypoint;
        waypoint.header = lump_pose.header;

        /*
         *  float x = radius*sin(angle*i)+width/2;
         *   float y = radius*cos(angle*i)+height/2;
        */
        waypoint.pose.pose.position.x = lump_pose.pose.position.x + (distance_to_lump + req.bounding_cylinder.diameter/2)*sin(angle*i);
        waypoint.pose.pose.position.y = lump_pose.pose.position.y + (distance_to_lump + req.bounding_cylinder.diameter/2)*cos(angle*i);
        waypoint.pose.pose.position.z = 0.0;

        float yaw_angle;
        float x = lump_pose.pose.position.x - waypoint.pose.pose.position.x;
        float y = lump_pose.pose.position.y - waypoint.pose.pose.position.y;
        yaw_angle = std::atan2(y,x);

//        std::cout << "x_delta: " << x << std::endl;
//        std::cout << "y_delta: " << y << std::endl;
//        std::cout << "Yaw Angle: " << yaw_angle << std::endl;

        tf::Quaternion q;
        q.setEuler(0,0,yaw_angle);

        waypoint.pose.pose.orientation.x = q.getX();
        waypoint.pose.pose.orientation.y = q.getY();
        waypoint.pose.pose.orientation.z = q.getZ();
        waypoint.pose.pose.orientation.w = q.getW();

        //fill covariance matrix (for a sharp point, all elements 0 and diagonal close to 0
        //row-major order
        float epsilon = 0.0001;
        for (int c = 0; c < waypoint.pose.covariance.size(); c++) {
            if (c%7 == 0) {
                waypoint.pose.covariance[c] = epsilon;
            }
        }

        resp.weights.push_back(1/nr_of_waypoints);

        resp.poses.push_back(waypoint);
    }
    visualizeWayPoints(req, resp);
    return true;

}

void CreateExamineWaypoint::visualizeWayPoints(squirrel_waypoint_msgs::ExamineWaypointRequest &req, squirrel_waypoint_msgs::ExamineWaypointResponse &resp) {
    geometry_msgs::PoseStamped lump_pose = req.object_pose;
    squirrel_object_perception_msgs::BCylinder bounding_cylinder = req.bounding_cylinder;

    visualization_msgs::MarkerArray marker_array;
    geometry_msgs::PoseArray pose_array;

    size_t counter = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = lump_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "lump_waypoint";
    marker.id = counter; counter++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = lump_pose.pose.position.x;
    marker.pose.position.y = lump_pose.pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = bounding_cylinder.diameter;
    marker.scale.y = bounding_cylinder.diameter;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    marker.text = "lump";
    marker_array.markers.push_back(marker);

    for (int i = 0; i < resp.poses.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = resp.poses[i].header.frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "waypoint";
        marker.id = counter; counter++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose.position.x = resp.poses[i].pose.pose.position.x;
        marker.pose.position.y = resp.poses[i].pose.pose.position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.3;
        marker.color.g = 1.0;
        marker.color.b = 0.3;
        marker_array.markers.push_back(marker);


        pose_array.poses.push_back(resp.poses[i].pose.pose);
    }

    markerPublisher.publish( marker_array );
    pose_array.header = resp.poses[0].header;
    posePublisher.publish(pose_array);
}


//bool CreateExamineWaypoint::exploreAreaWaypoint(squirrel_tests::ExploreWaypointRequest &req, squirrel_tests::ExploreWaypointResponse &resp) {
//    //TODO
//    //use the center of the area and create a circle around it
//    //or create several waypoints to cover the whole area.
//}

int main (int argc, char ** argv)
{
    CreateExamineWaypoint cwp;
    cwp.initialize (argc, argv);

    return 0;
}
