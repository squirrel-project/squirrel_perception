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

    std::string fileOctomap;
    n_->getParam("static_octomap_path", fileOctomap);
    n_->getParam("nr_waypoints", nr_of_waypoints);
    n_->getParam("distance_to_lump", distance_to_lump);

    if (!readOctomapFromFile(fileOctomap)) {
        ROS_ERROR("TUW: Could not read octomap for waypoint generation");
    }

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

    // compute waypoints
    std::vector<wpData> wp;
    int numWaypoints = computeWaypoints(oct, lump_pose,
                                        req.bounding_cylinder.diameter/2,
                                        req.bounding_cylinder.height, wp);

    if(!numWaypoints)
        return false;

    for(int i = 0; i < numWaypoints; i++)
    {
        geometry_msgs::PoseWithCovarianceStamped waypoint;
        waypoint.header = lump_pose.header;

        /*
         *  float x = radius*sin(angle*i)+width/2;
         *   float y = radius*cos(angle*i)+height/2;
        */
        /*
       waypoint.pose.pose.position.x = lump_pose.pose.position.x + (distance_to_lump + req.bounding_cylinder.diameter/2)*sin(angle*i);
       waypoint.pose.pose.position.y = lump_pose.pose.position.y + (distance_to_lump + req.bounding_cylinder.diameter/2)*cos(angle*i);
       waypoint.pose.pose.position.z = 0.0;
    */

        waypoint.pose.pose.position.x = wp[i].pos.x();
        waypoint.pose.pose.position.y = wp[i].pos.y();
        waypoint.pose.pose.position.z = 0.0f;

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

        resp.weights.push_back(1.0f/numWaypoints);

        resp.poses.push_back(waypoint);
        resp.tiltAngle.push_back(wp[i].tiltLB);
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

/* tests if any leaf in a given bounding box is occupied
 *
 * parameters: ptr of OcTree instance, spanning points of bounding box
 * returns: true if any leaf is occupied */
bool CreateExamineWaypoint::isBBXOccupied(octomap::OcTree *oct,
                                          octomap::point3d min,
                                          octomap::point3d max) {
    for(octomap::OcTree::leaf_bbx_iterator it = oct->begin_leafs_bbx(min, max);
        it != oct->end_leafs_bbx(); it ++) {
        if(oct->isNodeOccupied(*it))
            return true;
    }

    return false;
}

int CreateExamineWaypoint::computeWaypoints(octomap::OcTree *oct,
                                            geometry_msgs::PoseStamped lump,
                                            float radius, float height,
                                            std::vector<wpData> &wp) {
    float cx = lump.pose.position.x;
    float cy = lump.pose.position.y;
    float cz = lump.pose.position.z;
    float resolution = oct->getResolution();
    bool occupied[24];             // are places around object occupied?
    octomap::point3d wpCoord[24];
    float tiltLB[24];

    // compute maximum allowed down tilt value
    float maxTilt = ((atan(camera_height / (distance_to_lump - robot_base_dia/2)))
                     - vApAngle/2) / maxPanTilt;

    // 15° steps to check where there is free space around the object
    for(int i = 0; i < 24; i ++) {
        float x, y, z;
        float dist;

        /* start out with a distance that allows for complete picture without
       panning in multiples of distance_to_lump and continue as long with
       decreasing distance as criteria are not met (waypoint free and full view
       of object; minimum start distance = 2*distance_to_lump
    */
        for(dist = fmaxf(ceil((radius/sinf(hApAngle/2)-radius+robot_base_dia/2) /
                              distance_to_lump), 2.0) * distance_to_lump;
            dist >= distance_to_lump; dist -= distance_to_lump) {
            // compute waypoint and camera point
            x = cx + (radius + dist) * cosf(i * M_PI/12.0f);
            y = cy + (radius + dist) * sinf(i * M_PI/12.0f);
            wpCoord[i] = octomap::point3d(x, y, 0.0);

            octomap::point3d camOrig(x - robot_base_dia/2 * cosf(i * M_PI/12.0f),
                                     y - robot_base_dia/2 * sinf(i * M_PI/12.0f),
                                     camera_height);
            octomap::point3d camDir(-cosf(i * M_PI/12.0f),
                                    -sinf(i * M_PI/12.0f),
                                    -camera_height / (dist-robot_base_dia/2));

            // calculate camera angles
            float alpha, beta;

            if(cz >= camera_height) {
                alpha = atan((cz - camera_height)/(dist + 2*radius - robot_base_dia/2));
                beta = atan((cz + height - camera_height)/(dist - robot_base_dia/2));
            }
            else {
                alpha = atan((cz - camera_height)/(dist - robot_base_dia/2));
                if(cz + height < camera_height)
                    beta = atan((cz + height - camera_height) /
                                (dist + 2*radius - robot_base_dia/2));
                else
                    beta = atan((cz + height - camera_height)/(dist - robot_base_dia/2));
            }

            // if whole object fits into image -> focus it
            if(beta - alpha <= vApAngle) {
                tiltLB[i] = fmin(-(alpha+beta)/2 / maxPanTilt, maxTilt);
            }
            else { // if not
                break;
            }

            // compute one corner point of bounding box
            x = wpCoord[i].x() - robot_base_dia/2;
            y = wpCoord[i].y() - robot_base_dia/2;
            z = 0.01 + resolution;  // ensure that floor doesn't penetrate bbx
            octomap::point3d min(x,y,z);

            // now the other corner
            x = wpCoord[i].x() + robot_base_dia/2;
            y = wpCoord[i].y() + robot_base_dia/2;
            z = robot_height;
            octomap::point3d max(x,y,z);

            // if bounding box is not occupied check whether something blocks the view
            occupied[i] = true;

            if( !isBBXOccupied(oct, min, max) ) {
                // check whether a cast ray to the lower edge of the object hits sth.
                octomap::point3d end;
                oct->castRay(camOrig, camDir, end, true);

                /* in case the ray hit the ground -> nothing blocks view
       -> not occupied and continue with next in loop */
                if(end.z() < resolution) {
                    occupied[i] = false;
                    break;
                }
            }
        }
    }

    // check whether neighbouring free waypoints are reachable (isolated free wp)
    for(int i = 0; i < 24; i ++) {
        if(occupied[(i+1)%24] && occupied[(i+23)%24])
            occupied[i] = true;
    }

    // select a certain number of waypoints (ideally nr_of_waypoints)
    int nwp = 0;      // number of waypoints around object
    int wpIdx = -1;   // current waypoint index
    int startIdx;

    for(int i = 0; i < nr_of_waypoints; i ++) {
        if(!i) {  // i == 0 -> look for first free waypoint index
            for(int j = 0; j < 24; j ++) {
                if(!occupied[j]) {
                    wpIdx = j;
                    startIdx = j;
                    break;
                }
            }

            // if no free waypoint around object return 0
            if(wpIdx == -1) return 0;
        }
        else {
            // try the next waypoint 360/nr_of_waypoints degrees further
            // if occupied try the direct neighbours (+-15 degrees)
            wpIdx = (startIdx + i * 24 / nr_of_waypoints) % 24;
            if(occupied[wpIdx]) {
                if(occupied[wpIdx = (wpIdx+1)%24]) {
                    if(occupied[wpIdx = (wpIdx+22)%24])
                        wpIdx = -1;     /* for this waypoint there is no solution ->
                   decrease number of waypoints around object */
                }
            }
        }

        // add selected waypoint to array
        if(wpIdx != -1) {
            wpData out;

            out.pos = wpCoord[wpIdx];
            out.tiltLB = tiltLB[wpIdx];

            wp.push_back(out);
            nwp ++;
        }
    }

    return nwp;
}

bool CreateExamineWaypoint::readOctomapFromFile(std::string fileOctomap) {

    // load Octomap
    if(fileOctomap.find(".bt") != std::string::npos)
        // OcTree in binary format
        oct = new octomap::OcTree(fileOctomap);
    else {
        octomap::AbstractOcTree *aot = octomap::AbstractOcTree::read(fileOctomap);
        if(aot)
            oct = dynamic_cast<octomap::OcTree*>(aot);
    }

    if(!oct) {
        std::cerr << "OcTree could not be loaded." << std::endl;
        return false;
    }

    if (oct->getNumLeafNodes() == 0) {
        ROS_WARN("The static octomap is empty! You probably try to use the default octomap.");
	return false;
    }

    return true;
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

