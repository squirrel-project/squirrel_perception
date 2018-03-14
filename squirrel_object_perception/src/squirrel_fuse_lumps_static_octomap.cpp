#include "squirrel_fuse_lumps_static_octomap.h"

FuseLumpsIntoOctomap::FuseLumpsIntoOctomap(ros::NodeHandle* nodehandle):n_(*nodehandle) {
    fused_map = NULL;
    octomap_is_binary = false;
}

FuseLumpsIntoOctomap::~FuseLumpsIntoOctomap() {

}

void FuseLumpsIntoOctomap::initialize(int argc, char **argv) {
    create_octomap_with_lumps_srv = n_.advertiseService("/squirrel_create_octomap_with_lumps", &FuseLumpsIntoOctomap::createOctomapWithLumpsCB, this);
    receive_octomap_with_lumps_srv = n_.advertiseService("/squirrel_receive_octomap_with_lumps", &FuseLumpsIntoOctomap::receiveOctomapWithLumpsCB, this);

    octomap_pub = n_.advertise<octomap_msgs::Octomap>("fused_octomap", 1, true);

    ros::spin();
}

bool FuseLumpsIntoOctomap::createOctomapWithLumpsCB (squirrel_object_perception_msgs::CreateOctomapWithLumpsRequest &request, squirrel_object_perception_msgs::CreateOctomapWithLumpsResponse &response) {
    ROS_INFO("Create octomap with lumps was called");
    std::string octomap_path;
    n_.getParam("static_octomap_path", octomap_path);

    if (ends_with(octomap_path, "bt")) {
        octomap_is_binary = true;
    }
    octomap_lib.readOctoMapFromFile(octomap_path, fused_map , octomap_is_binary);
    if (fused_map->getNumLeafNodes() == 0) {
        ROS_WARN("The static octomap is empty! You probably try to use the default octomap.");
    }

    fused_map->expand();

    double leaf_size = fused_map->getNodeSize(16);

    for (int i = 0; i < request.lumps.size(); i++) {
        float diam = request.lumps[i].bounding_cylinder.diameter;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = request.lumps[i].header;
        pose_stamped.pose = request.lumps[i].pose;
        try
        {
            tf_listener.waitForTransform(request.lumps[i].header.frame_id, "/map", ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose("/map", pose_stamped, pose_stamped);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
            return false;
        }

        int nr_voxels_radius = ceil(diam/leaf_size) / 2;
        double z = leaf_size + leaf_size/2;

        octomap::point3d min, max;
        min.x() = pose_stamped.pose.position.x - nr_voxels_radius*leaf_size;
        max.x() = pose_stamped.pose.position.x + nr_voxels_radius*leaf_size;
        min.y() = pose_stamped.pose.position.y - nr_voxels_radius*leaf_size;
        max.y() = pose_stamped.pose.position.y + nr_voxels_radius*leaf_size;
        min.z() = z;
        max.z() = z;
        
        for(octomap::OcTree::leaf_bbx_iterator it = fused_map->begin_leafs_bbx(min, max); it != fused_map->end_leafs_bbx(); it ++) {
            octomap::OcTreeKey key = it.getKey();
            octomap::OcTreeNode* node = fused_map->search(key);
            if (node==NULL) {
                fused_map->setNodeValue(key,octomap::logodds(fused_map->getClampingThresMax()));
            } else if (!fused_map->isNodeOccupied(node)) {
                (*it).setLogOdds(octomap::logodds(fused_map->getClampingThresMax()));
            }
        }
    }
    return true;
}

bool FuseLumpsIntoOctomap::receiveOctomapWithLumpsCB (octomap_msgs::GetOctomapRequest &request, octomap_msgs::GetOctomapResponse &response) {
    if (fused_map == NULL || fused_map->getNumLeafNodes() == 0) {
        ROS_WARN("The octomap is empty. Please call the service CreateOctomapWithLumpsResponse first! Now only the static octomap gets returned.");
        squirrel_object_perception_msgs::CreateOctomapWithLumpsRequest request;
        squirrel_object_perception_msgs::CreateOctomapWithLumpsResponse response;
        createOctomapWithLumpsCB(request, response);
        if (fused_map == NULL || fused_map->getNumLeafNodes() == 0) {
            ROS_ERROR("Can not get the static octomap.");
        }   
    }
    octomap_msgs::Octomap map_msg;
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "/map";
    if (octomap_is_binary) {
        octomap_msgs::binaryMapToMsg(*fused_map, map_msg);
    } else {
        octomap_msgs::fullMapToMsg(*fused_map, map_msg);
    }
    response.map = map_msg;
    octomap_pub.publish(map_msg);
    return true;
}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "squirrel_fuse_lumps_static_octomap");
    ros::NodeHandle n ("~");

    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    FuseLumpsIntoOctomap fuse(&n);
    fuse.initialize(argc, argv);

    return 0;
}


