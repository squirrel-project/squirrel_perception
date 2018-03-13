#include "squirrel_fuse_lumps_static_octomap.h"

FuseLumpsIntoOctomap::FuseLumpsIntoOctomap(ros::NodeHandle* nodehandle):n_(*nodehandle) {
    octomap_is_binary = false;
}

FuseLumpsIntoOctomap::~FuseLumpsIntoOctomap() {

}

void FuseLumpsIntoOctomap::initialize(int argc, char **argv) {
    std::cout << "init: " << std::endl;
    create_octomap_with_lumps_srv = n_.advertiseService("/squirrel_create_octomap_with_lumps", &FuseLumpsIntoOctomap::createOctomapWithLumpsCB, this);
    receive_octomap_with_lumps_srv = n_.advertiseService("/squirrel_receive_octomap_with_lumps", &FuseLumpsIntoOctomap::receiveOctomapWithLumpsCB, this);

    octomap_pub = n_.advertise<octomap_msgs::Octomap>("fused_octomap", 1, true);

    ros::spin();
}

bool FuseLumpsIntoOctomap::createOctomapWithLumpsCB (squirrel_object_perception_msgs::CreateOctomapWithLumpsRequest &request, squirrel_object_perception_msgs::CreateOctomapWithLumpsResponse &response) {
    std::string octomap_path;
    n_.getParam("octomap_path", octomap_path);

    std::cout << "static octomap path: " << octomap_path << std::endl;

    if (ends_with(octomap_path, "bt")) {
        octomap_is_binary = true;
    }
    octomap_lib.readOctoMapFromFile(octomap_path, fused_map , octomap_is_binary);
    if (fused_map->getNumLeafNodes() == 0) {
        ROS_WARN("The static octomap is empty! You probably try to use the default octomap.");
    }

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

        //we ignore the height in favor of grasping and just mark one layer of voxels as occupied
        double z = leaf_size + leaf_size/2;
        for (double ix = pose_stamped.pose.position.x - nr_voxels_radius*leaf_size; ix <= pose_stamped.pose.position.x - nr_voxels_radius*leaf_size; ix += leaf_size) {
            for (double iy = pose_stamped.pose.position.y - nr_voxels_radius*leaf_size; iy <= pose_stamped.pose.position.y - nr_voxels_radius*leaf_size; iy += leaf_size) {
                octomap::OcTreeNode* node = fused_map->search(ix, iy, z);
                if (node==NULL) {
                    fused_map->setNodeValue(ix,iy,z,octomap::logodds(fused_map->getClampingThresMax()));
                } else if (!fused_map->isNodeOccupied(node)) {
                    node->setLogOdds(octomap::logodds(fused_map->getClampingThresMax()));
                }
            }
        }
    }

    return true;
}

bool FuseLumpsIntoOctomap::receiveOctomapWithLumpsCB (squirrel_object_perception_msgs::ReceiveOctomapWithLumpsRequest &request, squirrel_object_perception_msgs::ReceiveOctomapWithLumpsResponse &response) {
    if (fused_map->getNumLeafNodes() > 0) {
        octomap_msgs::Octomap map_msg;
        if (octomap_is_binary) {
            octomap_msgs::binaryMapToMsg(*fused_map, map_msg);
        } else {
            octomap_msgs::fullMapToMsg(*fused_map, map_msg);
        }
        response.fusedOctomap = map_msg;
        octomap_pub.publish(map_msg);
        return true;
    }
    ROS_ERROR("The octomap is empty. Please call the service CreateOctomapWithLumpsResponse first!");
    return false;
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


