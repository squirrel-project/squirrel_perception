#include "post_process_static_octomap.h"

PostProcessOctomap::PostProcessOctomap(ros::NodeHandle* nodehandle):n_(*nodehandle) {

}

PostProcessOctomap::~PostProcessOctomap() {

}

void PostProcessOctomap::initialize(int argc, char **argv) {
    std::string octomap_path;
    std::string final_octomap_path;
    n_.getParam("octomap_path", octomap_path);
    n_.getParam("final_octomap_path", final_octomap_path);

    ROS_INFO("Initialized post_process_static_octomap node");
    octomap_lib.readOctoMapFromFile(octomap_path, staticMap , ends_with(octomap_path, "bt"));
    staticMap->expand();
    std::cout << "Input map has " << octomap_lib.getNumberOccupiedLeafNodes(staticMap) << "nodes" << std::endl;

    //remove
    octomap_lib.removeFloor(staticMap);
    std::cout << "Successfully removed floor" << std::endl;

    //dilate
    octomap::OcTree final_octomap = octomap_lib.dilateOctomap(staticMap);
    std::cout << "Final map has " << octomap_lib.getNumberOccupiedLeafNodes(&final_octomap) << "nodes" << std::endl;
    octomap_lib.writeOctomap(final_octomap , final_octomap_path, ends_with(final_octomap_path, "bt"));

    ROS_INFO("Wrote octomap to file. Done!");

}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "post_process_static_octomap");
    ros::NodeHandle n ("~");

    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    PostProcessOctomap ppo(&n);
    ppo.initialize(argc, argv);

    return 0;
}


