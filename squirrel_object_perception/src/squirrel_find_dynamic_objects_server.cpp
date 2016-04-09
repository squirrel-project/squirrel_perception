#include "squirrel_find_dynamic_objects_server.h"

using namespace std;

RemoveBackground::RemoveBackground() : n_(new ros::NodeHandle("~")), message_store(*n_){
    RemoveBackground::id_cnt_ = 0;
}

RemoveBackground::~RemoveBackground() {
    if (n_)
        delete n_;
}

bool RemoveBackground::removeBackground (squirrel_object_perception_msgs::FindDynamicObjects::Request & request, squirrel_object_perception_msgs::FindDynamicObjects::Response & response) {

    //read the input
    octomap_msgs::OctomapConstPtr current_octomap_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary", *n_, ros::Duration(10));
    setCurrentOctomap(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*current_octomap_msg)));
    //std::string currentOctomapPath = "/home/edith/SQUIRREL/workspace/test5.bt";
    //n_->getParam("current_octomap_path", currentOctomapPath);
    //octomap_lib.readOctoMapFromFile(currentOctomapPath, currentMap , true);

    ROS_INFO("TUW: Current octomap read");

    //delete all visualization markers from previous calls
    for (std::vector<int>::iterator it = vis_marker_ids.begin() ; it != vis_marker_ids.end(); ++it) {
        zyl_marker.id = *it;
        zyl_marker.ns = "cluster_marker";
        zyl_marker.action = visualization_msgs::Marker::DELETE;
        markerPublisher.publish(zyl_marker);
    }
//    zyl_marker.action = 3; //DELETEALL
//    markerPublisher.publish(zyl_marker);

    octomap::OcTree subtractedMapVar = subtractOctomaps();
    if (octomap_lib.getNumberOccupiedLeafNodes(&subtractedMapVar) == 0) {
        ROS_INFO("TUW: Static and current octomap are the same");
        return false;
    }


    octomap::OcTree* subtractedMap = &subtractedMapVar;

    octomap_lib.writeOctomap(subtractedMap, "subtracted.bt", true);

    //remove voxels close to indicated obstacles in the map
    nav_msgs::OccupancyGridConstPtr grid_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", *n_, ros::Duration(10));

    pcl::PointCloud<PointT>::Ptr filtered_cloud = compareOctomapToGrid(subtractedMap, grid_map);
    if (filtered_cloud->size() == 0) {
        ROS_INFO("TUW: Current octomap and occupancy grid are very similar - no lump to detect");
        return false;
    }
    pcl::io::savePCDFileASCII ("cloud_after_map_comp.pcd", *filtered_cloud);


    //pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
    //octomap_lib.octomapToPointcloud(subtractedMap, filtered_cloud);
    //clustering, remove noise and cluster which are not connected to the floor
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters = removeClusters(filtered_cloud);
    //pcl::io::savePCDFileASCII ("cloud_final.pcd", *filtered_cloud);

    //Response added, updated and removed objects
    int cnt = 0;
    squirrel_object_perception_msgs::SceneObject lump;

    std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
    message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);

    //    std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > objects_remove (sceneObjects_results.size());
    //    for (std::size_t i = 0; i < sceneObjects_results.size(); ++i)
    //           objects_remove[i] = boost::make_shared<squirrel_object_perception_msgs::SceneObject>(*sceneObjects_results[i]);

    for (std::size_t i = 0; i < sceneObjects_results.size(); ++i) {
        response.dynamic_objects_removed.push_back(*sceneObjects_results[i]);
    }

    ROS_INFO("Number of objects already in DB: %zu", sceneObjects_results.size());

    ROS_INFO("TUW: Number of lumps identified: %zu", clusters.size());
    for (std::vector<pcl::PointCloud<PointT>::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it)
    {
        PointT min_p, max_p;
        pcl::getMinMax3D(*(*it), min_p, max_p);

        double pose_x = min_p.x + (max_p.x - min_p.x)/2;
        double pose_y = min_p.y + (max_p.y - min_p.y)/2;
        double pose_z = min_p.z + (max_p.z - min_p.z)/2;

        double x_diam = double(max_p.x - min_p.x + octomap_lib.leaf_size);
        double y_diam = double(max_p.y - min_p.y + octomap_lib.leaf_size);
        double diam = std::sqrt(std::pow(x_diam,2) + std::pow(y_diam,2));
//        double x_diam =  double(max_p.x - min_p.x + octomap_lib.leaf_size);
//        double y_diam =  double(max_p.y - min_p.y + octomap_lib.leaf_size);
//        double diam = std::max(x_diam, y_diam);
        double z_diam = double(max_p.z - min_p.z + octomap_lib.leaf_size);

        //double diam = std::max((float)x_diam, (float)y_diam);

        geometry_msgs::Pose pose_db;
        bool is_lump_in_db = false;
        bool is_classified =false;
        BOOST_FOREACH(boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> sceneObject_db, sceneObjects_results) {
            pose_db = (*sceneObject_db).pose;

            //object at the same position
            if((std::abs(pose_db.position.x - pose_x) < POSE_THRESH) && (std::abs(pose_db.position.y - pose_y) < POSE_THRESH)
                    && (std::abs(pose_db.position.z - pose_z) < POSE_THRESH) ) {
                if ((*sceneObject_db).category != "unknown") {
                    response.dynamic_objects_removed.erase(response.dynamic_objects_removed.begin() + cnt);
                    is_lump_in_db = true;
                    is_classified = true;
                    ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) is a classified object", pose_x, pose_y, pose_z);
                }
                //check if the size differs and update it if necessary
                else {
                    squirrel_object_perception_msgs::BCylinder bCylinder = (*sceneObject_db).bounding_cylinder;
                    if ((std::abs(bCylinder.diameter - diam) < 0.0001) && (std::abs(bCylinder.height == z_diam) < 0.0001)) {
                        lump=(*sceneObject_db);
                        is_lump_in_db = true;
                        response.dynamic_objects_removed.erase(response.dynamic_objects_removed.begin() + cnt);
                        ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) is already as unknown object in DB", pose_x, pose_y, pose_z);
                    }
                    else {
                        lump.header.frame_id = "map";
                        lump.header.stamp = ros::Time();
                        lump.pose.position.x = pose_x;
                        lump.pose.position.y = pose_y;
                        lump.pose.position.z = pose_z;
                        lump.pose.orientation.x = 0.0;
                        lump.pose.orientation.y = 0.0;
                        lump.pose.orientation.z = 0.0;
                        lump.pose.orientation.w = 1.0;
                        lump.bounding_cylinder.diameter = diam;
                        //lump.bounding_cylinder.diameter = y_diam;
                        lump.bounding_cylinder.height = z_diam;
                        lump.category="unknown";
                        lump.id= (*sceneObject_db).id;

                        is_lump_in_db = true;
                        response.dynamic_objects_updated.push_back(lump);
                        response.dynamic_objects_removed.erase(response.dynamic_objects_removed.begin() + cnt);

                        ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) got updated in DB", pose_x, pose_y, pose_z);
                    }
                }
            }
        }

        if (!is_lump_in_db) {   //nothing at the position
            lump.header.frame_id = "map";
            lump.header.stamp = ros::Time();
            lump.pose.position.x = pose_x;
            lump.pose.position.y = pose_y;
            lump.pose.position.z = pose_z;
            lump.pose.orientation.x = 0.0;
            lump.pose.orientation.y = 0.0;
            lump.pose.orientation.z = 0.0;
            lump.pose.orientation.w = 1.0;
            lump.bounding_cylinder.diameter = diam;
            //lump.bounding_cylinder.diameter = y_diam;
            lump.bounding_cylinder.height = z_diam;
            lump.category="unknown";
            lump.id=get_unique_object_id();
            response.dynamic_objects_added.push_back(lump);

            //message_store.insert(lump);

            cnt += 1;

            ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) got inserted in DB", pose_x, pose_y, pose_z);
        }


        //creates a marker that can be visualized in rviz
        if (!is_classified) {
            zyl_marker.header.frame_id = "map";
            zyl_marker.header.stamp = ros::Time();
            zyl_marker.ns = "cluster_marker";
            zyl_marker.id = std::atoi(lump.id.substr(4, string::npos).c_str());
            zyl_marker.lifetime = ros::Duration();
            zyl_marker.type = visualization_msgs::Marker::CYLINDER;
            zyl_marker.action = visualization_msgs::Marker::ADD;
            zyl_marker.pose.position.x = pose_x;
            zyl_marker.pose.position.y = pose_y;
            zyl_marker.pose.position.z = pose_z;
            zyl_marker.pose.orientation.x = 0.0;
            zyl_marker.pose.orientation.y = 0.0;
            zyl_marker.pose.orientation.z = 0.0;
            zyl_marker.pose.orientation.w = 1.0;
            zyl_marker.scale.x = diam;
            zyl_marker.scale.y = diam;
            zyl_marker.scale.z = z_diam;
            zyl_marker.color.r = 1.0;
            zyl_marker.color.g = 0.9;
            zyl_marker.color.b = 0.1;
            zyl_marker.color.a = 0.4;

            markerPublisher.publish(zyl_marker);

            vis_marker_ids.push_back(zyl_marker.id);

        }

    }

    ROS_INFO("TUW: Added %zu lumps", response.dynamic_objects_added.size());
    ROS_INFO("TUW: Updated %zu lumps", response.dynamic_objects_updated.size());
    ROS_INFO("TUW: Removed %zu lumps", response.dynamic_objects_removed.size());
    return true;
}

void RemoveBackground::initialize(int argc, char **argv) {

    n_->getParam("static_octomap_path", staticOctomapPath_);
    markerPublisher = n_->advertise<visualization_msgs::Marker>("vis_marker_dynamic_objects", 0);

    setStaticOctomap(staticOctomapPath_);

    if (staticMap->getNumLeafNodes() == 0) {
        ROS_WARN("The static octomap is empty! You probably try to use the default octomap.");
    }

    //insert some objects for testing purposes with corridor_test_edited.bt
    //    squirrel_object_perception_msgs::ObjectToDB test_object;

    //    test_object.id = "id1";
    //    test_object.category = "cat1";
    //    //test_object.cloud = ;
    //    test_object.pose.header.frame_id = "/map";
    //    test_object.pose.header.stamp = ros::Time();
    //    test_object.pose.pose.orientation.x = 0.0;
    //    test_object.pose.pose.orientation.y = 0.0;
    //    test_object.pose.pose.orientation.z = 0.0;
    //    test_object.pose.pose.orientation.w = 1.0;
    //    test_object.pose.pose.position.x = 11.1375;
    //    test_object.pose.pose.position.y = 5.1625;
    //    test_object.pose.pose.position.z = 0.125;

    //    message_store.insert(test_object);

    //    squirrel_object_perception_msgs::LumpToDB test_lump;
    //    test_lump.stamped_dynamic_object.header.frame_id = "/map";
    //    test_lump.stamped_dynamic_object.header.stamp = ros::Time();
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.position.x = 2.01;
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.position.y = 4.41;
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.position.z = 0.07;
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.orientation.x = 0.0;
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.orientation.y = 0.0;
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.orientation.z = 0.0;
    //    test_lump.stamped_dynamic_object.dynamic_object.pose.orientation.w = 1.0;
    //    test_lump.stamped_dynamic_object.dynamic_object.x_dim.data = 0;
    //    test_lump.stamped_dynamic_object.dynamic_object.y_dim.data = 0;
    //    test_lump.stamped_dynamic_object.dynamic_object.z_dim.data = 0;
    //    test_lump.is_examined.data = false;

    //    message_store.insert(test_lump);


    Remover_ = n_->advertiseService ("/squirrel_find_dynamic_objects", &RemoveBackground::removeBackground, this);

    ROS_INFO ("TUW: /squirrel_find_dynamic_objects ready to get service calls...");
    ros::spin ();
}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "squirrel_remove_background_server");

    RemoveBackground rm;
    rm.initialize(argc, argv);

    return 0;
}

void RemoveBackground::setStaticOctomap(std::string staticPath) {
    octomap_lib.readOctoMapFromFile(staticPath, this->staticMap, ends_with(staticPath, "bt"));
    staticMap->expand();
    octomap_lib.leaf_size = staticMap->getNodeSize(octomap_lib.tree_depth);
}

void RemoveBackground::setCurrentOctomap(octomap::OcTree *currentMap) {

    if (!currentMap) {
        ROS_INFO("TUW: no current octomap");
    }
    currentMap->expand();
    this->currentMap = currentMap;
}

octomap::OcTree RemoveBackground::subtractOctomaps() {
    cout << "Start subtracting..." << endl;
    {pcl::ScopeTime time("Subtracting ");
        return octomap_lib.subtractOctomap(staticMap, *currentMap);
    }
    cout << "Finished subtracting" << endl;
}

pcl::PointCloud<PointT>::Ptr RemoveBackground::compareOctomapToGrid(octomap::OcTree *octomap, const nav_msgs::OccupancyGridConstPtr& grid_map) {
    cout << "Start comparing octomap to gridmap..." << endl;
    {pcl::ScopeTime time("Compare Octomap to Grid");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        octomap_lib.octomapToPointcloud(octomap, cloud);
        this->compareCloudToMap(cloud, grid_map);
        cout << "Finished comparing octomap to gridmap" << endl;
        return cloud;
    }
}

void RemoveBackground::filterCloudNoise(pcl::PointCloud<PointT>::Ptr &cloud) {
    cout << "Start outlier removal..." << endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK (15);
    sor.setStddevMulThresh (0.2);
    sor.filter (*cloud);
    cout << "Finished outlier removal" << endl;
}

/* convert a occupancy grid map to a cv::Mat
 * unknown and occupied space have value 100 in the Mat
 * */
void RemoveBackground::mapToMat(const nav_msgs::OccupancyGridConstPtr& grid_map, cv::Mat &mat) {
    cout << "Start transforming map to mat..." << endl;
    unsigned int width = grid_map->info.width;
    unsigned int height = grid_map->info.height;

    mat = cv::Mat(width, height, CV_8UC1);
    std::vector<schar>::const_iterator mapDataIter = grid_map->data.begin();


    // allocate map structs so that x/y in the world correspond to x/y in the image
    // (=> cv::Mat is rotated by 90 deg, because it's row-major!)

    // iterate over map, store in image
    // (0,0) is lower left corner of OccupancyGrid
    for(unsigned int j = 0; j < height; ++j){
        for(unsigned int i = 0; i < width; ++i){
            //            int val = *mapDataIter;
            //            cout << val << endl;
            if(*mapDataIter == -1) {
                mat.at<uchar>(i,j) = 100; //set from unknown to occupied
            } else {
                mat.at<uchar>(i,j) = *mapDataIter;
            }
            ++mapDataIter;
        }
    }

    //    cv::Mat scaledMat;
    //    mat.convertTo(scaledMat,CV_8U,255.0/(100));
    //    cv::imshow("mat", scaledMat);
    //    cv::waitKey(-1);
    cout << "Finished transforming map to mat..." << endl;
}

/* compares a point cloud to a occupancy grid map
 * the point cloud is projected along the z-axis, and the occupied space of the map is dilated.
 * points which are projected on an occupied pixel are remove from the cloud (point is outside of the map or near the boundary)
 * */
void RemoveBackground::compareCloudToMap(pcl::PointCloud<PointT>::Ptr &cloud, const nav_msgs::OccupancyGridConstPtr& grid_map) {
    cout << "Start comparing map and cloud..." << endl;
    cv::Mat grid_mat;
    this->mapToMat(grid_map, grid_mat);

    int dilation_size = 2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                cv::Point( dilation_size, dilation_size ) );
    /// Apply the dilation operation
    cv::dilate( grid_mat, grid_mat, element );

    //    cv::Mat scaledMat;
    //    grid_mat.convertTo(scaledMat,CV_8U,255.0/(100));
    //    cv::imshow( "Dilation Demo", scaledMat );
    //    cv::waitKey(-1);

    for (pcl::PointCloud<PointT>::iterator it = cloud->begin(); it != cloud->end();) {
        //cout << it->x << ", " << it->y << ", " << it->z << endl;
        unsigned int grid_x = (unsigned int)((it->x - grid_map->info.origin.position.x) / grid_map->info.resolution);
        unsigned int grid_y = (unsigned int)((it->y - grid_map->info.origin.position.y) / grid_map->info.resolution);

        //remove points outside of the map and lying on the boundaries (100)
        if ((grid_mat.at<uchar>(grid_x, grid_y) == 100)){
            cloud->erase(it);
        }

        else {
            it++;
        }
    }
    cout << "Finished comparing map and cloud..." << endl;
}

/* cluster points of cloud
 * remove clusters which are a certain value above the floor or have a large z-value
 * also remove points which are not in a cluster (noise removal)
 * */
std::vector<pcl::PointCloud<PointT>::Ptr> RemoveBackground::removeClusters(pcl::PointCloud<PointT>::Ptr &cloud) {
    {pcl::ScopeTime time("Clustering ");
        cout << "Start clustering and removing..." << endl;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (std::sqrt(2) * octomap_lib.leaf_size + 0.0001); // circle radius in cm
        ec.setMinClusterSize (4);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud);

        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
        //std::cout << "Cluster indices size: " << cluster_indices.size() << std::endl;
        int j = 0;
        pcl::PCDWriter writer;
        for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            //cout << "CLUSTER " << j << endl;
            pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
                cloud_cluster->points.push_back (cloud->points[*pit]);
                //cout << cloud->points[*pit] << endl;
            }
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            //std::stringstream ss;
            //ss << "cloud_cluster_" << j << ".pcd";
            //writer.write<PointT> (ss.str (), *cloud_cluster, false);
            j++;

            PointT min_p, max_p;

            pcl::getMinMax3D(*cloud_cluster, min_p, max_p);
            //cout << "Min point: " << min_p.z << endl;
            //cout << "Max point: " << max_p.z << endl;

            //check if bounding box is above floor or is too tall
            if (min_p.z > octomap_lib.leaf_size + octomap_lib.leaf_size/2 + 0.0001|| max_p.z >= 0.5 || max_p.z <= 2* octomap_lib.leaf_size) { //max_p.z <= 0.05 should not be needed when camera is calibrated well
                //cout << "bad cluster" << endl;
            } else {
                *cloud_filtered += *cloud_cluster;
                clusters.push_back(cloud_cluster);
            }
        }
        cloud = cloud_filtered;
        cout << "Finished clustering and removing..." << endl;
        return clusters;
    }
}

std::string RemoveBackground::get_unique_object_id() {
    std::stringstream ss;
    ss << id_cnt_;
    std::string str = ss.str();
    id_cnt_++;
    return (std::string("lump") + str);
}

