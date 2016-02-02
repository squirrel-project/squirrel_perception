#include "squirrel_find_dynamic_objects_server.h"

using namespace std;

RemoveBackground::RemoveBackground() : n_(new ros::NodeHandle("~")), message_store(*n_){

}

RemoveBackground::~RemoveBackground() {
    if (n_)
        delete n_;
}

bool RemoveBackground::removeBackground (squirrel_object_perception_msgs::FindDynamicObjects::Request & request, squirrel_object_perception_msgs::FindDynamicObjects::Response & response) {
    OctomapLib omLib;

    //read the input
    octomap_msgs::OctomapConstPtr current_octomap_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary", *n_, ros::Duration(10));
    setCurrentOctomap(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*current_octomap_msg)));
    ROS_INFO("TUW: Current octomap read");

    octomap::OcTree subtractedMap = subtractOctomaps();
    if (omLib.getNumberOccupiedLeafNodes(&subtractedMap) == 0) {
        ROS_INFO("TUW: Static and current octomap are the same");
        return false;
    }

    omLib.writeOctomap(&subtractedMap, "corridor_subtracted.bt", true);

    //remove voxels close to indicated obstacles in the map
    nav_msgs::OccupancyGridConstPtr grid_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", *n_, ros::Duration(10));

    pcl::PointCloud<PointT>::Ptr filtered_cloud = compareOctomapToGrid(&subtractedMap, grid_map);
    if (filtered_cloud->size() == 0) {
        ROS_INFO("TUW: Current octomap and occupancy grid are very similar - no lump to detect");
        return false;
    }
    pcl::io::savePCDFileASCII ("cloud_after_map_comp.pcd", *filtered_cloud);

    //clustering, remove noise and cluster which are not connected to the floor
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters = removeClusters(filtered_cloud);
    pcl::io::savePCDFileASCII ("cloud_final.pcd", *filtered_cloud);

    //Response possible object positions
    int cnt = 0;
    squirrel_object_perception_msgs::StampedDynamicObject lump;

    std::vector< boost::shared_ptr<squirrel_object_perception_msgs::LumpToDB> > lump_results;
    message_store.query<squirrel_object_perception_msgs::LumpToDB>(lump_results);

    std::vector< boost::shared_ptr<squirrel_object_perception_msgs::ObjectToDB> > object_results;
    message_store.query<squirrel_object_perception_msgs::ObjectToDB>(object_results);

    ROS_INFO("TUW: Number of found lumps: %zu", clusters.size());
    for (std::vector<pcl::PointCloud<PointT>::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it)
    {
        PointT min_p, max_p;
        pcl::getMinMax3D(*(*it), min_p, max_p);

        double pose_x = min_p.x + (max_p.x - min_p.x)/2;
        double pose_y = min_p.y + (max_p.y - min_p.y)/2;
        double pose_z = min_p.z + (max_p.z - min_p.z)/2;

        geometry_msgs::Pose pose_db;
        bool is_lump_in_db = false;
        BOOST_FOREACH(boost::shared_ptr<squirrel_object_perception_msgs::LumpToDB> lump_db, lump_results) {
            pose_db = (*lump_db).stamped_dynamic_object.dynamic_object.pose;

            if((std::abs(pose_db.position.x - pose_x) < POSE_THRESH) && (std::abs(pose_db.position.y - pose_y) < POSE_THRESH)
                    && (std::abs(pose_db.position.z - pose_z) < POSE_THRESH)) {
                is_lump_in_db = true;
                ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) is already in lump DB", pose_x, pose_y, pose_z);
                break;
            }
        }

        bool is_object_in_db = false;
        if (!is_lump_in_db) {
            geometry_msgs::PoseStamped pose_stamped_db;
            BOOST_FOREACH(boost::shared_ptr<squirrel_object_perception_msgs::ObjectToDB> object_db, object_results) {
                pose_stamped_db = (*object_db).pose;

                if((std::abs(pose_stamped_db.pose.position.x - pose_x) < POSE_THRESH) && (std::abs(pose_stamped_db.pose.position.y - pose_y) < POSE_THRESH)
                        && (std::abs(pose_stamped_db.pose.position.z - pose_z) < POSE_THRESH)) {
                    is_object_in_db = true;
                    ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) is already in object DB", pose_x, pose_y, pose_z);
                    break;
                }
            }
        }

        if (!is_lump_in_db && !is_object_in_db) {
            lump.header.frame_id = "map";
            lump.header.stamp = ros::Time();
            lump.dynamic_object.pose.position.x = pose_x;
            lump.dynamic_object.pose.position.y = pose_y;
            lump.dynamic_object.pose.position.z = pose_z;
            lump.dynamic_object.pose.orientation.x = 0.0;
            lump.dynamic_object.pose.orientation.y = 0.0;
            lump.dynamic_object.pose.orientation.z = 0.0;
            lump.dynamic_object.pose.orientation.w = 1.0;
            lump.dynamic_object.x_dim.data = double(max_p.x - min_p.x);
            lump.dynamic_object.y_dim.data = double(max_p.y - min_p.y);
            lump.dynamic_object.z_dim.data = double(max_p.z - min_p.z);
            response.dynamic_objects.push_back(lump);

            //creates a marker that can be visualized in rviz
            visualization_msgs::Marker zyl_marker;
            zyl_marker.header.frame_id = "map";
            zyl_marker.header.stamp = ros::Time();
            zyl_marker.ns = "cluster_marker";
            zyl_marker.id = cnt;
            zyl_marker.lifetime = ros::Duration();
            zyl_marker.type = visualization_msgs::Marker::CYLINDER;
            zyl_marker.action = visualization_msgs::Marker::ADD;
            zyl_marker.pose.position.x = min_p.x + (max_p.x - min_p.x)/2;
            zyl_marker.pose.position.y = min_p.y + (max_p.y - min_p.y)/2;
            zyl_marker.pose.position.z = min_p.z + (max_p.z - min_p.z)/2;
            zyl_marker.pose.orientation.x = 0.0;
            zyl_marker.pose.orientation.y = 0.0;
            zyl_marker.pose.orientation.z = 0.0;
            zyl_marker.pose.orientation.w = 1.0;
            zyl_marker.scale.x = max_p.x - min_p.x;
            zyl_marker.scale.y = max_p.y - min_p.y;
            zyl_marker.scale.z = max_p.z - min_p.z;
            zyl_marker.color.r = 1.0;
            zyl_marker.color.g = 0.0;
            zyl_marker.color.b = 0.0;
            zyl_marker.color.a = 0.5;

            markerPublisher.publish(zyl_marker);
            cnt += 1;
        }
    }

    ROS_INFO("TUW: Found %zu new lumps", response.dynamic_objects.size());
    return true;
}

void RemoveBackground::initialize(int argc, char **argv) {

    n_->getParam("static_octomap_path", staticOctomapPath_);
    markerPublisher = n_->advertise<visualization_msgs::Marker>("vis_marker_dynamic_objects", 0);

    setStaticOctomap(staticOctomapPath_);


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

    ROS_INFO ("TUW: squirrel_find_dynamic_objects ready to get service calls...");
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
    ec.setClusterTolerance (0.03); // circle radius in cm
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    cout << cluster_indices.size() << " clusters found" << endl;

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;

    int j = 0;
    pcl::PCDWriter writer;
    for (std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        cout << "CLUSTER " << j << endl;
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back (cloud->points[*pit]);
            //cout << cloud->points[*pit] << endl;
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
        j++;

        PointT min_p, max_p;

        pcl::getMinMax3D(*cloud_cluster, min_p, max_p);
        cout << "Min point: " << min_p.z << endl;
        cout << "Max point: " << max_p.z << endl;

        //check if bounding box is above floor or is too tall
        if (min_p.z >= 0.1 || max_p.z >= 0.8 || max_p.z <= 0.05) { //max_p.z <= 0.05 should not be needed when camera is calibrated well
            cout << "bad cluster" << endl;
//            pcl::PointIndices indices = *it;
//            extract.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
//            extract.setNegative (true);
//            extract.filter (*cloud);
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

