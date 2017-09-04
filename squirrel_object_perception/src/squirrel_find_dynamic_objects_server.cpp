#include "squirrel_find_dynamic_objects_server.h"

using namespace std;

RemoveBackground::RemoveBackground() : n_(new ros::NodeHandle("~")), message_store(*n_){
    RemoveBackground::id_cnt_ = 0;
}

RemoveBackground::~RemoveBackground() {
    if (n_)
        delete n_;
    statistics_file.close();
}

bool RemoveBackground::removeBackground (squirrel_object_perception_msgs::FindDynamicObjects::Request & request, squirrel_object_perception_msgs::FindDynamicObjects::Response & response) {

    {pcl::ScopeTime overallTime("Service call");
        t_differencing = 0, t_comp2D = 0, t_cluster = 0;

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
            markerPublisherDynObjects.publish(zyl_marker);
        }
        //    zyl_marker.action = 3; //DELETEALL
        //    markerPublisher.publish(zyl_marker);
        vis_marker_ids.clear();

        octomap::point3d min, max;
        if (request.min.x == 0 && request.min.y == 0 && request.min.z == 0 && request.max.x == 0 && request.max.y == 0 && request.max.z == 0) {
            //default values --> use whole octomap
            ROS_INFO("Use the whole octomap for the subtraction step");
            double minX, minY, minZ, maxX, maxY, maxZ;
            currentMap->getMetricMin(minX, minY, minZ);
            currentMap->getMetricMax(maxX, maxY, maxZ);
            min.x() = minX;
            min.y() = minY;
            min.z() = minZ;
            max.x() = maxX;
            max.y() = maxY;
            max.z() = maxZ;
        } else {
            min.x() = request.min.x;
            min.y() = request.min.y;
            min.z() = request.min.z;
            max.x() = request.max.x;
            max.y() = request.max.y;
            max.z() = request.max.z;
        }

        octomap::OcTree subtractedMapVar = subtractOctomaps(min, max);
        visualizeBB(min, max);
        if (octomap_lib.getNumberOccupiedLeafNodes(&subtractedMapVar) == 0) {
            ROS_INFO("TUW: Static and current octomap are the same");
            return true;
        }


        octomap::OcTree* subtractedMap = &subtractedMapVar;

        octomap_lib.writeOctomap(*subtractedMap, "subtracted.bt", true);

        //remove voxels close to indicated obstacles in the map
        nav_msgs::OccupancyGridConstPtr grid_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", *n_, ros::Duration(10));

        pcl::PointCloud<PointT>::Ptr filtered_cloud = compareOctomapToGrid(subtractedMap, grid_map);
        if (filtered_cloud->size() == 0) {
            ROS_INFO("TUW: Current octomap and occupancy grid are very similar - no lump to detect");
            statistics_file << t_differencing << ";" << t_comp2D << ";" << ";;;\n";
            statistics_file.flush();
            return true;
        }
        if (filtered_cloud->size() > 0) {
            pcl::io::savePCDFileASCII ("cloud_after_map_comp.pcd", *filtered_cloud);
        }


        //pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
        //octomap_lib.octomapToPointcloud(subtractedMap, filtered_cloud);
        //clustering, remove noise and cluster which are not connected to the floor
        std::vector<pcl::PointCloud<PointT>::Ptr> clusters = removeClusters(filtered_cloud);
        if (filtered_cloud->size() > 0) {
            pcl::io::savePCDFileASCII ("cloud_final.pcd", *filtered_cloud);
        }

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
            double z_diam = double(max_p.z - min_p.z + octomap_lib.leaf_size);

            geometry_msgs::Pose pose_db;
            bool is_lump_in_db = false;
            bool is_classified =false;
            BOOST_FOREACH(boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> sceneObject_db, sceneObjects_results) {
                pose_db = (*sceneObject_db).pose;
                //get instersection of the two circles (returns the biggest overlapping area, if the overlap is big enough, the lump gets updated with the newst values)
                double overlap = doIntersect(pose_db.position.x, pose_db.position.y, (*sceneObject_db).bounding_cylinder.diameter, pose_x, pose_y, diam);
                std::cout << "Overlap: " << overlap << std::endl;
                if (overlap == 1.0) {
                    //check if they have the same size
                    squirrel_object_perception_msgs::BCylinder bCylinder = (*sceneObject_db).bounding_cylinder;
                    if ((std::abs(bCylinder.diameter - diam) < 0.0001) && (std::abs(bCylinder.height - z_diam) < 0.0001)) {
                        lump=(*sceneObject_db);
                        is_lump_in_db = true;
                        for (int i = 0; i < response.dynamic_objects_removed.size(); ++i)
                        {
                            if (response.dynamic_objects_removed[i].id == sceneObject_db->id)
                            {
                                response.dynamic_objects_removed.erase(response.dynamic_objects_removed.begin() + i);
                                break;
                            }
                        }
                        if ((*sceneObject_db).category != "unknown") {
                            ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) is already classified in DB", pose_x, pose_y, pose_z);
                            is_classified = true;
                            break;
                        } else {
                            ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) is already in DB", pose_x, pose_y, pose_z);
                        }
                    }
                }

                if (overlap >= 0.85 && !is_lump_in_db) {
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
                    lump.bounding_cylinder.height = z_diam;
                    lump.category="unknown";
                    lump.id= (*sceneObject_db).id;

                    is_lump_in_db = true;
                    response.dynamic_objects_updated.push_back(lump);

                    for (int i = 0; i < response.dynamic_objects_removed.size(); ++i)
                    {
                        if (response.dynamic_objects_removed[i].id == sceneObject_db->id)
                        {
                            response.dynamic_objects_removed.erase(response.dynamic_objects_removed.begin() + i);
                            break;
                        }
                    }
                    ROS_INFO("TUW: Lump at pose (x,y,z) = (%f, %f, %f) got updated in DB", pose_x, pose_y, pose_z);
                    break;
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
                lump.bounding_cylinder.height = z_diam;
                lump.category="unknown";
                lump.id=get_unique_object_id();
                response.dynamic_objects_added.push_back(lump);

                //message_store.insert(lump);

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
                zyl_marker.color.g = 1.0;
                zyl_marker.color.b = 1.0;
                zyl_marker.color.a = 0.6;
                markerPublisherDynObjects.publish(zyl_marker);
                vis_marker_ids.push_back(zyl_marker.id);
            }
        }

        ROS_INFO("TUW: Added %zu lumps", response.dynamic_objects_added.size());
        ROS_INFO("TUW: Updated %zu lumps", response.dynamic_objects_updated.size());
        ROS_INFO("TUW: Removed %zu lumps", response.dynamic_objects_removed.size());

        statistics_file << t_differencing << ";" << t_comp2D << ";" << t_cluster << ";" << overallTime.getTime() << ";" << currentMap->size() << ";" << clusters.size() << "\n";
        statistics_file.flush();

        return true;
    }
}

//returns the amount of intersection in percent (x% of the area of c1 is covered by c2)
//http://mathworld.wolfram.com/Circle-CircleIntersection.html
float RemoveBackground::doIntersect(double c1_posx, double c1_posy, double c1_rad, double c2_posx, double c2_posy, double c2_rad) {
    float intersectionPerc = 0.0;

    double distance, distance2;
    distance2  =(c1_posx-c2_posx)*(c1_posx-c2_posx) + (c1_posy-c2_posy)*(c1_posy - c2_posy);
    distance = std::sqrt(distance2);

    //not intersecting
    if (distance >= c1_rad+c2_rad) {
        intersectionPerc = 0.0;
        return intersectionPerc;
    }

    //one of the circles covers the other circle completely
    if (distance <= std::abs(c1_rad - c2_rad)) {
        intersectionPerc = 1.0;
        return intersectionPerc;
    }

    double overlappingArea;
    overlappingArea = c1_rad*c1_rad * acos((c1_rad*c1_rad - c2_rad*c2_rad + distance2) / (2*distance*c1_rad)) +
            c2_rad*c2_rad * acos((c2_rad*c2_rad - c1_rad*c1_rad + distance2) / (2*distance*c2_rad)) -
            0.5 * std::sqrt((-distance+c1_rad+c2_rad) * (distance + c1_rad-c2_rad) * (distance-c1_rad+c2_rad) * (distance+c1_rad+c2_rad));

    double areaC1 = c1_rad*c1_rad * M_PI;
    double areaC2 = c2_rad*c2_rad * M_PI;
    intersectionPerc = std::max(overlappingArea / areaC1, overlappingArea/areaC2);

    return intersectionPerc;

}

void RemoveBackground::initialize(int argc, char **argv) {

    n_->getParam("static_octomap_path", staticOctomapPath_);
    markerPublisherDynObjects = n_->advertise<visualization_msgs::Marker>("vis_marker_dynamic_objects", 0);
    markerPubBBTriangle = this->n_->advertise<visualization_msgs::Marker>("bb_triangle", 1);
    markerPubBB = this->n_->advertise<visualization_msgs::Marker>("bb_octomap_comp", 1);

    setStaticOctomap(staticOctomapPath_);
    octomap_lib.initStaticKeys(staticMap);

    if (staticMap->getNumLeafNodes() == 0) {
        ROS_WARN("The static octomap is empty! You probably try to use the default octomap.");
    }

    //insert some objects for testing purposes
//    squirrel_object_perception_msgs::SceneObject test_lump;
//    test_lump.header.frame_id = "/map";
//    test_lump.header.stamp = ros::Time();
//    test_lump.id = "test_lump";
//    test_lump.category = "unknown";
//    test_lump.pose.position.x = 1.5;
//    test_lump.pose.position.y = 1;
//    test_lump.pose.position.z = 0.00;
//    test_lump.pose.orientation.x = 0.0;
//    test_lump.pose.orientation.y = 0.0;
//    test_lump.pose.orientation.z = 0.0;
//    test_lump.pose.orientation.w = 1.0;
//    test_lump.bounding_cylinder.diameter = 0.8;
//    test_lump.bounding_cylinder.height = 2;
//    message_store.insert(test_lump);

    statistics_file.open("find_objects_statistic.txt", std::ofstream::out | std::ofstream::trunc);
    statistics_file << "Time for subtraction; Time for comparing cloud against 2D grid; Time to cluster and filter; overall time; number of nodes in current octomap; Number of clusters\n";
    statistics_file.flush();
    Remover_ = n_->advertiseService ("/squirrel_find_dynamic_objects", &RemoveBackground::removeBackground, this);
    checkWaypointServer = n_->advertiseService ("/squirrel_check_viewcone", &RemoveBackground::checkWaypoint, this);


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
    octomap_lib.leaf_size = staticMap->getNodeSize(octomap_lib.tree_depth);
}

void RemoveBackground::setCurrentOctomap(octomap::OcTree *currentMap) {

    if (!currentMap) {
        ROS_INFO("TUW: no current octomap");
    }
    octomap_lib.expandOccupiedNodes(currentMap);
    this->currentMap = currentMap;
}

octomap::OcTree RemoveBackground::subtractOctomaps(octomap::point3d min, octomap::point3d max) {
    cout << "Start subtracting..." << endl;
    {pcl::ScopeTime time("Subtracting ");
        octomap::OcTree result = octomap_lib.compareOctomapToStatic(staticMap, *currentMap, min, max);
        t_differencing = time.getTime();
        cout << "Finished subtracting" << endl;
        return result;
    }
}

pcl::PointCloud<PointT>::Ptr RemoveBackground::compareOctomapToGrid(octomap::OcTree *octomap, const nav_msgs::OccupancyGridConstPtr& grid_map) {
    cout << "Start comparing octomap to gridmap..." << endl;
    {pcl::ScopeTime time("Compare Octomap to Grid");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        octomap_lib.octomapToPointcloud(octomap, cloud);
        this->compareCloudToMap(cloud, grid_map);
        cout << "Finished comparing octomap to gridmap" << endl;
        t_comp2D = time.getTime();
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

    cv::imwrite("2d_grid.png", grid_mat);

    cv::Mat scaledMat;
    grid_mat.convertTo(scaledMat,CV_8U,255.0/(100));
    cv::imwrite("2d_grid_scaled.png", scaledMat);

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
            cout << "Min point: " << min_p.z << endl;
            cout << "Max point: " << max_p.z << endl;

            //check if bounding box is above floor or is too tall
            if (min_p.z > octomap_lib.leaf_size + octomap_lib.leaf_size/2 + 0.0001|| max_p.z >= 0.5) {// || max_p.z <= 2* octomap_lib.leaf_size) { //max_p.z <= 0.05 should not be needed when camera is calibrated well
                //cout << "bad cluster" << endl;
            } else {
                *cloud_filtered += *cloud_cluster;
                clusters.push_back(cloud_cluster);
            }
        }
        cloud = cloud_filtered;
        cout << "Finished clustering and removing..." << endl;
        t_cluster = time.getTime();
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

bool RemoveBackground::checkWaypoint (squirrel_object_perception_msgs::CheckWaypoint::Request & request, squirrel_object_perception_msgs::CheckWaypoint::Response & response) {
    {pcl::ScopeTime overallTime("Check waypoint call");

        //read the input
        octomap_msgs::OctomapConstPtr current_octomap_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary", *n_, ros::Duration(10));
        setCurrentOctomap(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*current_octomap_msg)));
        ROS_INFO("TUW: Current octomap read");

        geometry_msgs::Pose pose = request.waypoint;
        geometry_msgs::Quaternion q_msg = pose.orientation;
        tf::Quaternion q;
        q.setX(q_msg.x);
        q.setY(q_msg.y);
        q.setZ(q_msg.z);
        q.setW(q_msg.w);

        double yaw = tf::getYaw(q);

        // Calculate the triangle points encompasses the area that is viewed.
        tf::Vector3 view_point(pose.position.x, pose.position.y, pose.position.z);

        tf::Vector3 v0(request.viewing_distance, 0.0f, 0.0f);
        v0 = v0.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), yaw);

        tf::Vector3 v0_normalised = v0.normalized();

        tf::Vector3 v1 = v0;
        v1 = v1.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), request.fov / 2.0f);
        v1 = v1.normalize();

        float length = v0.length() / v0_normalised.dot(v1);
        v1 *= length;
        v1 += view_point;

        tf::Vector3 v2 = v0;
        v2 = v2.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), -request.fov / 2.0f);
        v2 = v2.normalize();
        length = v0.length() / v0_normalised.dot(v2);
        v2 *= length;
        v2 += view_point;

        ROS_INFO("Triangle with following points (%f,%f), (%f,%f), (%f,%f)", pose.position.x, pose.position.y,
                 v1.x(), v1.y(), v2.x(), v2.y());

        octomap::point3d min, max;
        min.x() = std::min(std::min(v1.x(), pose.position.x), v2.x());
        min.y() = std::min(std::min(v1.y(), v2.y()), pose.position.y);
        min.z() = -octomap_lib.leaf_size + 0.00001;
        max.x() = std::max(std::max(v1.x(), pose.position.x), v2.x());
        max.y() = std::max(std::max(v1.y(), v2.y()), pose.position.y);
        max.z() = 0-0.00001;


        //        visualization_msgs::Marker marker;
        //        marker.header.frame_id = "/map";
        //        marker.header.stamp = ros::Time::now();
        //        marker.ns = "bb";
        //        marker.id = 0;
        //        marker.type = visualization_msgs::Marker::CUBE;
        //        marker.action = visualization_msgs::Marker::ADD;
        //        marker.pose.position.x = min.x() + (max.x()-min.x())/2;
        //        marker.pose.position.y = min.y() + (max.y()-min.y())/2;;
        //        marker.pose.position.z = 0;
        //        marker.pose.orientation.x = 0.0;
        //        marker.pose.orientation.y = 0.0;
        //        marker.pose.orientation.z = 0.0;
        //        marker.pose.orientation.w = 1.0;
        //        marker.scale.x = max.x()-min.x();
        //        marker.scale.y = max.y()-min.y();
        //        marker.scale.z = 0.5;
        //        marker.color.r = 1.0f;
        //        marker.color.g = 0.0f;
        //        marker.color.b = 0.0f;
        //        marker.color.a = 0.5f;
        //        markerPubBBTriangle.publish(marker);

        int countMissingNodes = 0;
        int countTriangleNodes = 0;
        int countBB =0;
        //currentMap->expand();
        for (double ix =min.x(); ix < max.x(); ix += octomap_lib.leaf_size) {
            for (double iy =min.y(); iy < max.y(); iy += octomap_lib.leaf_size) {
                countBB += 1;
                octomap::point3d node_coordinates(ix, iy, max.z());
                int nvert = 3; //number of vertices
                std::vector<float> triangle_x, triangle_y;
                triangle_x.push_back(v1.x());
                triangle_x.push_back(v2.x());
                triangle_x.push_back(pose.position.x);
                triangle_x.push_back(v1.x());
                triangle_y.push_back(v1.y());
                triangle_y.push_back(v2.y());
                triangle_y.push_back(pose.position.y);
                triangle_y.push_back(v1.y());

                int i, j, is_in_viewcone = 0;
                for (i = 0, j = nvert-1; i < nvert; j = i++) {
                    if ( ((triangle_y[i]>= node_coordinates.y()) != (triangle_y[j]>node_coordinates.y())) &&
                         (node_coordinates.x() < (triangle_x[j]-triangle_x[i]) * (node_coordinates.y()-triangle_y[i]) / (triangle_y[j]-triangle_y[i]) + triangle_x[i]) )
                        is_in_viewcone = !is_in_viewcone;
                }
                if (is_in_viewcone) {
                    octomap::OcTreeNode* node = currentMap->search(ix, iy, max.z());
                    if (node == NULL) {
                        countMissingNodes+=1;
                        //ROS_INFO ("Unknown node in bounding box");
                    }
                    /* else if(!currentMap->isNodeOccupied(node)) { //this case should not occure
                        std::cout << "Node is not occupied" << std::endl;
                        //                    std::cout << "Waypoint should be used. It is not fully covered by the octomap! Position: ("
                        //                              << node_coordinates.x() << "," << node_coordinates.y() << "," << node_coordinates.z() << ")" << std::endl;
                        countMissingNodes+=1;
                    }*/ else {
                        //                    std::cout << "Node is occupied and in view cone: ("
                        //                              << node_coordinates.x() << "," << node_coordinates.y() << "," << node_coordinates.z() << ")" << std::endl;
                        countTriangleNodes+=1;
                    }
                }
            }
        }
        ROS_INFO("Number of elements in BB: %d", countBB);

        ROS_INFO("Number of missing nodes %d, number of filled nodes %d", countMissingNodes, countTriangleNodes);
        if (countMissingNodes == 0) {
            response.explore_waypoint.data = false;
            ROS_INFO("View cone is fully covered!");
            return true;
        }
        float f = countMissingNodes/(float)(countTriangleNodes+countMissingNodes);
        ROS_INFO("Overlapping measurement: %f", 1-f);
        if (countMissingNodes/(float)(countTriangleNodes+countMissingNodes) > 0.1) {   //more than 10 percent of the nodes are missing
            ROS_INFO("More than 10 percent of the view cone are not covered");
            response.explore_waypoint.data = true;
            return true;
        }

        ROS_INFO("Almost everything of the view cone is covered");
        response.explore_waypoint.data = false;
        return true;
    }
}

void RemoveBackground::visualizeBB(octomap::point3d min, octomap::point3d max) {
    //std::cout << min.x() <<"; " << min.y() << "; " << min.z() << std::endl;
    //std::cout << max.x() <<"; " << max.y() << "; " << max.z() << std::endl;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "BBoctomap";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = min.x() + (max.x() - min.x())/2;
    marker.pose.position.y = min.y() + (max.y() - min.y())/2;;
    marker.pose.position.z = min.z() + (max.z() - min.z())/2;;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = (max.x() - min.x());
    marker.scale.y = (max.y() - min.y());
    marker.scale.z = (max.z() - min.z());
    marker.color.a = 0.4; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerPubBB.publish(marker);
}
