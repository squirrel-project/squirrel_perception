#include "OctomapLib.h"


/**
 * Existing voxels in an octomap are either occupied or free. Unknown space is encoded implicitly by non-existing nodes.
 */

using namespace octomap;
typedef pcl::PointXYZRGB PointT;


bool OctomapLib::readOctoMapFromFile(std::string filename, OcTree *&ocTree, bool isBinary) {
    /*from ROS message:
    *octomap_msgs::Octomap map_msg;
    *AbstractOcTree* tree = octomap_msgs::msgToMap(map_msg);
    */
    if(isBinary) {
        ocTree = new OcTree(filename);
    } else {
         AbstractOcTree* tree = AbstractOcTree::read(filename);
         if(tree){ // read error returns NULL
             ocTree = dynamic_cast<OcTree*>(tree);
         }

    }
    if (!ocTree){
        return EXIT_FAILURE;
    } else {
        std::cout << "Octomap was read successfully from file" << std::endl;
        return true;
    }
}

void OctomapLib::tranformCloud2Map(pcl::PointCloud<PointT>::Ptr &cloud) {
    try
      {
        ros::Duration(1.0).sleep();
        tf_listener.waitForTransform("/kinect_depth_optical_frame","/map", ros::Time(0), ros::Duration(1.0));
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
          geometry_msgs::PointStamped p, pb;
          p.point.x = cloud->points[i].x;
          p.point.y = cloud->points[i].y;
          p.point.z = cloud->points[i].z;
          p.header.frame_id = "/kinect_depth_optical_frame";
          tf_listener.transformPoint("/map", p, pb);

          cloud->points[i].x = pb.point.x;
          cloud->points[i].y = pb.point.y;
          cloud->points[i].z = pb.point.z;
        }
      }
      catch (tf::TransformException& ex)
      {
        std::cout << "Transformation error: " << ex.what() << std::endl;
        return;
      }
}

void OctomapLib::checkCloudAgainstOctomap(const pcl::PointCloud<PointT>::Ptr &cloud, OcTree *ocTree) {
    pcl::PointCloud<PointT>::Ptr cloud_copy(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, *cloud_copy);
    int countNaNs = 0;
    for (size_t i = 0; i < cloud->points.size(); i++) {
        if (cloud->points[i].x != cloud->points[i].x) { //this cehcks for NaNs
            countNaNs +=1;
            cloud_copy->points[i].r = 255; //NaNs are red
            cloud_copy->points[i].g = 0;
            cloud_copy->points[i].b = 0;
        } else {
            OcTreeNode* node = ocTree->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 14);
            if (node != NULL) {
                if(ocTree->isNodeOccupied(*node)) {
                    //TODO: generate a binary mask with the same size as the pcl image and set occupied pixels to 1
                    cloud_copy->points[i].r = 0;
                    cloud_copy->points[i].g = 255; //occupied points are green (floor, walls, etc.)
                    cloud_copy->points[i].b = 0;
                }
            } else {
                cloud_copy->points[i].r = 0;
                cloud_copy->points[i].g = 0;
                cloud_copy->points[i].b = 255; //unknown octomap space is blue
            }
        }
    }
    std::cout << "Number of points that are NaN: " << countNaNs << std::endl;
    pcl::io::savePCDFileASCII ("cloud_against_octomap.pcd", *cloud_copy);
}

int OctomapLib::getNumberOccupiedLeafNodes(const OcTree *octomap) {
    int counter = 0;
    for(OcTree::leaf_iterator it = octomap->begin_leafs(), end = octomap->end_leafs(); it != end; ++it) {
        OcTreeNode node = *it;
        if(octomap->isNodeOccupied(node)) {
           counter += 1;
        }
    }
    return counter;
}

OcTree OctomapLib::subtractOctomap(const OcTree *minuendMap, OcTree subtrahendMap) {

    //traverse through the map that is subtrahended (smaller than the static map)
    for (OcTree::leaf_iterator it = subtrahendMap.begin_leafs(), end=subtrahendMap.end_leafs(); it!=end; ++it) {
        octomath::Vector3 nodeCoordinates = it.getCoordinate();
        OcTreeNode* node = minuendMap->search(nodeCoordinates);
        if(node) {
            if(minuendMap->isNodeOccupied(node)) {//then the node is probably background --> remove it
                it->setLogOdds(logodds(subtrahendMap.getClampingThresMin())); //sets occupancyProbability
            } else {
                //std::cout << "keep the node" << std::endl;
            }
        } else {
            //std::cout << "Node outside of minuen map" << std::endl;
            it->setLogOdds(logodds(subtrahendMap.getClampingThresMin()));
        }
    }
    subtrahendMap.updateInnerOccupancy();
    return subtrahendMap;
}


void OctomapLib::octomapToPointcloud(OcTree *octomap, pcl::PointCloud<PointT>::Ptr &cloud) {
    //leaf_iterator skips inner nodes (takes the children)
    for(OcTree::leaf_iterator it = octomap->begin_leafs(), end = octomap->end_leafs(); it != end; ++it) {
        OcTreeNode node = *it;
        if(octomap->isNodeOccupied(node)) {
           octomath::Vector3 coordinates = it.getCoordinate();
           PointT point = PointT(255, 255, 255);
           point.x = coordinates.x();
           point.y = coordinates.y();
           point.z = coordinates.z();
           cloud->push_back(point);
        }
    }
    pcl::io::savePCDFileASCII ("cloud_from_octomap.pcd", *cloud);
}

void OctomapLib::octomapExpandOccupiedNodes(OcTree *octomap) {
    for(OcTree::leaf_iterator it = octomap->begin_leafs(), end = octomap->end_leafs(); it != end; ++it) {
        OcTreeNode node = *it;
        if (octomap->isNodeOccupied(node)) {
                octomap->expand();
        }

    }
}

//void OctomapLib::octomapToMat(OcTree *octomap, cv::Mat &mat) {
//    unsigned int width, height, depth;
//    getOctomapDimension(octomap, width, height, depth);
//    double minX, minY, minZ, maxX, maxY, maxZ;
//    octomap->getMetricMin(minX, minY, minZ);
//    octomap->getMetricMax(maxX, maxY, maxZ);
//    octomap::point3d minPt(minX, minY, minZ);
//    octomap::point3d maxPt(maxX, maxY, maxZ);
//    OcTreeKey minKey = octomap->coordToKey(minPt, tree_depth);
//    OcTreeKey maxKey = octomap->coordToKey(maxPt, tree_depth);

//    mat = cv::Mat(width, height, CV_8UC1, cv::Scalar(255));

//    using namespace std;
//    //depth, width, height
//    cout << minKey[0] << ";" << maxKey[0] << ";" << maxKey[0] - minKey[0] <<endl;
//    cout << minKey[1] << ";" << maxKey[1] << ";" << maxKey[1] - minKey[1] <<endl;
//    cout << minKey[2] << ";" << maxKey[2] << ";" << maxKey[2] - minKey[2] <<endl;

////    for (int i = minKey[0]; i < maxKey[0]; i++) {
////        for (int i = minKey[0]; i < maxKey[0]; i++)
////    }

//    for(OcTree::leaf_iterator it = octomap->begin_leafs(), end = octomap->end_leafs(); it != end; ++it) {
//        OcTreeNode node = *it;
//        if(octomap->isNodeOccupied(node)) {
////           octomath::Vector3 coordinates = it.getCoordinate();
////           OcTreeKey nodeKey = it.getKey();
////           std::cout << "x: " << nodeKey[0] << "; y: " <<nodeKey[1] << "; z: " << nodeKey[2] << std::endl;

//           //mat.at<uchar>(row,col) = 0;
//        }
//    }

//}


void OctomapLib::getOctomapDimension(OcTree *octomap, unsigned int &width, unsigned int &height, unsigned int &depth) {
    double minX, minY, minZ, maxX, maxY, maxZ;
    octomap->getMetricMin(minX, minY, minZ);
    octomap->getMetricMax(maxX, maxY, maxZ);

    double res = octomap->getNodeSize(tree_depth);
    width = (maxY-minY) / res + 1; //depth
    height = (maxZ-minZ) / res + 1; //width
    depth = (maxX - minX) / res + 1; //height

    double x, y, z;
    octomap->getMetricSize(x, y, z);
    res = octomap->getResolution();
    x = x/res;
    y = y/res;
    z= z/res;
    std::cout << "Metric Size. Res: " << res << "; X: " << x << "; Y: " << y << "; Z: " << z << std::endl;


    std::cout << "Octomap - Width: " << width << "; Height: " << height << "; Depth: " << depth << std::endl;

}

void OctomapLib::printMapInfo(OcTree *ocTree) {
    std::cout << "Num Leaf Nodes: " << ocTree->getNumLeafNodes() << std::endl;
    std::cout << "Resoultion: " << ocTree->getResolution() << std::endl;
    std::cout << "Prob Miss: " << ocTree->getProbMiss() << std::endl;
    std::cout << "Prob Hit: " << ocTree->getProbHit() << std::endl;
    std::cout << "Clamping Max: " << ocTree->getClampingThresMax() << std::endl;
    std::cout << "Clamping Min: " << ocTree->getClampingThresMin() << std::endl;
    std::cout << "Occupancy Thresh: " << ocTree->getOccupancyThres() << std::endl;
    std::cout << "Num Nodes: " << ocTree->calcNumNodes() << std::endl;
    std::cout << "Tree Depth: " << ocTree->getTreeDepth() << std::endl;
    std::cout << "Volume: " << ocTree->volume() << std::endl;
    int counter = 0;
    for(OcTree::leaf_iterator it = ocTree->begin_leafs(), end = ocTree->end_leafs(); it != end; ++it) {
        OcTreeNode node = *it;
        counter+=1;
        if (ocTree->isNodeOccupied(node))
            std::cout << counter << " Occupancy: " << node.getOccupancy() << "; Is occupied" << std::endl;
        else
            std::cout << counter << " Occupancy: " << node.getOccupancy()  << "; Is NOT occupied" << std::endl;
    }
}

void OctomapLib::writeOctomap(OcTree *ocTree, std::string path, bool binary) {
    if (binary) {
        ocTree->writeBinary(path);
    } else {
        ocTree->write(path);
    }
}

OctomapLib::~OctomapLib() {
}
