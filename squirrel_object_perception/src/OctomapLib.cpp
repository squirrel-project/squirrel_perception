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

OcTree OctomapLib::dilateOctomap(OcTree *octomap) {

    OcTree dilated_map = *octomap;
    dilated_map.expand();
    for(OcTree::leaf_iterator it = octomap->begin_leafs(), end = octomap->end_leafs(); it != end; ++it) {
        OcTreeNode node = *it;
        OcTreeKey key = it.getKey();
        //octomath::Vector3 coord = dilated_map.keyToCoord(key);
        //set all neighbors to occupied
        if(dilated_map.isNodeOccupied(node)) {
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    //for (int k = -1; k <= 1; k++) {
                    OcTreeKey neighbor_key = key;
                    neighbor_key[0] += i;
                    neighbor_key[1] += j;
                    //    neighbor_key[2] += k;

                    OcTreeNode* neighbor_node = dilated_map.search(neighbor_key);
                    if (neighbor_node == NULL) {
                        dilated_map.setNodeValue(neighbor_key,logodds(dilated_map.getClampingThresMax()));
                        //dilated_map.insertRay(coord, dilated_map.keyToCoord(neighbor_key));
                    }
                    else if (!dilated_map.isNodeOccupied(neighbor_node)) {
                        neighbor_node->setLogOdds(logodds(dilated_map.getClampingThresMax()));
                    }
                    // }
                }
            }
        }
    }
    //writeOctomap(&dilated_map, "/home/edith/SQUIRREL/workspace/src/squirrel_perception/squirrel_object_perception/data/octomaps/dilated.bt",true);
    return dilated_map;
}

//this method should only be used when the two octomaps are expanded
OcTree OctomapLib::subtractOctomap(const OcTree *minuendMap, OcTree subtrahendMap) {
    //traverse through the map that is subtrahended
    for (OcTree::leaf_iterator it = subtrahendMap.begin_leafs(), end=subtrahendMap.end_leafs(); it!=end; ++it) {
        if(subtrahendMap.isNodeOccupied(*it)) {
            octomath::Vector3 nodeCoordinates = it.getCoordinate();
            OcTreeNode* node = minuendMap->search(nodeCoordinates);
            if(node) {
                if(minuendMap->isNodeOccupied(node)) {//then the node is probably background --> remove it
                    it->setLogOdds(logodds(subtrahendMap.getClampingThresMin())); //sets occupancyProbability
                } else { //static map node is free
                    //std::cout << "keep the node" << std::endl;

                }
            } else { //static map node is unknown
                //std::cout << "Node outside of minuen map" << std::endl;
                it->setLogOdds(logodds(subtrahendMap.getClampingThresMin()));
            }
        }
    }
    subtrahendMap.updateInnerOccupancy();
    return subtrahendMap;
}


//this method can be used when the occupied nodes of the static octomap are stored in a KeySet
//A bounding box can be specified. Values that are outside of the octomap, are replaced by the octomap's dimension
OcTree OctomapLib::compareOctomapToStatic(const OcTree *staticMap, OcTree currentOctomap, octomap::point3d min, octomap::point3d max) {

    double minX, minY, minZ, maxX, maxY, maxZ;
    currentOctomap.getMetricMin(minX, minY, minZ);
    currentOctomap.getMetricMax(maxX, maxY, maxZ);

    if (min.x() < minX) {
        min.x() = minX;
    }
    if (min.y() < minY) {
        min.y() = minY;
    }
    if (min.z() < minZ) {
        min.z() = minZ;
    }
    if (max.x() > maxX) {
        max.x() = maxX;
    }
    if (max.y() > maxY) {
        max.y() = maxY;
    }
    if (max.z() > maxZ) {
        max.z() = maxZ;
    }

    std::cout << min.x() <<"; " << min.y() << "; " << min.z() << std::endl;
    std::cout << max.x() <<"; " << max.y() << "; " << max.z() << std::endl;

    OcTree subtractedMap(leaf_size);

    //iterate over the current octomap and set the nodes ina new octomap to occupied when they are not in the static one
    for(octomap::OcTree::leaf_bbx_iterator it = currentOctomap.begin_leafs_bbx(min, max); it != currentOctomap.end_leafs_bbx(); it ++) {
        if(currentOctomap.isNodeOccupied(*it)) {
            KeySet::iterator key_it = static_keys.find(it.getKey());
            if (key_it != static_keys.end()) { //voxel is occupied in static map as well
                //it->setLogOdds(logodds(currentOctomap.getClampingThresMin()));
            }
            else { //check if node is unknown in the static map. this case should not happen too often as most of the occupied voxels are in the static map.
                OcTreeNode* node = staticMap->search(it.getCoordinate());
                if(!node) {
                    //it->setLogOdds(logodds(currentOctomap.getClampingThresMin()));
                } else {
                    subtractedMap.setNodeValue(it.getKey(), logodds(currentOctomap.getClampingThresMax()));
                }
            }
        }
    }
    //currentOctomap.updateInnerOccupancy();
    //return currentOctomap;
    return subtractedMap;
}

void OctomapLib::initStaticKeys(OcTree *staticMap) {

    for (OcTree::leaf_iterator it = staticMap->begin_leafs(), end=staticMap->end_leafs(); it!=end; ++it) {
        if(staticMap->isNodeOccupied(*it)) {
            expandNodeRecurs(&(*it), it.getDepth(), tree_depth);
        }
    }

    for (OcTree::leaf_iterator it = staticMap->begin_leafs(), end=staticMap->end_leafs(); it!=end; ++it) {
        if(staticMap->isNodeOccupied(*it)) {
            static_keys.insert(it.getKey());
        }
    }
}

void OctomapLib::expandOccupiedNodes(octomap::OcTree *octomap) {
    for (OcTree::leaf_iterator it = octomap->begin_leafs(), end=octomap->end_leafs(); it!=end; ++it) {
        if(octomap->isNodeOccupied(*it)) {
            expandNodeRecurs(&(*it), it.getDepth(), tree_depth);
        }
    }
}

//Copied from Octomap Framework
void OctomapLib::expandNodeRecurs(OcTreeNode* node, unsigned int depth, unsigned int max_depth) {
    if (depth >= max_depth)
        return;

    assert(node);

    // current node has no children => can be expanded
    if (!node->hasChildren()) {
        node->expandNode();
    }

    // recursively expand children
    for (unsigned int i=0; i<8; i++) {
        if (node->childExists(i)) {
            expandNodeRecurs(node->getChild(i), depth+1, tree_depth);
        }
    }
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

void OctomapLib::fillFloor(OcTree *octomap, octomap::point3d min, octomap::point3d max) {
    for (double ix =min.x(); ix < max.x(); ix += this->leaf_size) {
        for (double iy =min.y(); iy < max.y(); iy += this->leaf_size) {
            //ROS_INFO("x: %f, y: %f", ix, iy);
            octomap::OcTreeNode* node = octomap->search(ix, iy, 0);
            if (node==NULL) {
                octomap->setNodeValue(ix,iy,0,logodds(octomap->getClampingThresMax()));
            } else if (!octomap->isNodeOccupied(node)) {
                node->setLogOdds(logodds(octomap->getClampingThresMax()));
            }
        }
    }
}

//Removes the floor (voexels with height = leaf_size and no occupied voxel above)
void OctomapLib::removeFloor(OcTree *octomap) {
    double leaf_size = octomap->getNodeSize(this->tree_depth);
    double minX, minY, minZ, maxX, maxY, maxZ;
    octomap->getMetricMin(minX, minY, minZ);
    octomap->getMetricMax(maxX, maxY, maxZ);

    point3d min, max;
    min.x() = minX; min.y() = minY; min.z() = 0;
    max.x() = maxX; max.y() = maxY; max.z() = leaf_size/2;
    for(octomap::OcTree::leaf_bbx_iterator it = octomap->begin_leafs_bbx(min, max); it != octomap->end_leafs_bbx(); it ++) {
        if(octomap->isNodeOccupied(*it)) {
            OcTreeKey key = it.getKey();
            OcTreeKey key_above = key;
            key_above[2] += 1;
            OcTreeNode* node_above = octomap->search(key_above);
            if (node_above==NULL) {
                (*it).setLogOdds(logodds(octomap->getClampingThresMin()));
            } else {
                //check if node above is also occupied
                if (!octomap->isNodeOccupied(node_above)) {
                    (*it).setLogOdds(logodds(octomap->getClampingThresMin()));
                }
            }
        }
    }
}

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

void OctomapLib::writeOctomap(OcTree ocTree, std::string path, bool binary) {
    if (binary) {
        ocTree.writeBinary(path);
    } else {
        ocTree.write(path);
    }

}

OctomapLib::~OctomapLib() {
}
