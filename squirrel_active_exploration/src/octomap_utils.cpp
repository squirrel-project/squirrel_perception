#include "squirrel_active_exploration/octomap_utils.h"

using namespace std;
using namespace pcl;
using namespace octomap;

/* === COMPARISONS === */

bool compare_octree_key(const OcTreeKey &lhs, const OcTreeKey &rhs)
{
    if (lhs.k[0] < rhs.k[0]) return true;
    if (rhs.k[0] < lhs.k[0]) return false;
    // Otherwise, lhs.k[0] == rhs.k[0]
    if (lhs.k[1] < rhs.k[1]) return true;
    if (rhs.k[1] < lhs.k[1]) return false;
    // Otherwise, lhs.k[1] == rhs.k[1]
    if (lhs.k[2] < rhs.k[2]) return true;
    if (rhs.k[2] < lhs.k[2]) return false;
    return false;
}

bool equal_octree_key(const OcTreeKey &lhs, const OcTreeKey &rhs)
{
    if ((lhs.k[0] == rhs.k[0]) && (lhs.k[1] == rhs.k[1]) && (lhs.k[2] == rhs.k[2]))
        return true;
    else
        return false;
}

bool compare_pair_double_octree_key(const pair<double,OcTreeKey> &lhs, const pair<double,OcTreeKey> &rhs)
{
    if (lhs.first < rhs.first)
        return true;
    else
        return false;
}

bool compare_keys_box(const keys_box &lhs, const keys_box &rhs)
{
    if (lhs.first.size() < rhs.first.size())
        return true;
    else
        return false;
}

/* === OCTOMAP FUNCTIONS === */

bool octree_to_occupied_free_clouds(const OcTree &tree, PointCloud<PointT> &occupied, PointCloud<PointT> &free, const unsigned int &depth)
{
    unsigned int tree_depth = depth;
    if (depth <= 0 || depth >= tree.getTreeDepth())
        tree_depth = tree.getTreeDepth();

    // Create a point for each octree voxel
    occupied.clear();
    free.clear();
    occupied.resize(tree.getNumLeafNodes());
    free.resize(tree.getNumLeafNodes());
    int occupied_size = 0;
    int free_size = 0;
    for(OcTree::leaf_iterator it = tree.begin_leafs(tree_depth), end = tree.end_leafs(); it!= end; ++it)
    {
        point3d coord = it.getCoordinate();
        if (it->getOccupancy() > 0.5)
        {
            occupied.points[occupied_size].x = coord.x();
            occupied.points[occupied_size].y = coord.y();
            occupied.points[occupied_size].z = coord.z();
            ++occupied_size;
        }
        else
        {
            free.points[free_size].x = coord.x();
            free.points[free_size].y = coord.y();
            free.points[free_size].z = coord.z();
            ++free_size;
        }
    }
    occupied.resize(occupied_size);
    free.resize(free_size);
    return true;
}

bool get_octree_ground(const PointCloud<PointT> &in_cloud, vector<int> &indices, PointCloud<PointT> &ground, PointCloud<PointT> &remaining)
{
    SACSegmentation<PointT> seg;
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    PointCloud<PointT>::Ptr in_cloud_ptr (new PointCloud<PointT>(in_cloud));
    seg.setInputCloud (in_cloud_ptr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        ROS_ERROR("get_octree_ground::get_octree_ground : failed to get ground plane");
        return false;
    }
    indices = inliers->indices;

    // Extract the planar inliers from the input cloud
    ExtractIndices<PointT> extract;
    extract.setInputCloud (in_cloud_ptr);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (ground);

    // Extract the planer outliers
    extract.setNegative (true);
    extract.filter (remaining);

    return true;
}


PointCloud<PointT> octree_to_cloud(const OcTree &tree, const unsigned int &depth)
{
    PointCloud<PointT> occupied_cloud;
    PointCloud<PointT> free_cloud;
    octree_to_occupied_free_clouds(tree, occupied_cloud, free_cloud, depth);
    return occupied_cloud;
}

double octree_ground_height(const OcTree &tree, const unsigned int &depth)
{
    PointCloud<PointT> occupied_cloud = octree_to_cloud(tree, depth);
    vector<int> ground_indices;
    PointCloud<PointT> ground_cloud;
    PointCloud<PointT> nonground_cloud;
    get_octree_ground(occupied_cloud, ground_indices, ground_cloud, nonground_cloud);
    PointT min_gr, max_gr;
    getMinMax3D (ground_cloud, min_gr, max_gr);
    return max_gr.data[2];
}

double octree_resolution_at_depth(const OcTree &tree, const unsigned int &depth)
{
    // If invalid depth
    if (depth <= 0)
    {
        ROS_WARN("octomap_utils::octree_resolution_at_depth : input depth %u is invalid", depth);
        return tree.getResolution();
    }
    if (depth >= tree.getTreeDepth())
    {
        ROS_WARN("octomap_utils::octree_resolution_at_depth : input depth %u is maximum already", depth);
        return tree.getResolution();
    }
    //cout << "tree resolution " << tree.getResolution() << endl;
    // Otherwise find the separation between the centres of two neighbouring voxels
    point3d pta;
    OcTree::leaf_iterator it = tree.begin_leafs(depth);
    OcTree::leaf_iterator end = tree.end_leafs();
    pta = it.getCoordinate();
    double d_smallest = numeric_limits<double>::infinity();
    double d = 0;
    ++it;
    for (it; it != end; ++it)
    {
        // If the separation is smaller than the current best
        d = pta.distance(it.getCoordinate());
        if (d < d_smallest)
            d_smallest = d;
        // Advance the pointers and try the next keys
        pta = it.getCoordinate();
    }
    // If d_smallest is infinity
    if (d_smallest == numeric_limits<double>::infinity())
    {
        ROS_ERROR("octomap_utils::octree_resolution_at_depth : could not find valid resolution at depth %u", depth);
        return tree.getResolution();
    }
    //cout << "d_smallest " << d_smallest << endl;
    return d_smallest;
}

vector<point3d> voxels_from_keys(const OcTree &tree, const vector<OcTreeKey> &keys)
{
    // Get the coordinate for each key
    vector<point3d> points;
    points.resize(keys.size());
    for (size_t i = 0; i < keys.size(); ++i)
        points[i] = tree.keyToCoord(keys[i]);
    return points;
}

vector<OcTreeKey> visible_voxels(const OcTree &tree, const point3d &origin, const robot_parameters &robot, const double &zmin,
                                 const double &zheight, const unsigned int &depth)
{
    // Search the voxels in a bounding box
    point3d min (origin.x()-robot.outer_range, origin.y()-robot.inner_range, zmin);
    point3d max (origin.x()+robot.outer_range, origin.y()+robot.inner_range, zheight);
    // Check value of depth
    unsigned int tree_depth = depth;
    if (depth <= 0)
    {
        ROS_WARN("octomap_utils::visible_voxels : input depth %u is invalid", depth);
        tree_depth = tree.getTreeDepth();
    }
    if (depth >= tree.getTreeDepth())
    {
        ROS_WARN("octomap_utils::visible_voxels : input depth %u is already maximum", depth);
        tree_depth = tree.getTreeDepth();
    }
    // Store the distances
    vector<pair<double,OcTreeKey> > distances;
    for (OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(min,max,tree_depth), end = tree.end_leafs_bbx(); it != end; ++it)
        distances.push_back (make_pair(origin.distance(it.getCoordinate()),it.getIndexKey()));
    // Every voxel (known by its key) is within the range of the origin and above the minimum z height
    //cout << "Num points in range of origin " << distances.size() << endl;

    // Sort the distances from largest to smallest
    sort(distances.begin(), distances.end(), compare_pair_double_octree_key);

    vector<OcTreeKey> all_keys;
    for (vector<pair<double,OcTreeKey> >::reverse_iterator it = distances.rbegin(); it != distances.rend(); ++it)
    {
        // Get the keys on the ray
        point3d kpt = tree.keyToCoord(tree.adjustKeyAtDepth(it->second, tree_depth));
        KeyRay kr;
        if (tree.computeRayKeys(origin, kpt, kr))
        {
            // Check each key and add to visible_voxels
            for (KeyRay::iterator kit = kr.begin(), end = kr.end(); kit != end; ++kit)
            {
                OcTreeKey k = tree.adjustKeyAtDepth(*kit, tree_depth);
                OcTreeNode *node = tree.search(k, tree_depth);
                if (node)
                {
                    // If the node is occupied then break,
                    // nodes are returned in increasing distance so any after this are not
                    // visible because of the first occupied cell that is encountered
                    if (node->getOccupancy() > 0.5)
                    {
                        break;
                    }
                    // Otherwise continue to add
                    else
                    {
                        // Get the point
                        point3d p = tree.keyToCoord(k, tree_depth);
                        // Check that the height is within the limits
                        if (p.z() >= zmin && p.z() <= zheight)
                        {
                            // Check that the x-y is within the ranges
                            double xydist = origin.distanceXY(p);
                            if (xydist >= robot.inner_range && xydist <= robot.outer_range)
                                all_keys.push_back(k);
                        }
                    }
                }
            }
        }
    }
    //cout << "Num all keys " << all_keys.size() << endl;
    sort(all_keys.begin(), all_keys.end(), compare_octree_key);
    all_keys.erase(unique(all_keys.begin(), all_keys.end(), equal_octree_key), all_keys.end());
    //cout << "Num visible voxels " << all_keys.size() << endl;

    return all_keys;
}

vector<keys_box> compute_visibility(const OcTree &tree, const vector<point3d> &locations, const robot_parameters &robot,
                                    const double &zmin, const double &zheight, const unsigned int &depth)
{
    // Compute the visible voxels
    vector<keys_box> valid_locations;
    int locations_size = locations.size();
    for (size_t i = 0; i < locations_size; ++i)
    {
        vector<OcTreeKey> vis = visible_voxels(tree, locations[i], robot, zmin, zheight, depth);
        cout << i << "/" << locations_size << ": (" << locations[i].x() << " " << locations[i].y()<< " " << locations[i].z()
             << ") " << vis.size() << endl;
        if (vis.size() > 0)
            valid_locations.push_back(make_pair(vis,locations[i]));
    }
    // Sort from smallest to largest number of voxels
    sort(valid_locations.begin(), valid_locations.end(), compare_keys_box);
    // Reverses
    reverse(valid_locations.begin(), valid_locations.end());

    return valid_locations;
}

void valid_on_ground(const OcTree &tree, const double &robot_radius, const double &zmin, const double &zmax,
                     const double &floor_height, const point3d &location, bool &valid, const bool &check_floor)
{
    // Search the voxels in a bounding box
    point3d min (location.x()-robot_radius, location.y()-robot_radius, zmin);
    point3d max (location.x()+robot_radius, location.y()+robot_radius, zmax);
    // Find all voxels in this bounding box
    valid = true;
    bool valid_ground = false;
    if (!check_floor)
        valid_ground = true;
    for (OcTree::leaf_bbx_iterator it = tree.begin_leafs_bbx(min,max), end = tree.end_leafs_bbx(); it != end; ++it)
    {
        // Check that there are no obstacles at this location
        point3d pt = it.getCoordinate();
        if (location.distanceXY(pt) <= robot_radius && it->getOccupancy() > 0.5)
        {
            valid = false;
            break;
        }
        // Also check that there is ground available beneath the robot so that it can stand
        if (!valid_ground)
        {
            point3d pt = it.getCoordinate();
            if (location.distanceXY(pt) <= robot_radius && pt.z() >= zmin && pt.z() <= floor_height && it->getOccupancy() < 0.5)
                valid_ground = true;
        }
    }
    // If valid is true but no floor, then set valid to false
    if (valid && !valid_ground)
        valid = false;
    // Otherwise valid remains true
}

void valid_on_ground(const OcTree &tree, const double &robot_radius, const double &zmin, const double &zmax, const double &floor_height,
                     vector<keys_box> &locations, const bool &check_floor)
{
    // A new vector
    vector<keys_box> new_locations;
    // For each location, check that there are no occupied points in a small radius near it
    for (vector<keys_box>::size_type i = 0; i < locations.size(); ++i)
    {
        point3d loc = locations[i].second;
        bool valid = true;
        valid_on_ground(tree, robot_radius, zmin, zmax, floor_height, loc, valid, check_floor);
        if (valid)
            new_locations.push_back(locations[i]);
    }
    // Set the locations back
    locations = new_locations;
}

vector<point3d> exhaustive_coverage_set(const OcTree &tree, const robot_parameters &robot, const double &step, const unsigned int &depth)
{
    PointCloud<PointT> occupied_cloud = octree_to_cloud(tree);
    double zground = octree_ground_height(tree, depth);
    double zmin = zground  + _GROUND_THRESH;
    double zmax = zground + robot.height;
    double zheight = zmin + tree.getResolution();
    int radinc = 4;
    int numang = 8;

    // Min-max map
    PointCloud<PointT> inrange_cloud;
    inrange_cloud.resize(occupied_cloud.size());
    int n_size = 0;
    for (size_t i = 0; i < occupied_cloud.size(); ++i)
    {
        if (occupied_cloud.points[i].z >= zmin && occupied_cloud.points[i].z <= zmax)
        {
            inrange_cloud.points[n_size].x = occupied_cloud.points[i].x;
            inrange_cloud.points[n_size].y = occupied_cloud.points[i].y;
            inrange_cloud.points[n_size].z = occupied_cloud.points[i].z;

            ++n_size;
        }
    }
    inrange_cloud.resize(n_size);
    PointT minr, maxr;
    getMinMax3D (inrange_cloud, minr, maxr);

    double x = minr.data[0]+step;
    double y = minr.data[1]+step;
    double z = zmax;
//    point3d minpt (minr.data[0], minr.data[1], minr.data[2]);
//    point3d maxpt (maxr.data[0], maxr.data[1], maxr.data[2]);
    vector<point3d> cubes;
    while (true)
    {
        point3d p (x,y,z);
        // Check if valid on ground
        bool valid = true;
        valid_on_ground(tree, robot.radius, zmin, zmax, zheight, p, valid);

        // If valid
        if (valid)
        {
            cubes.push_back(p);
        }
        // Otherwise search around the vicinity of this location
        else
        {
            point3d q;
            bool finished = false;
            int rad_count = 0;
            // Keep on attempting to find valid locations on an increasing radius around this location
            while (!finished)
            {
                double r = ((double)(rad_count+1)/(double)radinc)*robot.radius;
                // Try different angles
                for (int j = 0; j < numang; ++j)
                {
                    double theta = ((double)j/(double)numang)*M_PI/180;
                    // New location
                    q = point3d(x+r*cos(theta), y+r*sin(theta), z);
                    valid = true;
                    valid_on_ground(tree, robot.radius, zmin, zmax, zheight, q, valid);
                    if (valid)
                    {
                        cubes.push_back(q);
                        finished = true;
                        break;
                    }
                }
                ++rad_count;
                if (rad_count == radinc)
                    finished = true;
            }
        }

        // Increase x
        x += step;
        if (x > maxr.data[0])
        {
            // Reset x
            x = minr.data[0]+step;
            // Increase y
            y += step;
            // If y has reached the limit then exit
            if (y > maxr.data[1])
                break;
        }
    }
    // Return
    return cubes;
}

bool compute_visible_locations(const OcTree &tree, const robot_parameters &robot, const double &step, vector<OcTreeKey> &tree_keys,
                               vector<keys_box> &locations, const unsigned int &depth)
{
    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    OcTree expanded_tree (tree);
    expanded_tree.expand();
    expanded_tree.getMetricMin(x0, y0, z0);
    expanded_tree.getMetricMax(x1, y1, z1);
    double zground = octree_ground_height(expanded_tree, depth);
    double zmin = zground  + _GROUND_THRESH;
    double zheight = zmin + octree_resolution_at_depth(tree, depth);

    // Tree keys
    tree_keys.clear();
    for (OcTree::leaf_iterator it = expanded_tree.begin_leafs(depth), end = expanded_tree.end_leafs(); it!= end; ++it)
    {
        // Get the point and check it is within the limits and is not occupied
        point3d pt = it.getCoordinate();
        if (pt.z() >= zmin && pt.z() <= zheight && it->getOccupancy() < 0.5)
            tree_keys.push_back (expanded_tree.adjustKeyAtDepth(it.getIndexKey(), depth));
    }
    sort(tree_keys.begin(), tree_keys.end(), compare_octree_key);
    tree_keys.erase(unique(tree_keys.begin(), tree_keys.end(), equal_octree_key), tree_keys.end());

    // Get the valid locations
    vector<point3d> cubes = exhaustive_coverage_set(expanded_tree, robot, step, depth);
    // Compute the visible voxels
    locations.clear();
    ros::Time start_time = ros::Time::now();
    locations = compute_visibility(expanded_tree, cubes, robot, zmin, zheight, depth);
    ros::Time end_time = ros::Time::now();

    // Print out the time information
    ros::Duration time_diff = end_time - start_time;
    double secs = time_diff.toSec();
    double msecs = secs*1000;
    ROS_INFO("octomap_utils::compute_visible_locations : time to compute all visibilites %.6f seconds", secs);
    ROS_INFO("octomap_utils::compute_visible_locations : average time to compute visibility %.6f milliseconds", msecs/locations.size());

    return true;
}

vector<keys_box> optimize_coverage_set(const vector<OcTreeKey> &tree_keys, const vector<keys_box> &locations, const double &range, const int &max_iters)
{
    ROS_INFO("octomap_utils::optimize_coverage_set : maximum iterations set to %u", max_iters);
    // Make a copy of the vectors
    vector<OcTreeKey> tree_keys_copy = tree_keys;
    vector<keys_box> locations_copy = locations;
    int num_begin = tree_keys_copy.size();
    int num_previous = tree_keys_copy.size();
    // Determine the coverage set by adding each location and updating the keys that are visible in the
    // remaining locations by removing what has been seen already
    vector<keys_box> coverage_set;
    // Add the first location to the coverage set
    keys_box current = locations_copy.front();
    coverage_set.push_back(current);
    int num_first = current.first.size();
    locations_copy.erase(locations_copy.begin());
    // Update the keys in the remaining locations
    update_keys(current, range, locations_copy); // locations will be resorted (largest to smallest), and zeros will be removed
    // Update the keys in the whole map
    remove_keys(current, tree_keys_copy); // remove keys visible from the location
    // Update
    int count = 0;
    while (tree_keys_copy.size() > 0)
    {
        cout << count << ": remaining = " << tree_keys_copy.size() << " (-" << num_previous-tree_keys_copy.size() << ")" << endl;
        // If the remaining is very small then exit
        if (tree_keys_copy.size() < 0.05*(double)num_begin)
        {
            ROS_WARN("octomap_utils::optimize_coverage_set : very few voxels remaining, exiting");
            break;
        }
        keys_box best_location = locations_copy.front();
        double best_distance = current.second.distance(best_location.second);
        double best_coverage = best_location.first.size();
        int best_pos = 0;
        // Get the location with the biggest visibility and closest to the current location
        for (size_t i = 1; i < locations_copy.size(); ++i)
        {
            // If the element does not have the same number of visible voxels, then reached the end
            if (locations_copy[i].first.size() != best_coverage)
                break;
            // Otherwise check the distance
            double d = current.second.distance(locations_copy[i].second);
            if (d < best_distance)
            {
                // This location is the new best
                best_location = locations_copy[i];
                best_distance = d;
                best_coverage = locations_copy[i].first.size();
                best_pos = i;
            }
        }
        cout << "location removes " << best_location.first.size() << endl;
        // If the number to remove is very small, then exit
        if (best_location.first.size() < 0.05*num_first)
        {
            ROS_WARN("octomap_utils::optimize_coverage_set : view does not remove many voxels, exiting");
            break;
        }
        num_previous = tree_keys_copy.size();
        // Remove the location
        locations_copy.erase(locations_copy.begin()+best_pos);
        // Update the remaining locations
        update_keys(best_location, range, locations_copy);
        // Update the map
        remove_keys(best_location, tree_keys_copy);
        // Set current location
        current = best_location;
        coverage_set.push_back(current);
        // Increment counter
        ++count;
        if (max_iters > 0)
        {
            if (count == max_iters)
            {
                ROS_WARN("octomap_utils::optimize_coverage_set : reached maximum number of iterations, exiting");
                break;
            }
        }
    }

    return coverage_set;
}

void update_keys(const keys_box &location, const double &range, vector<keys_box> &other_locations)
{
    // Update every element in vec by removing the elements in v
    double limit = 2.0*range;

    for (vector<keys_box>::size_type i = 0; i < other_locations.size(); ++i)
    {
        // If the location is within twice the range of the v location
        if (location.second.distance(other_locations[i].second) <= limit)
        {
            //int num_before = other_locations[i].first.size();
            //cout << i << " original # " << num_before << endl;
            vector<OcTreeKey> overlap (location.first.size() + other_locations[i].first.size());
            vector<OcTreeKey>::iterator int_it = set_intersection (other_locations[i].first.begin(), other_locations[i].first.end(),
                                                                   location.first.begin(), location.first.end(),
                                                                   overlap.begin(), compare_octree_key);
            overlap.resize(int_it - overlap.begin());
            // Remove the overlapping elements from other_locations[i] by taking the difference
            vector<OcTreeKey> remaining (overlap.size() + other_locations[i].first.size());
            vector<OcTreeKey>::iterator diff_itr = set_difference (other_locations[i].first.begin(), other_locations[i].first.end(),
                                                                   overlap.begin(), overlap.end(),
                                                                   remaining.begin(), compare_octree_key);
            remaining.resize(diff_itr - remaining.begin());
            other_locations[i].first = remaining;
//            cout << i << " updated  # " << other_locations[i].first.size() << endl;
//            if (num_before - other_locations[i].first.size() != 0)
//                cout << "DIFFERENCE " << num_before - other_locations[i].first.size() << endl;
        }
    }
    // Sort the result
    sort(other_locations.begin(), other_locations.end(), compare_keys_box);
    // Reverse
    reverse(other_locations.begin(), other_locations.end());
}

void remove_keys(const keys_box &location, vector<OcTreeKey> &key_map)
{
    // Remove elements in second that are found in first
    //cout << "Map original # " << key_map.size() << endl;

    // Find the matching keys
    vector<OcTreeKey> overlap (location.first.size() + key_map.size());
    vector<OcTreeKey>::iterator int_it = set_intersection (key_map.begin(), key_map.end(),
                                                           location.first.begin(), location.first.end(),
                                                           overlap.begin(), compare_octree_key);
    overlap.resize (int_it - overlap.begin());
    // Remove the overlapping keys
    vector<OcTreeKey> remaining (overlap.size() + key_map.size());
    vector<OcTreeKey>::iterator diff_itr = set_difference (key_map.begin(), key_map.end(),
                                                           overlap.begin(), overlap.end(),
                                                           remaining.begin(), compare_octree_key);
    remaining.resize(diff_itr - remaining.begin());
    // Set the output
    key_map = remaining;
}

vector<point3d> get_coverage_locations(const OcTree &tree, const string &dir, const robot_parameters &robot,
                                       const double &step, const int &max_iters, const unsigned int &depth)
{
    vector<point3d> result;

    ROS_WARN("octomap_utils::compute_visible_locations : computing visiblitlity");
    unsigned int tree_depth = depth;
    if (depth <= 0)
    {
        ROS_WARN("octomap_utils::compute_visible_locations : input depth of %u is invalid", depth);
        tree_depth = tree.getTreeDepth();
    }
    if (depth >= tree.getTreeDepth())
    {
        ROS_WARN("octomap_utils::compute_visible_locations : input depth of %u is already maximum", depth);
        tree_depth = tree.getTreeDepth();
    }

    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    OcTree expanded_tree (tree);
    expanded_tree.expand();
    expanded_tree.getMetricMin(x0, y0, z0);
    expanded_tree.getMetricMax(x1, y1, z1);

    vector<OcTreeKey> tree_keys;
    vector<keys_box> valid_cubes;
    bool valid_directory = false;
    if (dir.size() > 0)
    {
        // Check the directory exists
        struct stat st;
        if (stat(dir.c_str(),&st) == 0)
        {
            if (st.st_mode & S_IFDIR != 0)
                valid_directory = true;
            else
                ROS_WARN("octomap_utils::get_coverage_locations : %s is not a valid directory to save files", dir.c_str());
        }

        int ix = -1;
        if (!load_precomputed_data(dir, tree_depth, robot, step, ix))
        {
            if (!compute_visible_locations(tree, robot, step, tree_keys, valid_cubes, tree_depth))
                return result;
            ix = save_visible_locations(dir, tree_keys, valid_cubes, robot, step, tree_depth);
        }
        // If index is valid
        if (ix < 0)
            return result;

        // Load the precomputed tree keys and view keys
        string treefile = append_backslash_to_path(dir) + _TREE_KEYS_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
        vector<keys_box> tree_keys_box = read_keys_box(treefile);
        tree_keys = tree_keys_box[0].first;
        string viewfile = append_backslash_to_path(dir) + _VIEW_KEYS_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
        valid_cubes = read_keys_box(viewfile);
        //cout << "Number of valid cubes " << valid_cubes.size() << endl;
        sort(valid_cubes.begin(), valid_cubes.end(), compare_keys_box);
        reverse(valid_cubes.begin(), valid_cubes.end());
    }
    // If the directory it not valid then just compute the locations without saving
    if (!valid_directory)
        compute_visible_locations(tree, robot, step, tree_keys, valid_cubes, depth);

    vector<keys_box> coverage_path = optimize_coverage_set(tree_keys, valid_cubes, robot.outer_range, max_iters);
    // Unscale the locations
    for (size_t i = 0; i < coverage_path.size(); ++i)
        result.push_back(point3d(coverage_path[i].second.x(), coverage_path[i].second.y(), coverage_path[i].second.z()));

    return result;
}

void octree_visualize(const OcTree &tree, const robot_parameters &robot, const unsigned int &depth)
{
    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Num leaf nodes before : %lu", tree.getNumLeafNodes());
    ROS_INFO("Resolution : %.2f", tree.getResolution());
    OcTree new_tree (tree);
    new_tree.getMetricMin(x0, y0, z0);
    new_tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Metric min : x = %.2f, y = %.2f, z = %.2f", x0, y0, z0);
    ROS_INFO("Metric max : x = %.2f, y = %.2f, z = %.2f", x1, y1, z1);
    ROS_INFO("Volume : %2f", new_tree.volume());
    ROS_INFO("Num leaf nodes : %lu", new_tree.getNumLeafNodes());

    PointCloud<PointT>::Ptr occupied_cloud (new PointCloud<PointT>(octree_to_cloud(new_tree, depth)));

    // ground map
    vector<int> ground_indices;
    PointCloud<PointT>::Ptr ground_cloud (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr nonground_cloud (new PointCloud<PointT>());
    get_octree_ground(*occupied_cloud, ground_indices, *ground_cloud, *nonground_cloud);
    PointT min_gr, max_gr, min_ngr, max_ngr;
    getMinMax3D (*ground_cloud, min_gr, max_gr);
    getMinMax3D (*nonground_cloud, min_ngr, max_ngr);

    // 2d map and min-max map
    PointCloud<PointT>::Ptr inrange_cloud (new PointCloud<PointT>());
    inrange_cloud->resize(occupied_cloud->size());
    int n_size = 0;
    for (size_t i = 0; i < occupied_cloud->size(); ++i)
    {
        if (occupied_cloud->points[i].z >= (max_gr.data[2]+0.2) &&
            occupied_cloud->points[i].z <= (max_gr.data[2]+robot.height))
        {
            inrange_cloud->points[n_size].x = occupied_cloud->points[i].x;
            inrange_cloud->points[n_size].y = occupied_cloud->points[i].y;
            inrange_cloud->points[n_size].z = occupied_cloud->points[i].z;
            ++n_size;
        }
    }
    inrange_cloud->resize(n_size);

//    // Print out information
//    cout << "Num occupied cells " << occupied_cloud->size() << endl;
//    cout << "Num ground cells " << ground_cloud->size() << endl;
//    cout << "Num nonground cells " << nonground_cloud->size() << endl;
//    cout << "Num in range cells " << inrange_cloud->size() << endl;

//    // Downsample the point clouds before viewing
//    VoxelGrid<PointT> sor;
//    sor.setLeafSize (0.25f, 0.25f, 0.25f);
//    sor.setInputCloud (occupied_cloud);
//    sor.filter (*occupied_cloud);
//    sor.setInputCloud (ground_cloud);
//    sor.filter (*ground_cloud);
//    sor.setInputCloud (nonground_cloud);
//    sor.filter (*nonground_cloud);
//    sor.setInputCloud (inrange_cloud);
//    sor.filter (*inrange_cloud);

    // Use pcl visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer("Octree");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    int vox_size = ceil(7 + (double)tree.getTreeDepth()/(double)depth);
    string f = "occupied voxels";
    visualization::PointCloudColorHandlerCustom<PointT> occupied_handler (occupied_cloud, 20, 20, 230);  // Blue
    viewer->addPointCloud (occupied_cloud, occupied_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    f = "ground voxels";
    visualization::PointCloudColorHandlerCustom<PointT> ground_handler (ground_cloud, 20, 230, 230);  // Cyan
    viewer->addPointCloud (ground_cloud, ground_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

//    f = "nonground voxels";
//    visualization::PointCloudColorHandlerCustom<PointT> nonground_handler (nonground_cloud, 230, 230, 20);  // Yellow
//    viewer->addPointCloud (nonground_cloud, nonground_handler, f);
//    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, f);

//    f = "inrange voxels";
//    visualization::PointCloudColorHandlerCustom<PointT> inrange_handler (inrange_cloud, 230, 20, 20);  // Red
//    viewer->addPointCloud (inrange_cloud, inrange_handler, f);
//    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 6, f);

    // Add boundingboxes
    viewer->addCube(x0, x1, y0, y1, z0, z1, 0.0, 0.0, 1.0, "bounding box");
    viewer->addCube(min_gr.data[0], max_gr.data[0], min_gr.data[1], max_gr.data[1], min_gr.data[2], max_gr.data[2],
                    0.0, 1.0, 1.0, "bounding box ground");
    viewer->addCube(min_ngr.data[0], max_ngr.data[0], min_ngr.data[1], max_ngr.data[1], min_ngr.data[2], max_ngr.data[2],
                    1.0, 1.0, 0.0, "bounding box nonground");

    // Wait for the viewer to close before exiting
    while (!viewer->wasStopped ())
        viewer->spinOnce ();

    delete viewer;
}

void octree_visualize_grid(const OcTree &tree, const robot_parameters &robot, const double &step, const unsigned int &depth)
{
    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Num leaf nodes before : %lu", tree.getNumLeafNodes());
    OcTree expanded_tree (tree);
    expanded_tree.expand();
    //tree.read(filename);
    ROS_INFO("Resolution : %.2f", expanded_tree.getResolution());
    expanded_tree.getMetricMin(x0, y0, z0);
    expanded_tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Metric min : x = %.2f, y = %.2f, z = %.2f", x0, y0, z0);
    ROS_INFO("Metric max : x = %.2f, y = %.2f, z = %.2f", x1, y1, z1);
    ROS_INFO("Volume : %2f", expanded_tree.volume());
    ROS_INFO("Num leaf nodes : %lu", expanded_tree.getNumLeafNodes());

    // Get the sampled grid
    vector<point3d> cubes = exhaustive_coverage_set(expanded_tree, robot, step, depth);
    //cout << "Num cubes " << cubes.size() << endl;

    // Convert tree to visible point cloud
    PointCloud<PointT> occupied_cloud = octree_to_cloud(expanded_tree, depth);
    double zground = octree_ground_height(expanded_tree, depth);
    double zmin = zground  + _GROUND_THRESH;
    double zmax = zground + robot.height;
    double z = zmax;
    double zheight = zmin + octree_resolution_at_depth(tree, depth);

    // Min-max map
    PointCloud<PointT>::Ptr inrange_cloud (new PointCloud<PointT>());
    inrange_cloud->resize(occupied_cloud.size());
    int n_size = 0;
    for (size_t i = 0; i < occupied_cloud.size(); ++i)
    {
        if (occupied_cloud.points[i].z >= zmin && occupied_cloud.points[i].z <= z)
        {
            inrange_cloud->points[n_size].x = occupied_cloud.points[i].x;
            inrange_cloud->points[n_size].y = occupied_cloud.points[i].y;
            inrange_cloud->points[n_size].z = occupied_cloud.points[i].z;
            ++n_size;
        }
    }
    inrange_cloud->resize(n_size);
    PointT minr, maxr;
    getMinMax3D (*inrange_cloud, minr, maxr);
    //cout << "minr z " << minr.data[2] << endl;
    //cout << "maxr z " << maxr.data[2] << endl;

    // Floor
    PointCloud<PointT>::Ptr floor_cloud (new PointCloud<PointT>());
    floor_cloud->resize(expanded_tree.size());
    int counter = 0;
    for (OcTree::leaf_iterator it = expanded_tree.begin_leafs(depth), end = expanded_tree.end_leafs(); it!= end; ++it)
    {
        // Get the point and check it is within the limits and is not occupied
        point3d pt = it.getCoordinate();
        if (pt.z() >= zmin && pt.z() <= zheight && it->getOccupancy() < 0.5)
        {
            floor_cloud->points[counter].x = pt.x();
            floor_cloud->points[counter].y = pt.y();
            floor_cloud->points[counter].z = pt.z();
            ++counter;
        }
    }
    floor_cloud->resize(counter);

    // Convert the sampled grid to locations
    PointCloud<PointT>::Ptr positions (new PointCloud<PointT>());
    positions->resize(cubes.size());
    for (vector<point3d>::size_type i = 0; i < cubes.size(); ++i)
    {
        positions->points[i].x = cubes[i].x();
        positions->points[i].y = cubes[i].y();
        positions->points[i].z = cubes[i].z();
    }

    // Use pcl visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer("Octree");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    string f = "inrange voxels";
    int vox_size = ceil(14 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> inrange_handler (inrange_cloud, 20, 20, 230);  // Blue
    viewer->addPointCloud (inrange_cloud, inrange_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    f = "floor";
    vox_size = ceil(6 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> floor_handler (floor_cloud, 20, 230, 230);  // Cyan
    viewer->addPointCloud (floor_cloud, floor_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    // Add boundingboxes
    viewer->addCube(minr.data[0], maxr.data[0], minr.data[1], maxr.data[1], minr.data[2], maxr.data[2], 0.0, 0.0, 1.0, "bounding box");

//    for (size_t i = 0; i < cubes.size(); ++i)
//    {
//        f = "view " + boost::lexical_cast<string>(i);
//        viewer->addCube(cubes[i].x()-robot.outer_range, cubes[i].x()+robot.outer_range,
//                        cubes[i].y()-robot.outer_range, cubes[i].y()+robot.outer_range,
//                        zmin, zmax,
//                        0.0, 1.0, 0.0, f);
//    }

    f = "positions";
    vox_size = 15;
    visualization::PointCloudColorHandlerCustom<PointT> positions_handler (positions, 20, 230, 20);  // Green
    viewer->addPointCloud (positions, positions_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    // Wait for the viewer to close before exiting
    while (!viewer->wasStopped ())
        viewer->spinOnce ();

    delete viewer;
}

void octree_visualize_location(const OcTree &tree, const robot_parameters &robot, const unsigned int &depth)
{
    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Num leaf nodes before : %lu", tree.getNumLeafNodes());
    OcTree expanded_tree (tree);
    expanded_tree.expand();
    ROS_INFO("Resolution : %.2f", expanded_tree.getResolution());
    expanded_tree.getMetricMin(x0, y0, z0);
    expanded_tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Metric min : x = %.2f, y = %.2f, z = %.2f", x0, y0, z0);
    ROS_INFO("Metric max : x = %.2f, y = %.2f, z = %.2f", x1, y1, z1);
    ROS_INFO("Volume : %2f", expanded_tree.volume());
    ROS_INFO("Num leaf nodes : %lu", expanded_tree.getNumLeafNodes());

    // Convert tree to visible point cloud
    PointCloud<PointT> occupied_cloud = octree_to_cloud(expanded_tree, depth);
    double zground = octree_ground_height(expanded_tree, depth);
    double zmin = zground  + _GROUND_THRESH;
    double zmax = zground + robot.height;
    double zheight = zmin + octree_resolution_at_depth(expanded_tree, depth);
    //cout << "ZMIN " << zmin << endl;
    //cout << "ZHEIGHT " << zheight << endl;

    // Min-max map
    PointCloud<PointT>::Ptr inrange_cloud (new PointCloud<PointT>());
    inrange_cloud->resize(occupied_cloud.size());
    int n_size = 0;
    for (size_t i = 0; i < occupied_cloud.size(); ++i)
    {
        if (occupied_cloud.points[i].z >= zmin && occupied_cloud.points[i].z <= zmax)
        {
            inrange_cloud->points[n_size].x = occupied_cloud.points[i].x;
            inrange_cloud->points[n_size].y = occupied_cloud.points[i].y;
            inrange_cloud->points[n_size].z = occupied_cloud.points[i].z;
            ++n_size;
        }
    }
    inrange_cloud->resize(n_size);
    PointT minr, maxr;
    getMinMax3D (*inrange_cloud, minr, maxr);
    //cout << "minr z " << minr.data[2] << endl;
    //cout << "maxr z " << maxr.data[2] << endl;

    // Floor
    PointCloud<PointT>::Ptr floor_cloud (new PointCloud<PointT>());
    floor_cloud->resize(expanded_tree.size());
    int counter = 0;
    for (OcTree::leaf_iterator it = expanded_tree.begin_leafs(depth), end = expanded_tree.end_leafs(); it!= end; ++it)
    {
        // Get the point and check it is within the limits and is not occupied
        point3d pt = it.getCoordinate();
        if (pt.z() >= zmin && pt.z() <= zheight && it->getOccupancy() < 0.5)
        {
            floor_cloud->points[counter].x = pt.x();
            floor_cloud->points[counter].y = pt.y();
            floor_cloud->points[counter].z = pt.z();
            ++counter;
        }
    }
    floor_cloud->resize(counter);

    // Get visible voxels from the input test location
    //cout << "zmin " << zmin << endl;
    //cout << "zmax " << zmax << endl;
    // Define the location
    double x = x0+(x1-x0)/2.0 + 2.0;
    double y = y0+(y1-y0)/2.0 + 2.0;
    point3d origin (x,y,zmax);
    vector<OcTreeKey> vis = visible_voxels(expanded_tree, origin, robot, zmin, zheight, depth);
    PointCloud<PointT>::Ptr visible_cloud (new PointCloud<PointT>());
    visible_cloud->resize(vis.size());
    for (size_t i = 0; i < vis.size(); ++i)
    {
        point3d p = expanded_tree.keyToCoord(vis[i]);
        visible_cloud->points[i].x = p.x();
        visible_cloud->points[i].y = p.y();
        visible_cloud->points[i].z = p.z();
    }
    //cout << "Number of voxels visible from location " << vis.size() << endl;

    // Use pcl visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer("Octree");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    string f = "inrange voxels";
    int vox_size = ceil(14 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> inrange_handler (inrange_cloud, 20, 20, 230);  // Blue
    viewer->addPointCloud (inrange_cloud, inrange_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    f = "floor";
    vox_size = ceil(6 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> floor_handler (floor_cloud, 20, 230, 230);  // Cyan
    viewer->addPointCloud (floor_cloud, floor_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    f = "view";
    vox_size = 15;
    PointCloud<PointT>::Ptr view (new PointCloud<PointT>());
    view->resize(1);
    view->points[0].x = x;
    view->points[0].y = y;
    view->points[0].z = zmax;
    visualization::PointCloudColorHandlerCustom<PointT> viewpt_handler (view, 230, 20, 20);  // Red
    viewer->addPointCloud (view, viewpt_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    vox_size = ceil(14 + (double)tree.getTreeDepth()/(double)depth);
    if (visible_cloud->size() > 0)
    {
        f = "visible voxels";
        visualization::PointCloudColorHandlerCustom<PointT> visible_handler (visible_cloud, 230, 20, 230);  // Magenta
        viewer->addPointCloud (visible_cloud, visible_handler, f);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);
    }

    // Add boundingboxes
    viewer->addCube(minr.data[0], maxr.data[0], minr.data[1], maxr.data[1], minr.data[2], maxr.data[2], 0.0, 0.0, 1.0, "bounding box");
    viewer->addCube(x-robot.outer_range, x+robot.outer_range,
                    y-robot.outer_range, y+robot.outer_range,
                    zground, zmax,
                    1.0, 0.0, 0.0, "bounding box robot");

    // Wait for the viewer to close before exiting
    while (!viewer->wasStopped ())
        viewer->spinOnce ();

    delete viewer;
}

void octree_visualize_minimal_overlap(const OcTree &tree, const string &dir, const robot_parameters &robot, const double &step,
                                      int const &max_iters, const unsigned int &depth)
{
    ROS_WARN("octomap_utils::compute_visible_locations : computing visiblitlity");
    unsigned int tree_depth = depth;
    if (depth <= 0)
    {
        ROS_WARN("octomap_utils::compute_visible_locations : input depth of %u is invalid", depth);
        tree_depth = tree.getTreeDepth();
    }
    if (depth >= tree.getTreeDepth())
    {
        ROS_WARN("octomap_utils::compute_visible_locations : input depth of %u is already maximum", depth);
        tree_depth = tree.getTreeDepth();
    }

    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Num leaf nodes before : %lu", tree.getNumLeafNodes());
    OcTree expanded_tree (tree);
    expanded_tree.expand();
    ROS_INFO("Resolution : %.2f", expanded_tree.getResolution());
    expanded_tree.getMetricMin(x0, y0, z0);
    expanded_tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Metric min : x = %.2f, y = %.2f, z = %.2f", x0, y0, z0);
    ROS_INFO("Metric max : x = %.2f, y = %.2f, z = %.2f", x1, y1, z1);
    ROS_INFO("Volume : %2f", expanded_tree.volume());
    ROS_INFO("Num leaf nodes : %lu", expanded_tree.getNumLeafNodes());

    PointCloud<PointT> occupied_cloud = octree_to_cloud(expanded_tree, tree_depth);
    double zground = octree_ground_height(expanded_tree, depth);
    double zmin = zground  + _GROUND_THRESH;
    double zmax = zground + robot.height;
    double zheight = zmin + octree_resolution_at_depth(expanded_tree, tree_depth);

    // Min-max map
    PointCloud<PointT>::Ptr inrange_cloud (new PointCloud<PointT>());
    inrange_cloud->resize(occupied_cloud.size());
    int n_size = 0;
    for (size_t i = 0; i < occupied_cloud.size(); ++i)
    {
        if (occupied_cloud.points[i].z >= zmin && occupied_cloud.points[i].z <= zmax)
        {
            inrange_cloud->points[n_size].x = occupied_cloud.points[i].x;
            inrange_cloud->points[n_size].y = occupied_cloud.points[i].y;
            inrange_cloud->points[n_size].z = occupied_cloud.points[i].z;
            ++n_size;
        }
    }
    inrange_cloud->resize(n_size);
    PointT minr, maxr;
    getMinMax3D (*inrange_cloud, minr, maxr);

    vector<point3d> cubes = exhaustive_coverage_set(expanded_tree, robot, step, tree_depth);
    // Convert the sampled grid to locations
    PointCloud<PointT>::Ptr positions (new PointCloud<PointT>());
    positions->resize(cubes.size());
    for (vector<point3d>::size_type i = 0; i < cubes.size(); ++i)
    {
        positions->points[i].x = cubes[i].x();
        positions->points[i].y = cubes[i].y();
        positions->points[i].z = cubes[i].z();
    }

    vector<OcTreeKey> tree_keys;
    vector<keys_box> valid_cubes;
    int ix = -1;
    if (!load_precomputed_data(dir, tree_depth, robot, step, ix))
    {
        if (!compute_visible_locations(tree, robot, step, tree_keys, valid_cubes, tree_depth))
            return;
        ix = save_visible_locations(dir, tree_keys, valid_cubes, robot, step, tree_depth);
    }
    // If index is valid
    if (ix < 0)
        return;

    // Load the precomputed tree keys and view keys
    string treefile = append_backslash_to_path(dir) + _TREE_KEYS_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
    vector<keys_box> tree_keys_box = read_keys_box(treefile);
    tree_keys = tree_keys_box[0].first;
    string viewfile = append_backslash_to_path(dir) + _VIEW_KEYS_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
    valid_cubes = read_keys_box(viewfile);
    //cout << "Number of valid cubes " << valid_cubes.size() << endl;
    sort(valid_cubes.begin(), valid_cubes.end(), compare_keys_box);
    reverse(valid_cubes.begin(), valid_cubes.end());

    vector<keys_box> coverage_path = optimize_coverage_set(tree_keys, valid_cubes, robot.outer_range, max_iters);
    vector<point3d> path_locations;
    vector<PointCloud<PointT> > path_visible_voxels;
    for (size_t i = 0; i < coverage_path.size(); ++i)
    {
        // The location
        path_locations.push_back(point3d(coverage_path[i].second.x(),
                                         coverage_path[i].second.y(),
                                         coverage_path[i].second.z()));
        // The visible points
        PointCloud<PointT> vis_pts;
        vis_pts.resize(coverage_path[i].first.size());
        for (size_t j = 0; j < coverage_path[i].first.size(); ++j)
        {
            // Get the point from the tree
            point3d p = expanded_tree.keyToCoord(coverage_path[i].first[j]);
            vis_pts.points[j].x = p.x();
            vis_pts.points[j].y = p.y();
            vis_pts.points[j].z = zmin;  // Why doesn't p.z() work?
        }
        path_visible_voxels.push_back(vis_pts);
    }

    // Floor
    PointCloud<PointT>::Ptr floor_cloud (new PointCloud<PointT>());
    floor_cloud->resize(expanded_tree.size());
    int counter = 0;
    for (OcTree::leaf_iterator it = expanded_tree.begin_leafs(depth), end = expanded_tree.end_leafs(); it!= end; ++it)
    {
        // Get the point and check it is within the limits and is not occupied
        point3d pt = it.getCoordinate();
        if (pt.z() >= zmin && pt.z() <= zheight && it->getOccupancy() < 0.5)
        {
            floor_cloud->points[counter].x = pt.x();
            floor_cloud->points[counter].y = pt.y();
            floor_cloud->points[counter].z = pt.z();
            ++counter;
        }
    }
    floor_cloud->resize(counter);

    // Use pcl visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer("Octree");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    string f = "inrange voxels";
    int vox_size = ceil(14 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> inrange_handler (inrange_cloud, 20, 20, 230);  // Blue
    viewer->addPointCloud (inrange_cloud, inrange_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    // Add bounding box
    viewer->addCube(minr.data[0], maxr.data[0], minr.data[1], maxr.data[1], minr.data[2], maxr.data[2], 0.0, 0.0, 1.0, "bounding box");

    f = "floor";
    vox_size = ceil(6 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> floor_handler (floor_cloud, 20, 230, 230);  // Cyan
    viewer->addPointCloud (floor_cloud, floor_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    f = "positions";
    vox_size = 15;
    visualization::PointCloudColorHandlerCustom<PointT> positions_handler (positions, 20, 230, 20);  // Green
    viewer->addPointCloud (positions, positions_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    // Threads to simultaneously 1) keep viewer open and 2) display cloud and wait for next input to update the clouds
    boost::thread view_thread = boost::thread (&run_viewer, viewer);
    boost::thread location_update_thread = boost::thread(&run_view_locations, viewer, path_locations, path_visible_voxels,
                                                         robot.outer_range, zmin, zmax);
    // Join the threads
    view_thread.join();
    location_update_thread.join();
    // Finished, wait for thread to exit
    ROS_INFO("octomap_utils::octree_visualize_minimal_overlap : finished viewing");
    // Close the window
    delete viewer;
}

void octree_visualize_segments(const OcTree &tree, const vector<vector<OcTreeKey> > &segments)
{
    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    ROS_INFO("Num leaf nodes before : %lu", tree.getNumLeafNodes());
    ROS_INFO("Resolution : %.2f", tree.getResolution());
    ROS_INFO("Metric min : x = %.2f, y = %.2f, z = %.2f", x0, y0, z0);
    ROS_INFO("Metric max : x = %.2f, y = %.2f, z = %.2f", x1, y1, z1);
    ROS_INFO("Num leaf nodes : %lu", tree.getNumLeafNodes());

    PointCloud<PointT>::Ptr occupied_cloud (new PointCloud<PointT>(octree_to_cloud(tree)));

    // Get points for all segments

    // Use pcl visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer("Octree");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    string f = "occupied voxels";
    visualization::PointCloudColorHandlerCustom<PointT> occupied_handler (occupied_cloud, 20, 20, 230);  // Blue
    viewer->addPointCloud (occupied_cloud, occupied_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 8, f);

    for (size_t i = 0; i < segments.size(); ++i)
    {
        // Create a point cloud
        PointCloud<PointT>::Ptr seg_cloud (new PointCloud<PointT>());
        seg_cloud->resize(segments[i].size());
        for (size_t j = 0; j < segments[i].size(); ++j)
        {
            point3d p = tree.keyToCoord(segments[i][j]);
            seg_cloud->points[j].x = p.x();
            seg_cloud->points[j].y = p.y();
            seg_cloud->points[j].z = p.z();
        }
        f = "occupied voxels " + boost::lexical_cast<string>(i);
        visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg_cloud, 230, 20, 20);  // Red
        viewer->addPointCloud (seg_cloud, seg_handler, f);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 8, f);
        // Add boundingboxes
        PointT minr, maxr;
        getMinMax3D (*seg_cloud, minr, maxr);
        f = "bounding box " + boost::lexical_cast<string>(i);
        viewer->addCube(minr.data[0], maxr.data[0],
                        minr.data[1], maxr.data[1],
                        minr.data[2], maxr.data[2],
                        1.0, 0.0, 1.0, f);
    }

    // Wait for the viewer to close before exiting
    while (!viewer->wasStopped ())
        viewer->spinOnce ();

    delete viewer;
}

void visualize_coverage(const OcTree &tree, const robot_parameters &robot, const double& step, vector<point3d> &locations, const unsigned int &depth)
{
    double x0, y0, z0;
    double x1, y1, z1;
    tree.getMetricMin(x0, y0, z0);
    tree.getMetricMax(x1, y1, z1);
    OcTree expanded_tree (tree);
    expanded_tree.expand();
    expanded_tree.getMetricMin(x0, y0, z0);
    expanded_tree.getMetricMax(x1, y1, z1);

    PointCloud<PointT> occupied_cloud = octree_to_cloud(expanded_tree, depth);
    double zground = octree_ground_height(expanded_tree, depth);
    double zmin = zground  + _GROUND_THRESH;
    double zmax = zground + robot.height;
    double zheight = zmin + octree_resolution_at_depth(expanded_tree, depth);

    // Min-max map
    PointCloud<PointT>::Ptr inrange_cloud (new PointCloud<PointT>());
    inrange_cloud->resize(occupied_cloud.size());
    int n_size = 0;
    for (size_t i = 0; i < occupied_cloud.size(); ++i)
    {
        if (occupied_cloud.points[i].z >= zmin && occupied_cloud.points[i].z <= zmax)
        {
            inrange_cloud->points[n_size].x = occupied_cloud.points[i].x;
            inrange_cloud->points[n_size].y = occupied_cloud.points[i].y;
            inrange_cloud->points[n_size].z = occupied_cloud.points[i].z;
            ++n_size;
        }
    }
    inrange_cloud->resize(n_size);
    PointT minr, maxr;
    getMinMax3D (*inrange_cloud, minr, maxr);

    vector<point3d> cubes = exhaustive_coverage_set(expanded_tree, robot, step, depth);
    // Convert the sampled grid to locations
    PointCloud<PointT>::Ptr positions (new PointCloud<PointT>());
    positions->resize(cubes.size());
    for (vector<point3d>::size_type i = 0; i < cubes.size(); ++i)
    {
        positions->points[i].x = cubes[i].x();
        positions->points[i].y = cubes[i].y();
        positions->points[i].z = cubes[i].z();
    }

    // Convert the locations to a visible point cloud
    PointCloud<PointT>::Ptr coverage_locations (new PointCloud<PointT>());
    coverage_locations->resize(locations.size());
    for (vector<point3d>::size_type i = 0; i < locations.size(); ++i)
    {
        coverage_locations->points[i].x = locations[i].x();
        coverage_locations->points[i].y = locations[i].y();
        coverage_locations->points[i].z = locations[i].z();
    }

    // Floor
    PointCloud<PointT>::Ptr floor_cloud (new PointCloud<PointT>());
    floor_cloud->resize(expanded_tree.size());
    int counter = 0;
    for (OcTree::leaf_iterator it = expanded_tree.begin_leafs(depth), end = expanded_tree.end_leafs(); it!= end; ++it)
    {
        // Get the point and check it is within the limits and is not occupied
        point3d pt = it.getCoordinate();
        if (pt.z() >= zmin && pt.z() <= zheight && it->getOccupancy() < 0.5)
        {
            floor_cloud->points[counter].x = pt.x();
            floor_cloud->points[counter].y = pt.y();
            floor_cloud->points[counter].z = pt.z();
            ++counter;
        }
    }
    floor_cloud->resize(counter);

    // Use pcl visualization
    visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer("Octree");
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    string f = "inrange voxels";
    int vox_size = ceil(14 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> inrange_handler (inrange_cloud, 20, 20, 230);  // Blue
    viewer->addPointCloud (inrange_cloud, inrange_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    // Add bounding box
    viewer->addCube(minr.data[0], maxr.data[0], minr.data[1], maxr.data[1], minr.data[2], maxr.data[2], 0.0, 0.0, 1.0, "bounding box");

    f = "floor";
    vox_size = ceil(6 + (double)tree.getTreeDepth()/(double)depth);
    visualization::PointCloudColorHandlerCustom<PointT> floor_handler (floor_cloud, 20, 230, 230);  // Cyan
    viewer->addPointCloud (floor_cloud, floor_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    f = "positions";
    vox_size = 15;
    visualization::PointCloudColorHandlerCustom<PointT> positions_handler (positions, 20, 230, 20);  // Green
    viewer->addPointCloud (positions, positions_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    // The locations and their bounding boxes
    f = "coverage locations";
    vox_size = 15;
    visualization::PointCloudColorHandlerCustom<PointT> coverage_locations_handler (coverage_locations, 230, 20, 20);  // Red
    viewer->addPointCloud (coverage_locations, coverage_locations_handler, f);
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, vox_size, f);

    for (vector<point3d>::size_type i = 0; i < locations.size(); ++i)
    {
        f = "coverage location bounding box " + boost::lexical_cast<string>(i);
        viewer->addCube(locations[i].x()-robot.outer_range, locations[i].x()+robot.outer_range,
                        locations[i].y()-robot.outer_range, locations[i].y()+robot.outer_range,
                        zmin, zmax,
                        1.0, 0.0, 0.0, f);
    }

    // Wait for the viewer to close before exiting
    while (!viewer->wasStopped ())
        viewer->spinOnce ();

    delete viewer;
}

void run_view_locations(visualization::PCLVisualizer *viewer, vector<point3d> &coverage_path, vector<PointCloud<PointT> > &path_vis,
                        const double &range, const double &zmin, const double &zmax)
{
    ROS_INFO("octomap_utils::run_view_locations : starting");

    string f;
    string input;
    int i = 0;
    while (i < coverage_path.size())
    {
        // Wait for input
        cout << "Display location " << i+1 << "/" << coverage_path.size() << " [y/n]? ";
        cin >> input;
        // If "y" input
        if (input.compare("y") == 0 || input.compare("yes") == 0 ||
            input.compare("Y") == 0 || input.compare("YES") == 0)
        {
            // Get the point into a point cloud
            PointCloud<PointT>::Ptr pos (new PointCloud<PointT>());
            pos->resize(1);
            pos->points[0].x = coverage_path[i].x();
            pos->points[0].y = coverage_path[i].y();
            pos->points[0].z = coverage_path[i].z();
            // Display the point
            f = "coverage position" + boost::lexical_cast<string>(i);
            visualization::PointCloudColorHandlerCustom<PointT> position_handler (pos, 230, 20, 20);  // Red
            viewer->addPointCloud (pos, position_handler, f);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 15, f);
            // Display the bounding box
            f = "pos box " + boost::lexical_cast<string>(i);
            viewer->addCube(coverage_path[i].x()-range, coverage_path[i].x()+range,
                            coverage_path[i].y()-range, coverage_path[i].y()+range,
                            zmin, zmax,
                            1.0, 0.0, 0.0, f);  // Red
            // Display the floor points it covers
            f = "coverage points" + boost::lexical_cast<string>(i);
            PointCloud<PointT>::Ptr vis_pts_ptr (new PointCloud<PointT>(path_vis[i]));
            visualization::PointCloudColorHandlerCustom<PointT> points_handler (vis_pts_ptr, 230, 20, 230);  // Magenta
            viewer->addPointCloud (vis_pts_ptr, points_handler, f);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 15, f);
            // Increment
            ++i;
        }
        // If "n" input
        else if (input.compare("n") == 0 || input.compare("no") == 0 ||
                 input.compare("N") == 0 || input.compare("NO") == 0)
        {
            ROS_WARN("octomap_utils::run_view_locations : skipping location %u", i);
        }
        else if (input.compare("q") == 0 || input.compare("quit") == 0 ||
                 input.compare("Q") == 0 || input.compare("QUIT") == 0)
        {
            ROS_WARN("octomap_utils::run_view_locations : quitting");
            break;
        }
    }

    ROS_INFO("octomap_utils::run_view_locations : finished");
}

void run_viewer(visualization::PCLVisualizer *viewer)
{
    ROS_INFO("octomap_utils::run_viewer : starting");
    // Display the visualiser until 'q' key is pressed
    while (!viewer->wasStopped ())
        viewer->spinOnce ();
    ROS_INFO("octomap_utils::run_viewer : finished");
}

/* === IO === */

int save_visible_locations(const string &dir, const vector<OcTreeKey> &tree_keys, const vector<keys_box> &locations,
                           const robot_parameters &robot, const double &step, const unsigned int &depth)
{
    // Get all the config files
    vector<int> indices = get_all_config_files(dir);
    // Get a new index that hasn't been used already
    int ix = 0;
    while (true)
    {
        // If this index is not used
        if (find(indices.begin(), indices.end(), ix) == indices.end())
            break;
        // Otherwise continue
        ++ix;
    }
    ROS_INFO("octomap_utils::save_visible_locations : saving data to %s with index %u", dir.c_str(), ix);

    // Save the tree keys
    string save_tree_file = append_backslash_to_path(dir) + _TREE_KEYS_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
    vector<keys_box> fake_tree;
    fake_tree.push_back(make_pair(tree_keys,point3d(0,0,0)));
    write_keys_box(save_tree_file, fake_tree);

    // Write the keys_box to file
    string save_file = append_backslash_to_path(dir) + _VIEW_KEYS_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
    write_keys_box(save_file, locations);

    // Write the config file
    string save_config_file = append_backslash_to_path(dir) + _CONFIG_FILE + "_" + boost::lexical_cast<string>(ix) + ".txt";
    write_config_file(save_config_file, depth, robot, step);

    // Return the index that these were written to
    return ix;
}

void write_keys_box(const string &filename, const vector<keys_box> &vec)
{
    ofstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        for (vector<keys_box>::size_type i = 0; i < vec.size(); ++i)
        {
            myfile << vec[i].second.x() << " " << vec[i].second.y() << " " << vec[i].second.z() << " ";
            for (vector<OcTreeKey>::size_type j = 0; j < vec[i].first.size(); ++j)
                myfile << vec[i].first[j].k[0] << " " << vec[i].first[j].k[1] << " " << vec[i].first[j].k[0] << " ";
            myfile << "\n";
        }
        myfile.close();
    }
}

void write_config_file(const string &filename, const unsigned int &depth, const robot_parameters &robot, const double &step)
{
    ofstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        myfile << "TREE_DEPTH " << depth << "\n";
        myfile << "ROBOT_HEIGHT " << robot.height << "\n";
        myfile << "ROBOT_OUTER_RANGE " << robot.outer_range << "\n";
        myfile << "ROBOT_INNER_RANGE " << robot.inner_range << "\n";
        myfile << "ROBOT_RADIUS " << robot.radius << "\n";
        myfile << "GRID_STEP " << step << "\n";
        myfile.close();
    }
}

vector<keys_box> read_keys_box(const string &filename)
{
    vector<keys_box> result;
    string line;
    ifstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        while (getline(myfile,line))
        {
            string word;
            stringstream linestream (line);
            int count = 0;
            int k_count = -1;
            double x, y, z;
            int k0, k1, k2;
            vector<OcTreeKey> keys;
            while (linestream >> word)
            {
                if (count == 0)
                    x = atof(word.c_str());
                else if (count == 1)
                    y = atof(word.c_str());
                else if (count == 2)
                    z = atof(word.c_str());
                else if (count == 3)
                    k_count = 0;
                ++count;

                if (k_count >= 0)
                {
                    if (k_count == 0)
                        k0 = atoi(word.c_str());
                    else if (k_count == 1)
                        k1 = atoi(word.c_str());
                    else if (k_count == 2)
                        k2 = atoi(word.c_str());
                    ++k_count;

                    if (k_count == 3)
                    {
                        OcTreeKey k(k0,k1,k2);
                        keys.push_back(k);
                        k_count = 0;  // reset
                    }
                }
            }
            result.push_back(make_pair(keys,point3d(x,y,z)));
        }
        myfile.close();
    }

    return result;
}

void read_config_file(const string &filename, unsigned int &depth, robot_parameters &robot, double &step)
{
    string line;
    ifstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        int count = 0;
        while (getline(myfile,line))
        {
            string word;
            stringstream linestream (line);
            linestream >> word; // the name
            linestream >> word; // the value
            double val = atof(word.c_str());
            if (count == 0)
                depth = (int)val;
            else if (count == 1)
                robot.height = val;
            else if (count == 2)
                robot.outer_range = val;
            else if (count == 3)
                robot.inner_range = val;
            else if (count == 4)
                robot.radius = val;
            else if (count == 5)
                step = val;
            ++count;
            if (count == 6)
                break;
        }
        myfile.close();
    }
}

bool load_precomputed_data(const string &dir, const unsigned int depth, const robot_parameters &robot, const double &step, int &ix)
{
    // Get all available config files
    ix = -1;
    vector<int> indices = get_all_config_files(dir);
    if (indices.size() == 0)
    {
        ROS_WARN("octomap_utils::load_precomputed_data : no precomputed data in directory %s", dir.c_str());
        return false;
    }
    // Check each individual file and see if it matches the queries settings
    for (vector<int>::size_type i = 0; i < indices.size(); ++i)
    {
        // Load the file
        unsigned int in_depth;
        robot_parameters in_robot;
        double in_step;
        string f = append_backslash_to_path(dir) + _CONFIG_FILE + "_" + boost::lexical_cast<string>(i) + ".txt";
        read_config_file(f, in_depth, in_robot, in_step);
        // Compare the values to see if they are the same
        bool success = true;
        if (abs(depth-in_depth) > 0.5)
            success = false;
        if (fabs(robot.height-in_robot.height) > 0.1)
            success = false;
        if (fabs(robot.outer_range-in_robot.outer_range) > 0.1)
            success = false;
        if (fabs(robot.inner_range-in_robot.inner_range) > 0.1)
            success = false;
        if (fabs(robot.radius-in_robot.radius) > 0.1)
            success = false;
        if (fabs(step-in_step) > 0.05)
            success = false;
        // If all values are matching, then return this index
        if (success)
        {
            ix = i;
            ROS_INFO("octomap_utils::load_precomputed_data : found valid data for index %u", ix);
            return true;
        }
    }
    // If made it here then found no valid match
    ROS_WARN("octomap_utils::load_precomputed_data : no matching precomputed data in directory %s", dir.c_str());
    return false;
}

vector<int> get_all_config_files(const string &dir)
{
    vector<int> result;

    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR("octomap_utils::get_all_config_files : could not open %s", dir.c_str());
        return result;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            // If the first letters match _CONFIG_FILE
             if (strncmp(f.c_str(),_CONFIG_FILE, sizeof(_CONFIG_FILE)-1) == 0)
             {
                 // Get the number
                 size_t underscore = f.find_last_of('_');
                 size_t dot = f.find_last_of('.');
                 if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
                 {
                     int str_len = dot - underscore;
                     string str_ix = f.substr(underscore+1,str_len-1);
                     int ix = atoi(str_ix.c_str());
                     result.push_back(ix);
                 }
             }
        }
    }
    return result;
}

OcTree get_tree_from_filename(const string &str, bool &return_is_directory)
{
    // Find if this is a directory or a filename
    bool is_directory = true;
    size_t found = str.find_last_of(".");
    string ext;
    if (found != string::npos)
    {
        ext = str.substr(found+1);
        if (strcmp(ext.c_str(),"bt") == 0 || strcmp(ext.c_str(),"ot") == 0)
            is_directory = false;
    }
    return_is_directory = is_directory;

    // If this is a valid directory
    if (is_directory)
    {
        ROS_WARN("octomap_utils::get_tree_from_filename : input %s is a directory", str.c_str());
        DIR *dp;
        struct dirent *dirp;
        if((dp  = opendir(str.c_str())) == NULL)
        {
            ROS_ERROR("octomap_utils::get_tree_from_directory : could not open %s", str.c_str());
            OcTree tree (0.5);
            return tree;
        }

        while ((dirp = readdir(dp)) != NULL)
        {
            string f = string(dirp->d_name);
            if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
            {
                // If there is an extension
                size_t found = f.find_last_of(".");
                if (found != string::npos)
                {
                    ext = f.substr(found+1);
                    if (strcmp(ext.c_str(),"bt") == 0)
                    {
                        ROS_INFO("octomap_utils::get_tree_from_filename : found tree file %s", f.c_str());
                        string treefile = append_backslash_to_path(str) + f;
                        OcTree tree (treefile);
                        return tree;
                    }
                    else if (strcmp(ext.c_str(),"ot") == 0)
                    {
                        ROS_INFO("octomap_utils::get_tree_from_filename : found tree file %s", f.c_str());
                        string treefile = append_backslash_to_path(str) + f;
                        AbstractOcTree* a_tree = AbstractOcTree::read(treefile);
                        if (a_tree)
                        {
                            OcTree* tree = dynamic_cast<OcTree*>(a_tree);
                            return *tree;
                        }
                        else
                        {
                            OcTree tree (0.5);
                            return tree;
                        }
                    }
                }
            }
        }
        // If no tree was returned
        ROS_ERROR("octomap_utils::get_tree_from_filename : did not find a .bt or .ot file in directory %s", str.c_str());
        OcTree tree (0.5);
        return tree;
    }
    // Otherwise just read the string as an octree file name
    else
    {
        ROS_WARN("octomap_utils::get_tree_from_filename : input %s is a file", str.c_str());
        if (strcmp(ext.c_str(),"bt") == 0)
        {
            OcTree tree (str);
            return tree;
        }
        else if (strcmp(ext.c_str(),"ot") == 0)
        {
            AbstractOcTree* a_tree = AbstractOcTree::read(str);
            if (a_tree)
            {
                OcTree* tree = dynamic_cast<OcTree*>(a_tree);
                return *tree;
            }
            else
            {
                OcTree tree (0.5);
                return tree;
            }
        }
    }
}

string append_backslash_to_path(const string &str)
{
    string res = str;
    // If the string is empty
    if (str.size() == 0)
    {
        res = "/";
        return res;
    }
    // If the last character is already a back slash then ignore
    char last_char = str[str.length()-1];
    if (last_char != '/')
        res = str + "/";
    return res;
}
