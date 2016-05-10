#include "squirrel_active_exploration/active_exploration_utils.h"

using namespace std;
using namespace pcl;
using namespace octomap;
using namespace squirrel_object_perception_msgs;

namespace active_exploration_utils
{
    /* === FUNCTIONS CALLED BY ACTIVE EXPLORATION CLASS === */

    bool segment(const PointCloud<PointT> &cloud, ros::ServiceClient &seg_client, vector<vector<int> > &segments)
    {
        ROS_INFO("active_exploration_utils::segment : starting");
        // If the cloud has point
        if (cloud.size() > 0)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(cloud, msg);
            // Call the segmentation service
            Segment seg_srv;
            seg_srv.request.cloud = msg;
            if (!seg_client.call(seg_srv))
            {
                ROS_ERROR("active_exploration_utils::segment : could not call the segmentation service");
                return false;
            }
            segments.clear();
            segments.resize(seg_srv.response.clusters_indices.size());
            for (size_t i = 0; i < seg_srv.response.clusters_indices.size(); ++i)
                segments[i] = seg_srv.response.clusters_indices[i].data;
            // If successful segmentation
            if (segments.size() == 0)
            {
                ROS_ERROR("active_exploration_utils::segment : cloud has zero segments");
                return false;
            }
        }
        // Otherwise cannot segment an empty point cloud
        else
        {
            ROS_ERROR("active_exploration_utils::segment : cloud is empty");
            return false;
        }

        ROS_INFO("active_exploration_utils::segment : successfully segmented point cloud into %lu segments", segments.size());
        ROS_INFO("active_exploration_utils::segment : finished");
        return true;
    }

    bool segment(const PointCloud<PointT> &cloud, const vector<vector<int> > &segment_indices, vector<vector<int> > &segments)
    {
        ROS_INFO("active_exploration_utils::segment : starting");
        // If the cloud has point
        int cloud_size = cloud.size();
        if (cloud_size > 0)
        {
            segments.clear();
            for (vector<vector<int> >::const_iterator it = segment_indices.begin(); it != segment_indices.end(); ++it)
            {
                std_msgs::Int32MultiArray seg;
                seg.data = *it;
                // Verify that each index is valid in the point cloud
                bool valid = true;
                for (vector<int>::size_type i = 0; i < seg.data.size(); ++i)
                {
                    if (seg.data[i] > cloud_size)
                    {
                        ROS_WARN("active_exploration_utils::segment : segment %lu specifies point %u which is larger than the point cloud %u",
                                 i, seg.data[i], cloud_size);
                        valid = false;
                        break;
                    }
                }
                if (valid)
                    segments.push_back(*it);
            }
            // If successful segmentation
            if (segments.size() == 0)
            {
                ROS_ERROR("active_exploration_utils::segment : cloud has zero segments");
                return false;
            }
        }
        // Otherwise cannot segment an empty point cloud
        else
        {
            ROS_ERROR("active_exploration_utils::segment : cloud is empty");
            return false;
        }

        ROS_INFO("active_exploration_utils::segment : successfully segmented point cloud into %lu segments", segments.size());
        ROS_INFO("active_exploration_utils::segment : finished");
        return true;
    }

    bool filter_segments(const OcTree &tree, const PointCloud<PointT> &transformed_cloud, const Eigen::Vector4f &position,
                         const double &max_object_distance, const double &min_object_height, const double &max_object_height,
                         const double &min_object_length, const double &max_object_length, vector<vector<int> > &segments,
                         vector<int> &ground_indices, vector<OcTreeKey> &ground_keys)
    {
        // Check that a point cloud exists
        if (transformed_cloud.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::filter_segments : input cloud is empty");
            return false;
        }
        // Check that segments exist
        if (segments.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::filter_segments : _segments is empty");
            return false;
        }
        // Get the planes
        vector<int> plane_indices;
        if (!find_planes(transformed_cloud, plane_indices))
        {
            ROS_ERROR("active_exploration_utils::filter_segments : could not find planes");
            return false;
        }
        if (plane_indices.size() == 0)
            ROS_WARN("active_exploration_utils::filter_segments : no planes were detected");
        else
            ROS_INFO("active_exploration_utils::filter_segments : found %lu planes", plane_indices.size());

        // Keep track of the current ground
        ground_indices = plane_indices;
        for (vector<int>::size_type i = 0; i < ground_indices.size(); ++i)
        {
            // Get the key
            OcTreeKey k;
            point3d p (transformed_cloud.points[ground_indices[i]].x,
                       transformed_cloud.points[ground_indices[i]].y,
                       transformed_cloud.points[ground_indices[i]].z);
            if (tree.coordToKeyChecked(p,k))
            {
                OcTreeNode *node = tree.search(k);
                if (node)
                {
                    if (node->getOccupancy() > 0)
                        ground_keys.push_back(k);
                }
            }
        }
        sort(ground_keys.begin(), ground_keys.end(), compare_octree_key);
        ground_keys.erase(unique(ground_keys.begin(), ground_keys.end(), equal_octree_key), ground_keys.end());

        PointT grd_min_pt, grd_max_pt;
        getMinMax3D (transformed_cloud, grd_min_pt, grd_max_pt);
        // Verify each segment
        vector<int> plane_segs;
        vector<int> invalid_segs;
        for (vector<vector<int> >::size_type i = 0; i != segments.size(); ++i)
        {
            bool valid = true;
            PointCloud<PointT> temp;
            copyPointCloud(transformed_cloud, segments[i], temp);
            PointT min_pt, max_pt;
            getMinMax3D (temp, min_pt, max_pt);
            vector<int> v (segments[i].size() + plane_indices.size());
            vector<int>::iterator iter = set_intersection (segments[i].begin(), segments[i].end(),
                                                           plane_indices.begin(), plane_indices.end(), v.begin());
            v.resize(iter-v.begin());
            double percentage_overlap = (double)v.size() / (double)segments[i].size();
            // If more than 50% of the points overlap with a plane then this segment is a plane
            if (percentage_overlap >= 0.5)
            {
                //ROS_INFO("ActiveExploration::filter_segments : segment %lu has a %.2f overlap with a plane", i, percentage_overlap);
                valid = false;
                plane_segs.push_back(i);
    //            // Double check the z height of this segment
    //            if (_received_transform)
    //            {
    //                if (z_height < 0.25)
    //                {
    //                    ROS_INFO("ActiveExploration::filter_segments : rejecting segment %lu which has height is %.2f", i, z_height);
    //                    valid = false;
    //                    plane_segs.push_back(i);
    //                }
    //                else
    //                {
    //                    ROS_INFO("ActiveExploration::filter_segments : segment %lu has height %.2f, not small enough for ground", i, z_height);
    //                }
    //            }
    //            else
    //            {
    //                valid = false;
    //                plane_segs.push_back(i);
    //            }
            }
            // Reject the segment if it is far away
            if (valid)
            {
                Eigen::Vector4f centroid;
                compute3DCentroid (transformed_cloud, segments[i], centroid);
                float d = distance3D(position[0],centroid[0],position[1],centroid[1],position[2],centroid[2]);
                if (d > max_object_distance)
                {
                    ROS_INFO("active_exploration_utils::filter_segments : rejecting segment %lu which has distance is %.2f", i, d);
                    valid = false;
                    invalid_segs.push_back(i);
                }
            }
    //        // Reject objects that are on the floor (only want objects on table)
    //        if (valid && _received_transform)
    //        {
    //            if (_table_height_threshold > 0)  // negative implies don't use table height threshold
    //            {
    //                if (height_above_ground < _table_height_threshold) // should also check the minimum z point, but that may be too strong
    //                {
    //                    ROS_INFO("ActiveExploration::filter_segments : rejecting segment %lu which is above ground by %.2f",
    //                             i, height_above_ground);
    //                    invalid_segs.push_back(i);
    //                    valid = false;
    //                }
    //            }
    //        }
            // If this has not yet been labelled as ground, check that it is valid
            if (valid)
            {
                if (!is_valid_segment(temp, min_object_height, max_object_height, min_object_length, max_object_length, i))
                {
                    valid = false;
                    invalid_segs.push_back(i);
                }
            }
            // If the segment is still valid then add it to the segments list
            if (valid)
            {
                //ROS_INFO("ActiveExploration::filter_segments : segment %lu is valid", i);
            }
        }

    //    // Visualize the planes and invalid segments
    //    if (!visualize_background(plane_segs, invalid_segs, grd_min_pt.data[2]))
    //        ROS_ERROR("ActiveExploration::filter_segments : could not visualize the background");

        // Remove the segments
        invalid_segs.insert (invalid_segs.end(), plane_segs.begin(), plane_segs.end());
        sort(invalid_segs.begin(), invalid_segs.end());  // sort
        invalid_segs.erase(unique(invalid_segs.begin(), invalid_segs.end()), invalid_segs.end()); // remove duplicates
        vector<vector<int> > previous_segments = segments;
        segments.clear();
        for (size_t i = 0; i < previous_segments.size(); ++i)
        {
            if (find(invalid_segs.begin(),invalid_segs.end(),i) == invalid_segs.end())
                segments.push_back (previous_segments[i]);
        }

        return true;
    }

    bool is_valid_segment(const PointCloud<PointT> &cloud, const double &min_object_height, const double &max_object_height,
                          const double &min_object_length, const double &max_object_length, const int &ix)
    {
        PointT min_pt, max_pt;
        getMinMax3D (cloud, min_pt, max_pt);
        double x_length = fabs(max_pt.data[0] - min_pt.data[0]);
        double y_length = fabs(max_pt.data[1] - min_pt.data[1]);
        double z_height = fabs(max_pt.data[2] - min_pt.data[2]);
        // Reject objects thare are too tall, e.g. people, walls, or too small
        if (z_height < min_object_height || z_height > max_object_height)
        {
            ROS_INFO("active_exploration_utils::is_valid_segment : rejecting segment %u which has height %.2f", ix, z_height);
            return false;
        }
        // Reject objects that are too short or too long
        else if (x_length < min_object_length || x_length > max_object_length)
        {
            ROS_INFO("active_exploration_utils::is_valid_segment : rejecting segment %u which has x length %.2f", ix, x_length);
            return false;
        }
        else if (y_length < min_object_length || y_length > max_object_length)
        {
            ROS_INFO("active_exploration_utils::is_valid_segment : rejecting segment %u which has y length %.2f", ix, y_length);
            return false;
        }

        // Return
        return true;
    }

    bool is_valid_segment(const PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const double &min_object_height,
                          const double &max_object_height, const double &min_object_length, const double &max_object_length, const int &ix)
    {
        // Transform the point cloud
        PointCloud<PointT> transformed_cloud;
        transformPointCloud(cloud, transformed_cloud, transform);
        return is_valid_segment(transformed_cloud, min_object_height, max_object_height, min_object_length, max_object_length, ix);
    }

    bool extract_segment_octree_keys(const OcTree &tree, const PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments,
                                     vector<vector<OcTreeKey> > &segment_octree_keys)
    {
        ROS_INFO("active_exploration_utils::extract_segment_octree_keys : starting");
        // If there are no segments
        if (segments.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::extract_segment_octree_keys : segments is empty");
            return false;
        }
        // Otherwise find the voxels
        segment_octree_keys.clear();
        segment_octree_keys.resize(segments.size());
        for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            vector<OcTreeKey> seg_keys;
            for (vector<int>::size_type j = 0; j < segments[i].size(); ++j)
            {
                // Get the key
                OcTreeKey k;
                point3d p (transformed_cloud.points[segments[i][j]].x,
                           transformed_cloud.points[segments[i][j]].y,
                           transformed_cloud.points[segments[i][j]].z);
                if (tree.coordToKeyChecked(p, k))
                    seg_keys.push_back(k);
                else
                    ROS_WARN("active_exploration_utils::extract_segment_octree_keys : could not find point (%.2f,%.2f,%.2f)", p.x(), p.y(), p.z());
            }
            sort(seg_keys.begin(), seg_keys.end(), compare_octree_key);
            seg_keys.erase(unique(seg_keys.begin(), seg_keys.end(), equal_octree_key), seg_keys.end());
            segment_octree_keys[i] = seg_keys;
        }
    //    // Visualize the segments
    //    octree_visualize_segments(_tree, 0.2, _segment_octree_keys);
        ROS_INFO("active_exploration_utils::extract_segment_octree_keys : finished");
        return true;
    }

    /* === POSE ESTIMATION / OBJECT TRACKING === */

    bool estimate_pose(const PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments, vector<Pose> &poses)
    {
        ROS_INFO("active_exploration_utils::estimate_pose : starting");
        // If have not segmented the point cloud
        if (segments.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::estimate_pose : have not segmented the point cloud");
            return false;
        }
        // Clear poses
        poses.clear();
        // Convert each segment to a point cloud
        for (vector<vector<int> >::const_iterator it = segments.begin(); it != segments.end(); ++it)
        {
            // Extract the point cloud from the indices
            PointCloud<PointT> seg;
            copyPointCloud(transformed_cloud, *it, seg);
            // Compute the centroid
            Eigen::Vector4f centroid;
            compute3DCentroid (seg, centroid);
            // Compute the bounding box limits
            Eigen::Vector4f bb_min, bb_max;
            getMinMax3D (transformed_cloud, *it, bb_min, bb_max);
            // Add to _poses vector
            poses.push_back (Pose (centroid, bb_min, bb_max));
        }

        ROS_INFO("active_exploration_utils::estimate_pose : finished");
        return true;
    }

    /* === CLASSIFICATION === */

    bool classify(const PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments, ros::ServiceClient &class_client,
                  vector<Classification> &class_estimates, vector<vector<InstLookUp> > &instance_directories,
                  vector<vector<InstToMapTF> > &instances_to_map_tfs)
    {
        ROS_INFO("active_exploration_utils::classify : starting");
        // If have not segmented the point cloud
        if (segments.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::classify : have not segmented the point cloud");
            return false;
        }
        // Clear the class estimates
        class_estimates.clear();
        // Pass the data to the message
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(transformed_cloud, cloud_msg);

        vector<std_msgs::Int32MultiArray> seg_msg;
        seg_msg.resize(segments.size());
        for(vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            std_msgs::Int32MultiArray seg;
            seg.data = segments[i];
            seg_msg[i] = seg;
        }
        Classify class_srv;
        class_srv.request.cloud = cloud_msg;
        class_srv.request.clusters_indices = seg_msg;
        // Call the service
        if (!class_client.call(class_srv))
        {
            ROS_ERROR("active_exploration_utils::classify : could not call the classification service");
            return false;
        }
        ROS_INFO("active_exploration_utils::classify : successfully classified the segments");
        class_estimates = class_srv.response.class_results;
        // Replace the double back slashes in the file paths
        class_estimates = fix_path_names(class_estimates);
        // Read the best instance and extract the pose transform file for each of the object class result
        if (!extract_instance_directories(class_estimates, instance_directories))
        {
            ROS_ERROR("active_exploration_utils::classify : could not extract the instance directories");
            return false;
        }
        ROS_INFO("active_exploration_utils::classify : successfully extracted instance directories");
        // Get the transformations of the instances to the maps
        if (!transform_instances_to_map(transformed_cloud, segments, instance_directories, instances_to_map_tfs))
        {
            ROS_ERROR("active_exploration_utils::classify : could not compute the transforms to the map");
            return false;
        }

        ROS_INFO("active_exploration_utils::classify : class estimates");
        for (size_t i = 0; i < class_estimates.size(); ++i)
        {
            ROS_INFO("Segment %lu -", i);
            for (size_t j = 0; j < class_estimates[i].class_type.size(); ++j)
                ROS_INFO("  %-15s %.2f", class_estimates[i].class_type[j].data.c_str(), class_estimates[i].confidence[j]);
        }

        ROS_INFO("active_exploration_utils::classify : finished");
        return true;
    }

    vector<Classification> fix_path_names(vector<Classification> &class_estimates)
    {
        vector<Classification> result;
        string from = "//";
        string to = "/";
        for (size_t i = 0; i < class_estimates.size(); ++i)
        {
            result.push_back(class_estimates[i]);
            for (vector<std_msgs::String>::iterator it = result[i].pose.begin(); it != result[i].pose.end(); ++it)
            {
                // Replace the double backslash with a single backslash
                if (it->data.find("//"))
                {
                    string s = it->data;
                    replace_substr(s,from,to);
                    it->data = s;
                }
            }
        }
        return result;
    }

    bool extract_instance_directories(const vector<Classification> &class_estimates, vector<vector<InstLookUp> > &instance_directories)
    {
        instance_directories.clear();
        // If no elements in class estimates
        if (class_estimates.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::extract_instance_directories : could not extract instance directory because class_estmates is has 0 elements");
            return false;
        }
        // Resize the vector
        instance_directories.resize(class_estimates.size());
        for (size_t i = 0; i < class_estimates.size(); ++i)
        {
            for (vector<std_msgs::String>::const_iterator it = class_estimates[i].pose.begin(); it != class_estimates[i].pose.end(); ++it)
            {
                string inst_dir = _NULL_FILE;
                string class_type = rem_backslash(class_estimates[i].class_type[distance<vector<std_msgs::String>::const_iterator>
                                    (class_estimates[i].pose.begin(),it)].data);
                string inst_name = _NULL_FILE;
                string ix = _NULL_FILE;
                // Extract the pose filename
                string s = it->data;
                // Split the filename and read off the directory and the integer of the pose
                size_t found = s.find_last_of("/\\");
                if (found != string::npos)
                {
                    string folder = s.substr(0,found); // path to instance
                    string pos = s.substr(found+1);  // name of instance in the directory
                    // The instance folder is one up from "folder" (the descriptors type)
                    found = folder.find_last_of("/\\");
                    if (found != string::npos)
                    {
                        inst_dir = folder.substr(0,found);
                        // The instance name the last directory in the path
                        size_t found2 = inst_dir.find_last_of("/\\");
                        if (found2 != string::npos)
                            inst_name = inst_dir.substr(found2+1);
                        else
                            ROS_WARN("active_exploration_utils::extract_instance_directories : could not get instance name from path %s",
                                     inst_dir.c_str());
                    }
                    else
                    {
                        ROS_WARN("active_exploration_utils::extract_instance_directories : could not get instance path in directory %s",
                                 folder.c_str());
                    }
                    // The index is between the underscore and dot in pos
                    size_t underscore = pos.find_last_of("_");
                    size_t dot = pos.find_last_of(".");
                    if (underscore != string::npos && dot != string::npos && (dot - found) > 0)
                    {
                        int str_len = dot - underscore;
                        ix = pos.substr(underscore+1,str_len-1);
                        //ix = atoi(str_ix.c_str());
                    }
                    else
                    {
                        ROS_WARN("active_exploration_utils::extract_instance_directories : could not get the pose number in file %s",
                                 pos.c_str());
                    }
                }
                else
                {
                    ROS_WARN("active_exploration_utils::extract_instance_directories : could not split filename %s",
                             s.c_str());
                }
                // Add to the list
                InstLookUp lu;
                lu._dir = inst_dir;
                lu._class_type = class_type;
                lu._instance_name = inst_name;
                lu._ix = ix;
                instance_directories[i].push_back(lu);
                //cout << "pose file " << s << endl;
                //cout << "instance directory " << inst_dir << endl;
                //cout << "class " << class_type << endl;
                //cout << "instance name " << inst_name << endl;
                //cout << "index " << ix << endl;
            }
        }
        return true;
    }

    bool transform_instances_to_map(const pcl::PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments,
                                    const vector<vector<InstLookUp> > &instance_directories, vector<vector<InstToMapTF> > &transforms)
    {
        transforms.clear();
        if (segments.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::transform_instances_to_map : segments has zero elements");
            return false;
        }
        // Resize the transforms vector
        transforms.resize(segments.size());
        // For each segment
        for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            if (segments[i].size() == 0)
            {
                ROS_WARN("active_exploration_utils::transform_instances_to_map : segment %lu has no points", i);
                InstToMapTF itf;
                itf._transform = Eigen::Matrix4f::Identity();
                itf._score = numeric_limits<double>::infinity();
                transforms[i].push_back (itf);
            }
            else if (instance_directories[i].size() == 0)
            {
                ROS_WARN("active_exploration_utils::transform_instances_to_map : segment %lu has no recognised instances", i);
                InstToMapTF itf;
                itf._transform = Eigen::Matrix4f::Identity();
                itf._score = numeric_limits<double>::infinity();
                transforms[i].push_back (itf);
            }
            else
            {
                // Get the subcloud
                PointCloud<PointT> seg;
                copyPointCloud(transformed_cloud, segments[i], seg);

                cout << "transform_instances_to_map : segment " << i << " has " << seg.size() << " points" << endl;
                int nan_count = 0, valid_count = 0;
                for (size_t j = 0; j < seg.size(); ++j)
                {
                    //cout << seg.points[j].x << " " << seg.points[j].y << " " << seg.points[j].z << endl;
                    if (isnan(seg.points[j].x))
                        nan_count++;
                    else
                        valid_count++;
                }
                cout << "nan = " << nan_count << ", valid = " << valid_count << endl;

                // Read the clouds
                for (vector<InstLookUp>::size_type j = 0; j < instance_directories[i].size(); ++j)
                {
                    string view_file = add_backslash(instance_directories[i][j]._dir) + _INSTANCE_VIEW_FILENAME +
                                                     instance_directories[i][j]._ix + ".pcd";
                    // If the view file is invalid
                    if (strcmp(instance_directories[i][j]._class_type.c_str(),_NULL_FILE) == 0 ||
                        strcmp(instance_directories[i][j]._instance_name.c_str(),_NULL_FILE) == 0 ||
                        strcmp(instance_directories[i][j]._ix.c_str(),_NULL_FILE) == 0)
                    {
                        ROS_WARN("active_exploration_utils::transform_instances_to_map : invalid instance directory %s/%s/%s",
                                 instance_directories[i][j]._class_type.c_str(), instance_directories[i][j]._instance_name.c_str(),
                                 instance_directories[i][j]._ix.c_str());
                        InstToMapTF itf;
                        itf._transform = Eigen::Matrix4f::Identity();
                        itf._score = numeric_limits<double>::infinity();
                        transforms[i].push_back (itf);
                    }
                    // Otherwise
                    else
                    {
                        PointCloud<PointT> view;
                        if (io::loadPCDFile(view_file.c_str(), view) == -1)
                            ROS_WARN("active_exploration_utils::transform_instances_to_map : could not load point cloud %s", view_file.c_str());
                        Eigen::Matrix4f tr;
                        double score;
                        //transform_cloud_to_cloud (view, seg, _poses[i].get_centroid(), tr, score);
                        transform_cloud_to_cloud (view, seg, tr, score);
                        // TODO check that this is correct
                        // tr brings it to the map frame (i.e. segment * _transform)
                        // _transform.inverse() * tr brings the point cloud back to the raw frame
                        InstToMapTF itf;
                        itf._transform = tr;
                        itf._score = score;
                        transforms[i].push_back (itf);
                        //cout << "transform segment " << i << " instance " << j << endl;
                        //cout << view_file << endl;
                        //cout << tr << endl;


    //                    // Check what the clouds look like
    //                    cout << "icp transform" << endl;
    //                    cout << tr << endl;
    //                    Eigen::Matrix4f tri = _transform.inverse();
    //                    io::savePCDFileASCII("observation.pcd", *cloud);
    //                    PointCloud<PointT>::Ptr cloud_transformed (new PointCloud<PointT>());
    //                    transformPointCloud (*cloud, *cloud_transformed, _transform);
    //                    io::savePCDFileASCII("transformed_observation.pcd", *cloud_transformed);
    //                    PointCloud<PointT>::Ptr original (new PointCloud<PointT>());
    //                    copyPointCloud(*cloud, _segments[i].data, *original);
    //                    io::savePCDFileASCII("original_segment.pcd", *original);
    //                    PointCloud<PointT>::Ptr seg_transformed (new PointCloud<PointT>());
    //                    transformPointCloud (*original, *seg_transformed, _transform);
    //                    io::savePCDFileASCII("transformed_segment.pcd", *seg_transformed);
    //                    io::savePCDFileASCII("original_view.pcd", *view);
    //                    PointCloud<PointT>::Ptr view_transform (new PointCloud<PointT>());
    //                    transformPointCloud (*view, *view_transform, tr);
    //                    io::savePCDFileASCII("transformed_view.pcd", *view_transform);
    //                    PointCloud<PointT>::Ptr view_transformed_to_raw (new PointCloud<PointT>());
    //                    Eigen::Matrix4f tr_raw = _transform.inverse() * tr;
    //                    transformPointCloud (*view, *view_transformed_to_raw, tr_raw);
    //                    io::savePCDFileASCII("transformed_to_raw_view.pcd", *view_transformed_to_raw);
    //                    return false;


                    }
                }
            }
        }
        return true;
    }

    bool transform_instances_to_map(const PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments,
                                    const vector<vector<InstLookUp> > &instance_directories, vector<vector<Eigen::Matrix4f> > &transforms)
    {
        vector<vector<InstToMapTF> > itfs;
        if (!transform_instances_to_map(transformed_cloud, segments, instance_directories, itfs))
        {
            ROS_ERROR("active_exploration_utils::transform_instances_to_map : could not get transforms");
            return false;
        }
        // Otherwise unwrap into just the transforms
        transforms.clear();
        transforms.resize(itfs.size());
        for (vector<vector<InstToMapTF> >::size_type i = 0; i < itfs.size(); ++i)
        {
            transforms[i].resize(itfs[i].size());
            for (vector<InstToMapTF>::size_type j = 0; j < itfs[i].size(); ++j)
                transforms[i][j] = itfs[i][j]._transform;
        }
        return true;
    }

    /* === GET ENTROPY MAPS === */

    bool retrieve_entropy_maps(const vector<vector<int> > &segments, const vector<vector<InstLookUp> > &instance_directories,
                               ros::ServiceClient &em_client, vector<vector<EntMap> > &entropy_maps)
    {
        ROS_INFO("active_exploration_utils::retrieve_entropy_maps : starting");
        entropy_maps.clear();
        // Check if there are available segments and class estimates
        if (segments.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::retrieve_entropy_maps : no segments available");
            return false;
        }
        if (instance_directories.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::retrieve_entropy_maps : no instance directories available");
            return false;
        }
        if (segments.size() != instance_directories.size())
        {
            ROS_ERROR("active_exploration_utils::retrieve_entropy_maps : segments size %lu does not match instance directories size %lu",
                      segments.size(), instance_directories.size());
            return false;
        }
        // For each segment call the entropy map service
        entropy_maps.resize(instance_directories.size());
        squirrel_object_perception_msgs::EntropyMap em_srv;
        for (size_t i = 0; i < instance_directories.size(); ++i)
        {
            // For each class estimate
            for (size_t j = 0; j < instance_directories[i].size(); ++j)
            {
                // Get the instance entropy map
                // Call the service to get the instance entropy map
                em_srv.request.class_type.data = instance_directories[i][j]._class_type;
                em_srv.request.instance_name.data = instance_directories[i][j]._instance_name;
                if (!em_client.call(em_srv))
                {
                    ROS_ERROR("active_exploration_utils::retrieve_entropy_maps : could not call the entropy map service");
                    return false;
                }
    //            ROS_INFO("ActiveExploration::retrieve_entropy_maps : extracted entropy map with %lu elements and instance cloud size %lu",
    //                      _em_srv.response.entropy_map.size(), _em_srv.response.cloud.data.size());
                // Convert to local structure
                EntMap emap;
                if (active_exploration_utils::fromROSMsg(em_srv, emap))
                {
                    emap._valid = true;
                    entropy_maps[i].push_back (emap);
                }
                else
                {
                    ROS_ERROR("active_exploration_utils::retrieve_entropy_maps : could not convert the entropy map for %s %s",
                              em_srv.request.class_type.data.c_str(), em_srv.request.instance_name.data.c_str());
                    emap._valid = false;
                    entropy_maps[i].push_back (emap);
                }
            }
        }
        ROS_INFO("active_exploration_utils::retrieve_entropy_maps : retrieved %lu entropy maps", entropy_maps.size());
        if (entropy_maps.size() != segments.size())
            ROS_WARN("active_exploration_utils::retrieve_entropy_maps : retrieved %lu entropy maps, should be %lu",
                     entropy_maps.size(), segments.size());
        ROS_INFO("active_exploration_utils::retrieve_entropy_maps : finished");
        return true;
    }

    /* === COMPUTE ENTROPY === */

    bool compute_entropy(const vector<Classification> &class_estimates, vector<double> &entropies)
    {
        ROS_INFO("active_exploration_utils::compute_entropy : starting");
        // Check that classification results exist
        if (class_estimates.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::compute_entropy : have not classified the point cloud");
            return false;
        }

        // Compute the entropies
        entropies.clear();
        // Compute the entropies
        for (vector<Classification>::const_iterator it = class_estimates.begin(); it != class_estimates.end(); ++it)
            entropies.push_back ((double)entropy(it->confidence));

        ROS_INFO("active_exploration_utils::compute_entropy : finished");
        return true;
    }

    bool rank_entropy(const vector<double> &entropies, vector<int> &entropy_ranking)
    {
        ROS_INFO("active_exploration_utils::rank_entropy : starting");
        // Check that entropies have been calculated
        if (entropies.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::rank_entropy : have not computed the entropies of the segments");
            return false;
        }

        // Ranke the entropies
        entropy_ranking.clear();
        // Compute the ranking
        entropy_ranking = rank(entropies);
        reverse (entropy_ranking.begin(), entropy_ranking.end());

        ROS_INFO("active_exploration_utils::rank_entropy : finished");
        return true;
    }

    /* === UPDATE HYPOTHESES === */

    bool update_hypotheses(const vector<vector<int> > &overlaps, const OcTree &tree, const Hypothesis &previous_hyp, const Hypothesis &current_hyp,
                           vector<vector<int> > &updated_associations, Hypothesis &result_hyp)
    {
        ROS_INFO("active_exploration_utils::update_hypotheses : starting");
        //                   class , p file , conf, inst dir  , transform
        typedef boost::tuple<string, string, float, InstLookUp, InstToMapTF> class_info;

        // Extract the information from the hypothesis structures
        vector<vector<int> > _segments = current_hyp._segments;
        vector<vector<OcTreeKey> > _octree_keys = current_hyp._octree_keys;
        vector<Classification> _class_estimates = current_hyp._class_estimates;
        vector<Pose> _poses = current_hyp._poses;
        vector<vector<InstLookUp> > _instance_directories = current_hyp._instance_directories;
        vector<vector<InstToMapTF> > _transforms = current_hyp._transforms;

        vector<vector<OcTreeKey> > octree_keys = previous_hyp._octree_keys;
        vector<Classification> class_estimates = previous_hyp._class_estimates;
        vector<Pose> poses = previous_hyp._poses;
        vector<vector<InstLookUp> > instance_directories = previous_hyp._instance_directories;
        vector<vector<InstToMapTF> > transforms = previous_hyp._transforms;

        // Add a small epsilon to each entropy
        for (size_t i = 0; i < _class_estimates.size(); ++i)
        {
            for (size_t j = 0; j < _class_estimates[i].confidence.size(); ++j)
                _class_estimates[i].confidence[j] += _EPS;
        }
        vector<Classification> class_estimates_cp = class_estimates;
        for (size_t i = 0; i < class_estimates_cp.size(); ++i)
        {
            for (size_t j = 0; j < class_estimates_cp[i].confidence.size(); ++j)
                class_estimates_cp[i].confidence[j] += _EPS;
        }

    //    // Print out the information
    //    cout << "** BEFORE UPDATING ** PREVIOUS CLASS RESULTS **" << endl;
    //    cout << "Size octree keys " << octree_keys.size() << endl;
    //    cout << "Size poses " << poses.size() << endl;
    //    cout << "Size class estimates " << class_estimates.size() << endl;
    //    cout << "Size instance directories " << instance_directories.size() << endl;
    //    cout << "Size instance tfs " << transforms.size() << endl;
    //    for (size_t i = 0; i < class_estimates.size(); ++i)
    //    {
    //        cout << "Segment " << i << endl;
    //        cout << "num ests = " << class_estimates[i].class_type.size()
    //             << " num dirs = " << instance_directories[i].size()
    //             << " num tfs = " << transforms[i].size() << endl;
    //        for (size_t j = 0; j < class_estimates[i].class_type.size(); ++j)
    //        {
    //            cout << "class " << j << endl;
    //            cout << "   est " << class_estimates[i].class_type[j].data << endl;
    //            cout << "   conf " << class_estimates[i].confidence[j] << endl;
    //            cout << "   inst " << instance_directories[i][j]._class_type << "/"
    //                                << instance_directories[i][j]._instance_name << "/"
    //                                << instance_directories[i][j]._ix << endl;
    //            cout << "   tf " << transforms[i][j]._transform << endl;
    //            cout << "   score " << transforms[i][j]._score << endl;
    //        }
    //    }

    //    cout << "** BEFORE UPDATING ** LATEST CLASS RESULTS **" << endl;
    //    cout << "Size segments " << _segments.size() << endl;
    //    cout << "Size octree keys " << _segment_octree_keys.size() << endl;
    //    cout << "Size poses " << _poses.size() << endl;
    //    cout << "Size class estimates " << _class_estimates.size() << endl;
    //    cout << "Size instance directories " << _instance_directories.size() << endl;
    //    cout << "Size instance tfs " << _transforms.size() << endl;
    //    for (size_t i = 0; i < _class_estimates.size(); ++i)
    //    {
    //        cout << "Segment " << i << endl;
    //        cout << "num ests = " << _class_estimates[i].class_type.size()
    //             << " num dirs = " << _instance_directories[i].size()
    //             << " num tfs = " << _transforms[i].size() << endl;
    //        for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
    //        {
    //            cout << "class " << j << endl;
    //            cout << "   est " << _class_estimates[i].class_type[j].data << endl;
    //            cout << "   conf " << _class_estimates[i].confidence[j] << endl;
    //            cout << "   inst " << _instance_directories[i][j]._class_type << "/"
    //                                << _instance_directories[i][j]._instance_name << "/"
    //                                << _instance_directories[i][j]._ix << endl;
    //            cout << "   tf " << _transforms[i][j]._transform << endl;
    //            cout << "   score " << _transforms[i][j]._score << endl;
    //        }
    //    }

        // Assumes that the current _class_estimates is resized
        if (_class_estimates.size() != overlaps.size())
        {
            ROS_ERROR("active_exploration_utils::update_hypotheses : incorrect class estimates size");
            ROS_ERROR("_class_estimates is %lu and overlaps is %lu", _class_estimates.size(), overlaps.size());
            return false;
        }
        // Go through the vector of overlaps and update the class estimates
        vector<int> merged_into_current;
        vector<vector<class_info> > train_instances;
        vector<int> used_ix;
        updated_associations.clear();
        updated_associations.resize(overlaps.size());
        for (vector<vector<int> >::size_type i = 0; i < overlaps.size(); ++i)
        {
            ROS_INFO("Segment %lu", i);
            updated_associations[i] = overlaps[i];
            // If this segment has not already been merged into a previous segment
            if (find(merged_into_current.begin(), merged_into_current.end(), i) == merged_into_current.end())
            {
                ROS_INFO(" - original class estimates -");
                for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                    ROS_INFO("  %-15s %.2f", _class_estimates[i].class_type[j].data.c_str(), _class_estimates[i].confidence[j]);
    //            ROS_INFO(" - original instance directories -");
    //            for (size_t j = 0; j < _instance_directories[i].size(); ++j)
    //                ROS_INFO("  %s/%s/%s", _instance_directories[i][j]._class_type.c_str(), _instance_directories[i][j]._instance_name.c_str(),
    //                                       _instance_directories[i][j]._ix.c_str());
                // Maintain a vector of the best scoring poses and transform files
                vector<class_info> best_train_instance;
                double min_eps = numeric_limits<double>::infinity();
                for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                {
                    best_train_instance.push_back(boost::make_tuple(_class_estimates[i].class_type[j].data,
                                                                    _class_estimates[i].pose[j].data,
                                                                    _class_estimates[i].confidence[j],
                                                                    _instance_directories[i][j],
                                                                    _transforms[i][j]));
                    if (_class_estimates[i].confidence[j] < min_eps)
                        min_eps = _class_estimates[i].confidence[j];
                }
                // Check the min eps value, if it is larger than _EPS then use _EPS
                if (min_eps > _EPS)
                    min_eps = _EPS;
                // If any other segment has the same overlaps then must also include this in the new segment and merge them together
                vector<int> merges;
                vector<int> curr_overlaps = overlaps[i];
                for (vector<int>::size_type j = i+1; j < overlaps.size(); ++j)
                {
                    bool do_merge = false;
                    for (vector<int>::size_type k = 0; k < overlaps[i].size(); ++k)
                    {
                        // If overlaps[j] contains any element in overlaps[i]
                        // then merge overlaps[i] and overlaps[j]
                        if (find(overlaps[j].begin(), overlaps[j].end(),overlaps[i][k]) != overlaps[j].end())
                        {
                            merges.push_back(j);
                            do_merge = true;
                        }
                    }
                    // Add the merged object's overlaps to the current list of overlaps if they don't already exist
                    if (do_merge)
                    {
                        // Include overlaps[j] into the list of overlaps[i]
                        curr_overlaps.insert(curr_overlaps.end(), overlaps[j].begin(), overlaps[j].end());
                        sort(curr_overlaps.begin(), curr_overlaps.end());
                        curr_overlaps.erase(unique(curr_overlaps.begin(), curr_overlaps.end()), curr_overlaps.end());
                        merged_into_current.push_back(j);
                        // Add to the new associations
                        updated_associations[i].insert(updated_associations[i].end(), overlaps[j].begin(), overlaps[j].end());
                        sort(updated_associations[i].begin(), updated_associations[i].end());
                        updated_associations[i].erase(unique(updated_associations[i].begin(), updated_associations[i].end()),
                                                      updated_associations[i].end());
                    }
                }
                // If found merges
                if (merges.size() > 0)
                {
                    // Remove duplicates in merges
                    sort(merges.begin(), merges.end());
                    merges.erase(unique(merges.begin(), merges.end()), merges.end());
                    // Merge
                    for (vector<int>::size_type j = 0; j < merges.size(); ++j)
                    {
                        // Merge segmentation
                        if (!merge_segments(_segments[i], _segments[merges[j]], _segments[i]))
                        {
                            ROS_ERROR("active_exploration_utils::update_hypotheses : could not update segmentation %lu with current segment %u",
                                      i, merges[j]);
                            return false;
                        }
                        // Merge the class estimates
                        if (!merge_estimates(_class_estimates[i], _class_estimates[merges[j]], min_eps, _class_estimates[i]))
                        {
                            ROS_ERROR("active_exploration_utils::update_hypotheses : could not update class estimate %lu with current segment %u",
                                      i, merges[j]);
                            return false;
                        }
                        // Merge the segment octree keys
                        if (!merge_segment_octree_keys(tree, _octree_keys[i], _octree_keys[merges[j]], _octree_keys[i]))
                        {
                            ROS_ERROR("active_exploration_utils::update_hypotheses : could not update segment tree keys %lu with current segment %u",
                                      i, merges[j]);
                            return false;
                        }
                        // Merge the poses
                        if (!merge_poses(_poses[i], _poses[merges[j]], _poses[i]))
                        {
                            ROS_ERROR("active_exploration_utils::update_hypotheses : could not update pose %lu with current segment %u",
                                      i, merges[j]);
                            return false;
                        }
                        vector<int> not_matched;
                        for (size_t k = 0; k < best_train_instance.size(); ++k)
                            not_matched.push_back(k);
                        // Update the best files and transforms vector
                        for (size_t k = 0; k < _class_estimates[merges[j]].class_type.size(); ++k)
                        {
                            bool found_match = false;
                            for (size_t kk = 0; kk < best_train_instance.size(); ++kk)
                            {
                                // If the same class
                                if (strcmp(_class_estimates[merges[j]].class_type[k].data.c_str(), best_train_instance[kk].get<0>().c_str()) == 0)
                                {
                                    found_match = true;
    //                                // If the confidence is better then replace with the files
    //                                if (_class_estimates[merges[j]].confidence[k] > best_train_instance[kk].get<2>())
                                    // If the alignment score is better then replace with the files and transforms
                                    if (_transforms[merges[j]][k]._score < best_train_instance[kk].get<4>()._score)
                                    {
                                        // Replace
                                        best_train_instance[kk] = boost::make_tuple(_class_estimates[merges[j]].class_type[k].data,
                                                                                    _class_estimates[merges[j]].pose[k].data,
                                                                                    _class_estimates[merges[j]].confidence[k],
                                                                                    _instance_directories[merges[j]][k],
                                                                                    _transforms[merges[j]][k]);
                                    }
                                    vector<int>::iterator pos = find(not_matched.begin(), not_matched.end(), kk);
                                    if (pos != not_matched.end())
                                        not_matched.erase(pos);
                                }
                            }
                            // If not found then need to add this new class to the list,
                            // but the confidence must be multiplied by a small epsilon value
                            if (!found_match)
                            {
                                best_train_instance.push_back(boost::make_tuple(_class_estimates[merges[j]].class_type[k].data,
                                                                                _class_estimates[merges[j]].pose[k].data,
                                                                                _class_estimates[merges[j]].confidence[k]*min_eps,
                                                                                _instance_directories[merges[j]][k],
                                                                                _transforms[merges[j]][k]));
                            }
                        }
                        // Add the unmatched elements from the merge index
                        for (vector<int>::size_type k = 0; k < not_matched.size(); ++k)
                        {
                            // The confidence is multiplied by min_eps because it has 0 confidence since it was not found
                            // in the first classification
                            best_train_instance[k] = boost::make_tuple(best_train_instance[k].get<0>(),
                                                                       best_train_instance[k].get<1>(),
                                                                       best_train_instance[k].get<2>()*min_eps,
                                                                       best_train_instance[k].get<3>(),
                                                                       best_train_instance[k].get<4>());
                        }
                    }
                }
                // Update with all of the previous segments by searching through the overlaps
                for (vector<int>::size_type j = 0; j < curr_overlaps.size(); ++j)
                {
                    // Merge the class estimates
                    if (!merge_estimates(_class_estimates[i], class_estimates_cp[curr_overlaps[j]], min_eps, _class_estimates[i]))
                    {
                        ROS_ERROR("active_exploration_utils::update_hypotheses : could not update class estimate %lu", i);
                        return false;
                    }
                    // Merge the segment tree keys
                    if (!merge_segment_octree_keys(tree, _octree_keys[i], octree_keys[curr_overlaps[j]], _octree_keys[i]))
                    {
                        ROS_ERROR("active_exploration_utils::update_hypotheses : could not update segment octree keys %lu", i);
                        return false;
                    }
                    // Merge the poses
                    if (!merge_poses(_poses[i], poses[curr_overlaps[j]], _poses[i]))
                    {
                        ROS_ERROR("active_exploration_utils::update_hypotheses : could not update pose %lu", i);
                        return false;
                    }
                    vector<int> not_matched;
                    for (size_t k = 0; k < best_train_instance.size(); ++k)
                        not_matched.push_back(k);
                    // Update the best files vector
                    for (size_t k = 0; k < class_estimates_cp[curr_overlaps[j]].class_type.size(); ++k)
                    {
                        bool found_match = false;
                        for (size_t kk = 0; kk < best_train_instance.size(); ++kk)
                        {
                            // If the same class
                            if (strcmp(class_estimates_cp[curr_overlaps[j]].class_type[k].data.c_str(), best_train_instance[kk].get<0>().c_str()) == 0)
                            {
                                found_match = true;
    //                            // If the confidence is better then replace with the files belonging to this object
    //                            if (class_estimates[curr_overlaps[j]].confidence[k] > best_train_instance[kk].get<2>())
                                // If the alignment score is better then replace with the files and transforms
                                if (transforms[curr_overlaps[j]][k]._score < best_train_instance[kk].get<4>()._score)
                                {
                                    // Replace
                                    best_train_instance[kk] = boost::make_tuple(class_estimates_cp[curr_overlaps[j]].class_type[k].data,
                                                                                class_estimates_cp[curr_overlaps[j]].pose[k].data,
                                                                                class_estimates_cp[curr_overlaps[j]].confidence[k],
                                                                                instance_directories[curr_overlaps[j]][k],
                                                                                transforms[curr_overlaps[j]][k]);
                                }
                                vector<int>::iterator pos = find(not_matched.begin(), not_matched.end(), kk);
                                if (pos != not_matched.end())
                                    not_matched.erase(pos);
                            }
                        }
                        // If not found then need to add this new class to the list
                        // but the confidence must be multiplied by a small epsilon value
                        if (!found_match)
                        {
                            best_train_instance.push_back(boost::make_tuple(class_estimates_cp[curr_overlaps[j]].class_type[k].data,
                                                                            class_estimates_cp[curr_overlaps[j]].pose[k].data,
                                                                            class_estimates_cp[curr_overlaps[j]].confidence[k]*min_eps,
                                                                            instance_directories[curr_overlaps[j]][k],
                                                                            transforms[curr_overlaps[j]][k]));
                        }
                    }
                    // Add the unmatched elements from the merge index
                    for (vector<int>::size_type k = 0; k < not_matched.size(); ++k)
                    {
                        // The confidence is multiplied by min_eps because it has 0 confidence since it was not found
                        // in the first classification
                        best_train_instance[k] = boost::make_tuple(best_train_instance[k].get<0>(),
                                                                   best_train_instance[k].get<1>(),
                                                                   best_train_instance[k].get<2>()*min_eps,
                                                                   best_train_instance[k].get<3>(),
                                                                   best_train_instance[k].get<4>());
                    }
                    // Add the index to the list of used indices
                    used_ix.push_back(curr_overlaps[j]);
                }
                train_instances.push_back (best_train_instance);
                ROS_INFO(" - combined class estimates -");
                for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
                    ROS_INFO("  %-15s %.2f", _class_estimates[i].class_type[j].data.c_str(), _class_estimates[i].confidence[j]);
            }
        }

        // Erase the segment indices, class estimates and poses
        // that were merged into other segments
        sort(merged_into_current.begin(), merged_into_current.end());  // sort
        merged_into_current.erase(unique(merged_into_current.begin(), merged_into_current.end()), merged_into_current.end());  // unique
        reverse(merged_into_current.begin(), merged_into_current.end());  // reverse
        ROS_INFO("Removing %lu segments that were merged", merged_into_current.size());
        // Iterate from highest index to lowest
        for (vector<int>::size_type i = 0; i < merged_into_current.size(); ++i)
        {
            _segments.erase(_segments.begin() + merged_into_current[i]);
            _class_estimates.erase(_class_estimates.begin() + merged_into_current[i]);
            _octree_keys.erase(_octree_keys.begin() + merged_into_current[i]);
            _poses.erase(_poses.begin() + merged_into_current[i]);
            updated_associations.erase(updated_associations.begin() + merged_into_current[i]);
        }
        // Update the pose files, instance training directories and transforms
        _instance_directories.clear();
        _instance_directories.resize(_class_estimates.size());
        _transforms.clear();
        _transforms.resize(_class_estimates.size());
        //ROS_INFO("Updating pose estimates to highest scoring classification");
        for (size_t i = 0; i < _class_estimates.size(); ++i)
        {
            _instance_directories[i].resize(_class_estimates[i].class_type.size());
            _transforms[i].resize(_class_estimates[i].class_type.size());
            // Add to the lists in order of the confidences
            for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
            {
                // Find the corresponding class type in the best_train_instance vector
                for (size_t k = 0; k < train_instances[i].size(); ++k)
                {
                    // If the same class type
                    if (strcmp(_class_estimates[i].class_type[j].data.c_str(),train_instances[i][k].get<0>().c_str()) == 0)
                    {
                        _class_estimates[i].pose[j].data = train_instances[i][k].get<1>();
                        _instance_directories[i][j] = train_instances[i][k].get<3>();
                        _transforms[i][j] = train_instances[i][k].get<4>();
                        break;
                    }
                }
            }
        }
        // Add any objects from the previous list that were not matched with any objects in the current list
        sort(used_ix.begin(), used_ix.end());
        used_ix.erase(unique(used_ix.begin(), used_ix.end()),used_ix.end());
        //ROS_INFO("Adding segments that were missed");
        for (size_t i = 0; i < class_estimates.size(); ++i)
        {
            // If this index is not in the used_ix list
            if (find(used_ix.begin(),used_ix.end(),i) == used_ix.end())
            {
                //ROS_INFO("   adding %lu",i);
                // Append this object to the list of hypotheses
                // Empty segment (because there is no reference to a point cloud)
                vector<int> seg;
                _segments.push_back(seg);
                // Segment octree keys
                _octree_keys.push_back (octree_keys[i]);
                // Pose
                _poses.push_back(poses[i]);
                // Class estimate
                _class_estimates.push_back(class_estimates[i]);
                // Instance directory
                _instance_directories.push_back(instance_directories[i]);
                // Transforms
                _transforms.push_back(transforms[i]);
                // New associations
                vector<int> temp;
                temp.push_back(i);
                updated_associations.push_back(temp);
            }
        }

    //    // Print out the merged information
    //    cout << "Size segments " << _segments.size() << endl;
    //    cout << "Size octree keys " << _octree_keys.size() << endl;
    //    cout << "Size poses " << _poses.size() << endl;
    //    cout << "Size class estimates " << _class_estimates.size() << endl;
    //    cout << "Size instance directories " << _instance_directories.size() << endl;
    //    cout << "Size instance tfs " << _transforms.size() << endl;
    //    for (size_t i = 0; i < _class_estimates.size(); ++i)
    //    {
    //        cout << "Segment " << i << endl;
    //        cout << "num ests = " << _class_estimates[i].class_type.size()
    //             << " num dirs = " << _instance_directories[i].size()
    //             << " num tfs = " << _transforms[i].size() << endl;
    //        for (size_t j = 0; j < _class_estimates[i].class_type.size(); ++j)
    //        {
    //            cout << "class " << j << endl;
    //            cout << "   est " << _class_estimates[i].class_type[j].data << endl;
    //            cout << "   conf " << _class_estimates[i].confidence[j] << endl;
    //            cout << "   inst " << _instance_directories[i][j]._class_type << "/"
    //                                << _instance_directories[i][j]._instance_name << "/"
    //                                << _instance_directories[i][j]._ix << endl;
    //            cout << "   tf " << _transforms[i][j]._transform << endl;
    //            cout << "   score " << _transforms[i][j]._score << endl;
    //        }
    //    }


    //    cin.ignore();

        // Put the information into the output
        result_hyp._segments = _segments;
        result_hyp._octree_keys = _octree_keys;
        result_hyp._class_estimates = _class_estimates;
        result_hyp._poses = _poses;
        result_hyp._instance_directories = _instance_directories;
        result_hyp._transforms = _transforms;

        ROS_INFO("active_exploration_utils::update_hypotheses : finished");
        return true;
    }

    bool merge_segments(const vector<int> &first, const vector<int> &second, vector<int> &output)
    {
        // Clear the output object
        vector<int> result;
        //output.clear();

        if (first.size() == 0)
        {
            ROS_WARN("active_exploration_utils::merge_segments : first segment has no elements");
            output = second;
            return true;
        }
        if (second.size() == 0)
        {
            ROS_WARN("active_exploration_utils::merge_segments : second segment has no elements");
            output = first;
            return true;
        }
        // Otherwise merge the segments
        result = first;
        for (size_t i = 0; i < second.size(); ++i)
            result.push_back(second[i]);
        // Sort
        sort(result.begin(), result.end());
        // Remove duplicates
        result.erase(unique(result.begin(), result.end()), result.end());
        output = result;
        return true;
    }

    bool merge_estimates(const Classification &first, const Classification &second, const double &min_eps, Classification &output)
    {
        typedef boost::tuple<string,string,float> class_info;

//        // Clear the output object
//        output.class_type.clear();
//        output.confidence.clear();
//        output.pose.clear();
        Classification result;

        if (first.class_type.size() == 0)
        {
            ROS_WARN("active_exploration_utils::merge_estimates : first class estimate has no elements");
            output = second;
            return true;
        }
        if (second.class_type.size() == 0)
        {
            ROS_WARN("active_exploration_utils::merge_estimates : second class estimate has no elements");
            output = first;
            return true;
        }
        // Otherwise merge the estimates
        vector<int> not_matched;
        for (size_t i = 0; i < second.class_type.size(); ++i)
            not_matched.push_back(i);
        // Go through each element and update the class
        vector<class_info> combined_res;
        combined_res.resize(first.class_type.size());
        for (size_t i = 0; i < first.class_type.size(); ++i)
        {
            combined_res[i] = boost::make_tuple(first.class_type[i].data, first.pose[i].data, first.confidence[i]);
            bool found_in_second = false;
            for (size_t j = 0; j < second.class_type.size(); ++j)
            {
                // If the same class
                if (strcmp(first.class_type[i].data.c_str(), second.class_type[j].data.c_str()) == 0)
                {
                    found_in_second = true;
                    combined_res[i] = boost::make_tuple(combined_res[i].get<0>(),
                                                        combined_res[i].get<1>(),
                                                        combined_res[i].get<2>()*second.confidence[j]);
                    vector<int>::iterator pos = find(not_matched.begin(), not_matched.end(), j);
                    if (pos != not_matched.end())
                        not_matched.erase(pos);
                }
            }
            // If this element was not found in the second then must multiply the confidence by
            // min_eps because it has essentially 0 confidence in the second classification
            if (!found_in_second)
                combined_res[i] = boost::make_tuple(first.class_type[i].data, first.pose[i].data, first.confidence[i]*min_eps);
        }
        // Add the unmatched elements from second
        for (vector<int>::size_type i = 0; i < not_matched.size(); ++i)
        {
            // The confidence is multiplied by min_eps because it has 0 confidence since it was not found
            // in the first classification
            combined_res.push_back(boost::make_tuple(second.class_type[not_matched[i]].data,
                                                     second.pose[not_matched[i]].data,
                                                     second.confidence[not_matched[i]]*min_eps));
        }
        // Normalise the confidences
        float normalization_const = 0;
        for (vector<class_info>::size_type i = 0; i < combined_res.size(); ++i)
            normalization_const += combined_res[i].get<2>();
        vector<pair<int,float> > sort_vec;
        for (vector<class_info>::size_type i = 0; i < combined_res.size(); ++i)
        {
            combined_res[i] = boost::make_tuple(combined_res[i].get<0>(),
                                                combined_res[i].get<1>(),
                                                combined_res[i].get<2>()/normalization_const);
            sort_vec.push_back(make_pair(i,combined_res[i].get<2>()));
        }
        // Sort the classes in order of highest confidence to lowest confidence
        sort(sort_vec.begin(), sort_vec.end(), compare<float>);
        reverse(sort_vec.begin(), sort_vec.end());
        // Assign the values to the output object
        result.class_type.resize(combined_res.size());
        result.confidence.resize(combined_res.size());
        result.pose.resize(combined_res.size());
        for (vector<class_info>::size_type i = 0; i < combined_res.size(); ++i)
        {
            std_msgs::String str_tmp;
            str_tmp.data = combined_res[sort_vec[i].first].get<0>();
            result.class_type[i] = str_tmp;
            str_tmp.data = combined_res[sort_vec[i].first].get<1>();
            result.pose[i] = str_tmp;
            result.confidence[i] = combined_res[sort_vec[i].first].get<2>();
    //        cout << "class " << combined_res[sort_vec[i].first].get<0>()
    //             << " conf = " << combined_res[sort_vec[i].first].get<2>() << endl;
        }

        output = result;

        return true;
    }

    bool merge_segment_octree_keys(const OcTree &tree, const vector<OcTreeKey> &first, const vector<OcTreeKey> &second, vector<OcTreeKey> &output)
    {
        //output.clear();
        vector<OcTreeKey> result;
        if (first.size() == 0)
        {
            ROS_WARN("active_exploration_utils::merge_segment_octree_keys : first vector is empty");
            for (size_t i = 0; i < second.size(); ++i)
            {
                OcTreeNode *node = tree.search(second[i]);
                if (node)
                {
                    if (node->getOccupancy() > 0.5)
                        result.push_back(second[i]);
                    else
                        cout << "Rejecting key" << endl;
                }
            }
            //output = second;
            output = result;
            return true;
        }
        if (second.size() == 0)
        {
            ROS_WARN("active_exploration_utils::merge_segment_octree_keys : second vector is empty");
            for (size_t i = 0; i < first.size(); ++i)
            {
                OcTreeNode *node = tree.search(first[i]);
                if (node)
                {
                    if (node->getOccupancy() > 0.5)
                        result.push_back(first[i]);
                    else
                        cout << "Rejecting key" << endl;
                }
            }
            //output = first;
            output = result;
            return true;
        }
        // Otherwise merge the octree key vectors
        //output.clear();
        // Insert the values from the first vector
        for (size_t i = 0; i < first.size(); ++i)
        {
            OcTreeNode *node = tree.search(first[i]);
            if (node)
            {
                if (node->getOccupancy() > 0.5)
                    result.push_back(first[i]);
            }
        }
        // Insert the values from the second vector
        for (size_t i = 0; i < second.size(); ++i)
        {
            OcTreeNode *node = tree.search(second[i]);
            if (node)
            {
                if (node->getOccupancy() > 0.5)
                    result.push_back(second[i]);
            }
        }
    //    // Insert the values from the first vector
    //    output.insert (output.end(), first.begin(), first.end());
    //    // Insert the values from the second vector
    //    output.insert (output.end(), second.begin(), second.end());
        // Sort
        sort(result.begin(), result.end(), compare_octree_key);
        // Remove duplicates
        result.erase(unique(result.begin(), result.end(), equal_octree_key), result.end());

        output = result;

        return true;
    }

    bool merge_poses(const Pose &first, const Pose &second, Pose &output)
    {
        Pose result;

        if (!first.is_valid())
        {
            ROS_WARN("active_exploration_utils::merge_poses : first pose is invalid");
            output = second;
            return true;
        }
        if (!second.is_valid())
        {
            ROS_WARN("active_exploration_utils::merge_poses : second pose is invalid");
            output = first;
            return true;
        }
        // Otherwise merge the poses
        // Compute the mean of the centroid
        Eigen::Vector4f first_centroid = first.get_centroid();
        Eigen::Vector4f second_centroid = second.get_centroid();
        Eigen::Vector4f mean_centroid;
        for (size_t i = 0; i < 4; ++i)
            mean_centroid[i] = (first_centroid[i] + second_centroid[i]) / 2;
        // Compute the minimum min bounding box
        Eigen::Vector4f first_bb_min = first.get_bb_min();
        Eigen::Vector4f second_bb_min = second.get_bb_min();
        Eigen::Vector4f min_min;
        for (size_t i = 0; i < 4; ++i)
            min_min[i] = min(first_bb_min[i], second_bb_min[i]);
        // Compute the maximum max bounding box
        Eigen::Vector4f first_bb_max = first.get_bb_max();
        Eigen::Vector4f second_bb_max = second.get_bb_max();
        Eigen::Vector4f max_max;
        for (size_t i = 0; i < 4; ++i)
            max_max[i] = max(first_bb_max[i], second_bb_max[i]);
        // Create the new pose
        result = Pose(mean_centroid, min_min, max_max);

        result = output;

        return true;
    }

    /* === OVERLAP CHECKING === */

    bool bounding_box_overlap(const Eigen::Vector4f &min1, const Eigen::Vector4f &max1, const Eigen::Vector4f &min2, const Eigen::Vector4f &max2)
    {
        // If the objects fail any of the conditions
        if (min1[0] > max2[0] || min2[0] > max1[0])
            return false;
        if (min1[1] > max2[1] || min2[1] > max1[1])
            return false;
        if (min1[2] > max2[2] || min2[2] > max1[2])
            return false;
        // Otherwise return true
        return true;
    }

    bool voxels_overlap(const vector<OcTreeKey> &first, const vector<OcTreeKey> &second, const double &threshold)
    {
        vector<OcTreeKey> overlap (first.size() + second.size());
        vector<OcTreeKey>::iterator it = set_intersection (first.begin(), first.end(),
                                                           second.begin(), second.end(),
                                                           overlap.begin(), compare_octree_key);
        overlap.resize(it - overlap.begin());

        // If the percentage of overlapping voxels is above _VOXEL_OVERLAP_THRESH then this is worthy of an overlap
        double first_overlap = (double)overlap.size()/(double)first.size();
        double second_overlap = (double)overlap.size()/(double)second.size();
        double max_overlap = first_overlap;
        if (second_overlap > first_overlap)
            max_overlap = second_overlap;
        cout << "overlap is min(" << first_overlap << "," << second_overlap << ") = " << max_overlap << endl;
        if (max_overlap  >= threshold)
        {
            return true;
        }
        else
        {
            printf(ANSI_COLOR_YELLOW  "WARN active_exploration_utils::voxels_overlap : only overlapping by %.2f!"  ANSI_COLOR_RESET "\n", max_overlap);
            return false;
        }
    }

    vector<vector<int> > segment_overlap(const vector<Pose> &first, const vector<vector<OcTreeKey> > &first_keys,
                                         const vector<Pose> &second, const vector<vector<OcTreeKey> > &second_keys, const double &threshold)
    {
        // Assumes that the poses are already transformed into the same frame
        // Create a vector for a
        vector<vector<int> > res;
        // If one of the vectors is empty
        if (first.size() == 0)
            printf(ANSI_COLOR_YELLOW  "WARN active_exploration_utils::segment_overlap : first bounding boxes vector is empty!"  ANSI_COLOR_RESET "\n");
        if (second.size() == 0)
            printf(ANSI_COLOR_YELLOW  "WARN active_exploration_utils::segment_overlap : second bounding boxes vector is empty!"  ANSI_COLOR_RESET "\n");
        if (first_keys.size() == 0)
            printf(ANSI_COLOR_YELLOW  "WARN active_exploration_utils::segment_overlap : first tree keys vector is empty!"  ANSI_COLOR_RESET "\n");
        if (second_keys.size() == 0)
            printf(ANSI_COLOR_YELLOW  "WARN active_exploration_utils::segment_overlap : second tree keys vector is empty!"  ANSI_COLOR_RESET "\n");
        // Otherwise find the overlaps
        // If size of first bounding boxes equals the size of the first tree keys
        // and the size of the second bounding boxes equals the size of the second tree keys
        if ((first.size() == first_keys.size()) && (second.size() == second_keys.size()))
        {
            res.resize(first.size());
            for (vector<Pose>::size_type i = 0; i < first.size(); ++i)
            {
                vector<int> overlaps;
                for (vector<Pose>::size_type j = 0; j < second.size(); ++j)
                {
                    // If the two objects overlap, add j to i's list of overlaps
                    if (bounding_box_overlap(first[i].get_bb_min(), first[i].get_bb_max(), second[j].get_bb_min(), second[j].get_bb_max()))
                    {
                        printf("active_exploration_utils::segment_overlap : bounding box says segment %lu overlaps with %lu\n", i, j);
                        if (voxels_overlap(first_keys[i], second_keys[j], threshold))
                        {
                            printf("active_exploration_utils::segment_overlap : voxels overlap agrees\n");
                            overlaps.push_back(j);
                        }
                        else
                        {
                            printf(ANSI_COLOR_YELLOW  "WARN active_exploration_utils::segment_overlap : voxels disagree"  ANSI_COLOR_RESET "\n");
                        }
                    }
                }
                // Store all the overlaps for i
                res[i] = overlaps;
            }
        }
        // Otherwise if first_keys and second_keys are not empty just do voxel checking
        else if (first_keys.size() > 0 && second_keys.size() > 0)
        {
            res.resize(first_keys.size());
            for (vector<vector<OcTreeKey> >::size_type i = 0; i < first_keys.size(); ++i)
            {
                vector<int> overlaps;
                for (vector<vector<OcTreeKey> >::size_type j = 0; j < second_keys.size(); ++j)
                {
                    // If the two objects overlap, add j to i's list of overlaps
                    if (voxels_overlap(first_keys[i], second_keys[j], threshold))
                    {
                        printf("active_exploration_utils::segment_overlap : voxels overlap says segment %lu overlaps with %lu\n", i, j);
                        overlaps.push_back(j);
                    }
                }
                // Store all the overlaps for i
                res[i] = overlaps;
            }
        }
        // Otherwise if either first_keys or second_keys is empty, but first and second are not empty, then just do bounding box checking
        else if ((first_keys.size() == 0 || second_keys.size() == 0) && (first.size() > 0 && second.size() > 0))
        {
            res.resize(first.size());
            for (vector<Pose>::size_type i = 0; i < first.size(); ++i)
            {
                vector<int> overlaps;
                for (vector<Pose>::size_type j = 0; j < second.size(); ++j)
                {
                    // If the two objects overlap, add j to i's list of overlaps
                    if (bounding_box_overlap(first[i].get_bb_min(), first[i].get_bb_max(), second[j].get_bb_min(), second[j].get_bb_max()))
                        overlaps.push_back(j);
                }
                // Store all the overlaps for i
                res[i] = overlaps;
            }
        }
        // Otherwise all vectors are empty and so return an empty vector

        // Return
        return res;
    }

    vector<vector<int> > segment_overlap(const vector<Pose> &first, const vector<Pose> &second, const double &threshold)
    {
        vector<vector<OcTreeKey> > first_keys;
        vector<vector<OcTreeKey> > second_keys;
        return segment_overlap(first, first_keys, second, second_keys, threshold);
    }

    vector<vector<int> > segment_overlap(const vector<vector<OcTreeKey> > &first_keys, const vector<vector<OcTreeKey> > &second_keys,
                                         const double &threshold)
    {
        vector<Pose> first;
        vector<Pose> second;
        return segment_overlap(first, first_keys, second, second_keys, threshold);
    }

    /* === PLANNING === */

    bool next_best_view(int &next_best_index, vector<double> &utilities, const OcTree &tree, const Hypothesis &hypothesis,
                        const SIM_TYPE &sim, const vector<Eigen::Vector4f> &map_locations, const double &variance,
                        const bool &do_visualize)
    {
        ROS_INFO("active_exploration_utils::next_best_view : starting");
        next_best_index = -1;

        vector<vector<int> > segments = hypothesis._segments;
        vector<Classification> class_estimates = hypothesis._class_estimates;
        vector<double> entropies = hypothesis._entropies;
        vector<int> entropy_ranking = hypothesis._entropy_ranking;
        vector<vector<EntMap> > emaps = hypothesis._emaps;
        vector<vector<InstLookUp> > instance_directories = hypothesis._instance_directories;
        vector<vector<InstToMapTF> > transforms = hypothesis._transforms;

        if (map_locations.size() == 0)
        {
            ROS_ERROR("active_exploration_utils::next_best_view : map locations is empty");
            return false;
        }
        // If the sim type is random then return a random index
        if (sim == RANDOM)
        {
            next_best_index = int_rand(0, map_locations.size());
            return true;
        }
        // If extracting the expected view and classifying it
        bool classify_view = false;
        if (sim == MIN_VIEW_CLASSIFICATION_ENTROPY || sim == MAX_VIEW_CLASSIFICATION_PROB)
            classify_view = true;
        // If need to choose the nearest view or use choose by weighting the controibution of all possible views
        bool all_model_views = true;
        if (sim == NEAREST_AREA || sim == NEAREST_MIN_ENTROPY || sim == NEAREST_MAX_CLASS_PROB)
            all_model_views = false;
        // Surface area, class entropy or recognition probability
        enum UTIL_TYPE {AREA = 0, ENTROPY = 1, PROBABILITY = 2};
        UTIL_TYPE u_type = AREA;
        if (sim == MIN_CLASS_ENTROPY || sim == MIN_CLASS_ENTROPY_UNOCCLUDED || sim == NEAREST_MIN_ENTROPY)
            u_type = ENTROPY;
        else if (sim == MAX_CLASS_PROB || sim == MAX_CLASS_PROB_UNOCCLUDED || sim == NEAREST_MAX_CLASS_PROB)
            u_type = PROBABILITY;
        // Occlusion or not
        bool unoccluded = false;
        if (sim == MAX_AREA_UNOCCLUDED || sim == MIN_CLASS_ENTROPY_UNOCCLUDED || sim == MAX_CLASS_PROB_UNOCCLUDED)
            unoccluded = true;

        // Otherwise compute the next best view by selecting the location in map_locations that has the highest utility
        // Ignore segments which are already very well classified
        vector<int> segs_for_planning;
        // Sum the entropies to get the normlization constant
        double segs_max_ent = -1;
        for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            // Get the confidence of the most likely class
            if (class_estimates[i].confidence[0] < _CONFIDENCE_THRESHOLD)
            {
                segs_for_planning.push_back (i);
                if (entropies[i] > segs_max_ent)
                    segs_max_ent = entropies[i];
            }
        }
        // If segs for planning is empty then just plan for the objct with the highest entropy
        if (segs_for_planning.size() == 0)
        {
            // If there are no rankings available
            if (entropy_ranking.size() == 0)
            {
                ROS_ERROR("active_exploration_utils::next_best_view : no available entropy rankings");
                return false;
            }
            // Otherwise
            segs_for_planning.push_back(entropy_ranking[0]);
            segs_max_ent = entropies[entropy_ranking[0]];
        }
        // Scale the entropy values
        if (segs_max_ent <= 0)
        {
            ROS_WARN("active_exploration_utils::next_best_view : scaling factor for segment entropy is %.4f", segs_max_ent);
            segs_max_ent = 1;
        }
        vector<double> uncertainty_weight;
        uncertainty_weight.resize(segs_for_planning.size());
        for (vector<int>::size_type i = 0; i < segs_for_planning.size(); ++i)
            uncertainty_weight[i] = entropies[segs_for_planning[i]]/segs_max_ent;

        // If the sim type requires the views to be extracted and classified
        if (classify_view)
        {
            next_best_index = extracted_point_cloud_next_best_view(map_locations, segs_for_planning, uncertainty_weight, sim);
        }
        else
        {
            // Compute the utility for each segment from the surface area proportions or the class entropy values
            // High surface area proportion means high utility
            // Low entropy (low uncertainty) means high utility
            // Find the maximum value and scale everything by this value (it will then be 1 and everything less)
            // If using entropy hen let [utility = 1 - scaled_entropy] to set high utility to low entropy
            vector<vector<double> > model_views_max_score;
            model_views_max_score.resize(segs_for_planning.size());
            for (vector<int>::size_type i = 0; i < segs_for_planning.size(); ++i)
            {
                int sx = segs_for_planning[i];
                model_views_max_score[i].resize(emaps[sx].size());
                for (vector<EntMap>::size_type j = 0; j < emaps[sx].size(); ++j)
                {
                    size_t max_iter;
                    if (u_type == ENTROPY)
                        max_iter = emaps[sx][j]._class_entropies.size();
                    if (u_type == PROBABILITY)
                        max_iter = emaps[sx][j]._recognition_probabilities.size();
                    else
                        max_iter = emaps[sx][j]._surface_areas.size();
                    // Iterate through each view
                    model_views_max_score[i][j] = -1;
                    for (vector<double>::size_type k = 0; k < max_iter; ++k)
                    {
                        double e;
                        if (u_type == ENTROPY)
                            e = emaps[sx][j]._class_entropies[k];
                        if (u_type == PROBABILITY)
                            e = emaps[sx][j]._recognition_probabilities[k];
                        else
                            e = emaps[sx][j]._surface_areas[k];
                        // Replace the current value if the new value is larger
                        if (e > model_views_max_score[i][j])
                            model_views_max_score[i][j] = e;
                    }
                    if (model_views_max_score[i][j] <= 0)
                    {
                        ROS_WARN("active_exploration_utils::next_best_view : scaling factor for model view score is %.4f",
                                 model_views_max_score[i][j]);
                        model_views_max_score[i][j] = 1;
                    }
                }
            }
            // Compute the scaled utilities and also transform the views into the map frame
            vector<vector<vector<double> > > scaled_model_utilities;
            scaled_model_utilities.resize(segs_for_planning.size());
            vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > model_views;
            model_views.resize(segs_for_planning.size());
            for (vector<int>::size_type i = 0; i < segs_for_planning.size(); ++i)
            {
                // Segment index
                int sx = segs_for_planning[i];
                // View index from the instance directories
                int view_ix = atoi(instance_directories[sx][0]._ix.c_str()); // most likely viewpoint
                // Resize the vectors
                scaled_model_utilities[i].resize(emaps[sx].size());
                model_views[i].resize(emaps[sx].size());
                for (vector<EntMap>::size_type j = 0; j < emaps[sx].size(); ++j)
                {
                    // Resize the model utility vector
                    if (u_type == ENTROPY)
                        scaled_model_utilities[i][j].resize(emaps[sx][j]._class_entropies.size());
                    if (u_type == PROBABILITY)
                        scaled_model_utilities[i][j].resize(emaps[sx][j]._recognition_probabilities.size());
                    else
                        scaled_model_utilities[i][j].resize(emaps[sx][j]._surface_areas.size());
                    // Resize the model views vector;
                    model_views[i][j].first.resize(emaps[sx][j]._camera_poses.size());
                    model_views[i][j].second.resize(emaps[sx][j]._camera_poses.size());
                    // Get the transform of the strongest matching viewpoint to the segment
                    int vec_ix = 0; // vector index
                    vector<int>::const_iterator it = find(emaps[sx][j]._ixs.begin(), emaps[sx][j]._ixs.end(), view_ix);
                    if (it != emaps[sx][j]._ixs.end())
                        vec_ix = distance<vector<int>::const_iterator>(emaps[sx][j]._ixs.begin(), it);
                    else
                        ROS_WARN("active_exploration_utils::next_best_view : invalid index for transform %u", vec_ix);
                    double max_scaled_val = 0;
                    for (vector<double>::size_type k = 0; k < emaps[sx][j]._surface_areas.size(); ++k)
                    {
                        if (u_type == ENTROPY)
                            scaled_model_utilities[i][j][k] = 1 - (emaps[sx][j]._class_entropies[k]/model_views_max_score[i][j]) + _EPS;
                        if (u_type == PROBABILITY)
                            scaled_model_utilities[i][j][k] = emaps[sx][j]._recognition_probabilities[k]/model_views_max_score[i][j] + _EPS;
                        else
                            scaled_model_utilities[i][j][k] = emaps[sx][j]._surface_areas[k]/model_views_max_score[i][j] + _EPS;
                        // Scale the utility value
                        if (scaled_model_utilities[i][j][k] > max_scaled_val)
                            max_scaled_val = scaled_model_utilities[i][j][k];
                        // Transform to model frame
                        // Transform = instance_to_map * inverse(segment_to_instance) * instance_view_to_model;
                        //Eigen::Matrix4f i_transform = _instances_to_map_tfs[sx][j] * _transforms_to_instances[sx][j].inverse() * emaps[sx][j]._transforms[k];  // CHANGED
                        Eigen::Matrix4f i_transform = transforms[sx][j]._transform *
                                                      emaps[sx][j]._transforms[vec_ix].inverse() *
                                                      emaps[sx][j]._transforms[k];
                        PointCloud<PointT> tr_model_v_cloud;
                        transformPointCloud(emaps[sx][j]._clouds[k], tr_model_v_cloud, i_transform);
                        PointCloud<PointT> pt;
                        pt.resize(1);
                        pt.points[0].x = emaps[sx][j]._camera_poses[k][0];
                        pt.points[0].y = emaps[sx][j]._camera_poses[k][1];
                        pt.points[0].z = emaps[sx][j]._camera_poses[k][2];
                        transformPointCloud(pt, pt, i_transform);
                        model_views[i][j].first[k] = tr_model_v_cloud;
                        model_views[i][j].second[k] = Eigen::Vector4f(pt.points[0].x, pt.points[0].y, pt.points[0].z, 0);
                    }
                    // Re adjust the scaled utility values to the range (0,1)
                    for (vector<double>::size_type k = 0; k < scaled_model_utilities[i][j].size(); ++k)
                        scaled_model_utilities[i][j][k] = scaled_model_utilities[i][j][k]/max_scaled_val;
                }
            }

            // Compute the utility for each location
            if (all_model_views)
                next_best_index = gaussian_weighted_next_best_view(utilities, tree, hypothesis, map_locations, segs_for_planning,
                                                                   model_views, scaled_model_utilities, uncertainty_weight, variance,
                                                                   unoccluded, do_visualize);
            else
                next_best_index = nearest_next_best_view(utilities, class_estimates, map_locations, segs_for_planning, model_views,
                                                         scaled_model_utilities, uncertainty_weight);
        }
        // Check that the index is valid
        if (next_best_index < 0)
        {
            ROS_ERROR("active_exploration_utils::next_best_view : invalid next best view");
            return false;
        }

        ROS_INFO("active_exploration_utils::next_best_view : finished");

        return true;
    }

    bool next_best_view(int &next_best_index, const OcTree &tree, const Hypothesis &hypothesis, const SIM_TYPE &sim,
                        const vector<Eigen::Vector4f> &map_locations, const double &variance, const bool &do_visualize)
    {
        vector<double> utilities;
        return next_best_view(next_best_index, utilities, tree, hypothesis, sim, map_locations, variance, do_visualize);
    }

    int nearest_next_best_view(vector<double> &utilities, const vector<Classification> &class_estimates,
                               const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                               const vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > &model_views,
                               const vector<vector<vector<double> > > &scaled_model_utilities, const vector<double> &uncertainty_weight)
    {
        // This function does not compute utilities using the objective function, therefore keep it empty
        utilities.clear();

        // For each segment find the closest location in the map
        vector<pair<int,double> > nearest_indices;
        for (vector<int>::size_type i = 0; i < seg_indices.size(); ++i)
        {
            int sx = seg_indices[i];
            // For each class that this segment could be
            for (vector<Classification>::size_type j = 0; j < class_estimates[sx].confidence.size(); ++j)
            {
                // Get the index of the best view
                int best_ix = max_element(scaled_model_utilities[i][j].begin(), scaled_model_utilities[i][j].end())
                              - scaled_model_utilities[i][j].begin();
                // Get the utility
                double u = scaled_model_utilities[i][j][best_ix];
                // Scale the utility by the confidence of the class
                u *= class_estimates[sx].confidence[j];
                // Scale the utility by the uncertainty of the segment
                u *= uncertainty_weight[i];
                // Get the map location
                Eigen::Vector4f max_area_location = model_views[i][j].second[best_ix];
                // Get the map location closest to this location
                int clos_ix = 0;
                double clos_dist = eigdistance3D(max_area_location, map_locations[0]);
                for (vector<Eigen::Vector4f>::size_type k = 0; k < map_locations.size(); ++k)
                {
                    double d = eigdistance3D(max_area_location, map_locations[k]);
                    if (d < clos_dist)
                    {
                        clos_ix = k;
                        clos_dist = d;
                    }
                }
                // Store in the nearest_indices vector
                if (clos_ix >= 0)
                {
                    // Loop through the vector and if the index has already been entered then add this utility
                    bool added_to_existing = false;
                    for (vector<pair<int,double> >::size_type k = 0; k < nearest_indices.size(); ++k)
                    {
                        // If this index is the same
                        if (nearest_indices[k].first == clos_ix)
                        {
                            added_to_existing = true;
                            // Get the previous utility
                            double u_prev = nearest_indices[k].second;
                            // Replace
                            nearest_indices[k] = make_pair(clos_ix, u + u_prev);
                            break;
                        }
                    }
                    // If the index was not found
                    if (!added_to_existing)
                        nearest_indices.push_back(make_pair(clos_ix, u));
                }
            }
        }
        // If elements in the vector
        if (nearest_indices.size() > 0)
        {
            // Sort the vector of indices and utilities
            sort(nearest_indices.begin(), nearest_indices.end(), compare<double>);
            // Want the maximum utility, so return the last element
            return nearest_indices.back().first;
        }
        else
        {
            ROS_ERROR("active_exploration_utils::next_best_view : did not add any locations to the nearest_indices vector");
            return -1;
        }
    }

    int nearest_next_best_view(const vector<Classification> &class_estimates,
                               const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                               const vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > &model_views,
                               const vector<vector<vector<double> > > &scaled_model_utilities, const vector<double> &uncertainty_weight)
    {
        vector<double> utilities;
        return nearest_next_best_view(utilities, class_estimates, map_locations, seg_indices, model_views, scaled_model_utilities,
                                      uncertainty_weight);
    }

    int gaussian_weighted_next_best_view(vector<double> &utilities, const OcTree &tree, const Hypothesis &hypothesis,
                                         const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                                         const vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > &model_views,
                                         const vector<vector<vector<double> > > &scaled_model_utilities,
                                         const vector<double> &uncertainty_weight, const double &variance, const bool &unoccluded,
                                         const bool &do_visualize)
    {
        vector<Classification> class_estimates = hypothesis._class_estimates;
        vector<vector<EntMap> > emaps = hypothesis._emaps;
        vector<vector<InstLookUp> > instance_directories = hypothesis._instance_directories;
        vector<vector<OcTreeKey> > octree_keys = hypothesis._octree_keys;
        vector<vector<InstToMapTF> > transforms = hypothesis._transforms;
        utilities.clear();
        utilities.resize(map_locations.size());

        // Do one visualization if do_visualize is set to true
        bool single_vis = false;
        // WARN : no visualization ever when commented out
        if (do_visualize)
            single_vis = true;
        for (vector<Eigen::Vector4f>::size_type i = 0; i < map_locations.size(); ++i)
        {
            cout << " * * * LOCATION " << i << endl;
            utilities[i] = 0;
            // Consider the contribution from each segment that needs to be viewed
            vector<vector<PointCloud<PointT> > > expected_clouds;
            expected_clouds.resize(seg_indices.size());
            vector<vector<vector<int> > > visible_indices;
            visible_indices.resize(seg_indices.size());
            for (vector<int>::size_type j = 0; j < seg_indices.size(); ++j)
            {
                int sx = seg_indices[j];
                //_entropies.push_back ((double)entropy(it->confidence));
                // Sum the values at this location for each class hypothesis
                double seg_utility = 0;
                for (vector<EntMap>::size_type k = 0; k < emaps[sx].size(); ++k)
                {
                    // Compute the utility
                    double u = location_utility(map_locations[i], model_views[j][k].second, scaled_model_utilities[j][k], variance);

    //                if (single_vis)
    //                {
    //                    // Get the most likely view and use that to transform the model into
    //                    int view_ix = atoi(instance_directories[sx][0]._ix.c_str()); // most likely viewpoint
    //                    int vec_ix = 0; // vector index
    //                    vector<int>::const_iterator it = find(emaps[sx][k]._ixs.begin(), emaps[sx][k]._ixs.end(), view_ix);
    //                    if (it != emaps[sx][j]._ixs.end())
    //                    {
    //                        vec_ix = distance<vector<int>::const_iterator>(emaps[sx][j]._ixs.begin(), it);
    //                        Eigen::Matrix4f itf = emaps[sx][j]._transforms[vec_ix];
    //                        if (!visualize_in_model_frame(map_locations[i], emaps[sx][k], transforms[sx][k]._transform, itf))
    //                            ROS_WARN("active_exploration_utils::gaussian_weighted_next_best_view : could not visualize location in model frame");
    //                    }
    //                    else
    //                    {
    //                        ROS_WARN("active_exploration_utils::gaussian_weighted_next_best_view : invalid index for transform %u", vec_ix);
    //                    }
    //                    single_vis = false;
    //                }

                    // Reduce the utility by the percentage of how many points will actually be visible, when viewed from this location
                    if (unoccluded)
                    {
                        PointCloud<PointT> exp_pc;
                        vector<int> vis_ix;
                        double percent_visible = percentage_visible_points(tree, map_locations[i], instance_directories[sx][k], emaps[sx][k],
                                                                           transforms[sx][k]._transform, octree_keys[sx], exp_pc, vis_ix);
                        u *= percent_visible;
                        expected_clouds[j].push_back(exp_pc);
                        visible_indices[j].push_back(vis_ix);
                    }
                    // Reduce the utility by the confidence of this class
                    //cout << "Hypothesis utility " << u << endl;
                    u *= class_estimates[sx].confidence[k];
                    //cout << "Class confidence " << class_estimates[sx].confidence[k] << endl;
                    //cout << "Scaled hypothesis utility " << u << endl;
                    seg_utility += u;
                }
                // Weight the utility of the segment by its uncertainty
                // If the segment is very uncertaint then maximum weight, otherwise reduce the weight if it is well known
                //cout << "Seg utility " << seg_utility << endl;
                seg_utility *= uncertainty_weight[j];
                //cout << "Uncertainty weight " << uncertainty_weight[j] << endl;
                //cout << "Scaled seg utility " << seg_utility << endl;
                // Add to the list of utilities
                utilities[i] += seg_utility;
            }
    //        if (unoccluded && _visualization_on)
    //        {
    //            if (!visualize_expected_clouds(map_locations[i], seg_indices, expected_clouds, visible_indices))
    //                ROS_WARN("ActiveExploration::gaussian_weighted_next_best_view : could not visualize expected point clouds");
    //        }
        }
        // Determine the location with the maximum utility
        return distance(utilities.begin(), max_element(utilities.begin(), utilities.end()));
    }

    int gaussian_weighted_next_best_view(const OcTree &tree, const Hypothesis &hypothesis,
                                         const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                                         const vector<vector<pair<vector<PointCloud<PointT> >,vector<Eigen::Vector4f> > > > &model_views,
                                         const vector<vector<vector<double> > > &scaled_model_utilities,
                                         const vector<double> &uncertainty_weight, const double &variance, const bool &unoccluded,
                                         const bool &do_visualize)
    {
        vector<double> utilities;
        return gaussian_weighted_next_best_view(utilities, tree, hypothesis, map_locations, seg_indices, model_views,
                                                scaled_model_utilities, uncertainty_weight, variance, unoccluded, do_visualize);
    }

    int extracted_point_cloud_next_best_view(const vector<Eigen::Vector4f> &map_locations, const vector<int> &seg_indices,
                                             const vector<double> &uncertainty_weight, const SIM_TYPE &sim)
    {
        return 0;
    }

    double location_utility(const Eigen::Vector4f &location, const vector<Eigen::Vector4f> &training_locations,
                            const vector<double> &training_utilities, const double &variance)
    {
        // If one of the vectors is empty then return 0
        if (training_locations.size() == 0)
        {
            ROS_ERROR("planning_utils::location_utility : training locations is empty");
            return 0;
        }
        if (training_utilities.size() == 0)
        {
            ROS_ERROR("planning_utils::location_utility : training utilities is empty");
            return 0;
        }

        // Check the sizes of the vectors
        int N_MAX = training_locations.size();
        if (training_utilities.size() != N_MAX)
        {
            ROS_WARN("planning_utils::location_utility : training locations has size %lu and training utilities has size %lu",
                     training_locations.size(), training_utilities.size());
            // Choose the smallest of the two sizes
            if (training_utilities.size() < N_MAX)
                N_MAX = training_utilities.size();
        }
        // Sum and normalize the utility contribution from the training locations
        // Weight the utility contribution of each training location by the distance from the location in a gaussian
        double u = 0;
        for (int i = 0; i < N_MAX; ++i)
            u += training_utilities[i] * gaussian_func(eigdistance3D(location, training_locations[i]), variance);
        // Return the normalized value
        return u/N_MAX;
    }

    PointCloud<PointT> get_expected_point_cloud(const Eigen::Vector4f &location_in_map, const EntMap &emap, const int &emap_ix,
                                                const Eigen::Matrix4f instance_to_map_tfs)
    {
        // Transform the map_location to the model frame
        // transform = 1) transform location from map to instance
        //             2) transform instance to model
        // transform = _emap[emap_ix]._transform * instance_to_map_tfs.inverse()
        Eigen::Matrix4f map_to_model_tf = emap._transforms[emap_ix] * instance_to_map_tfs.inverse();
        // Transform the test location into the model frame
        PointCloud<PointT> p;
        p.resize(1);
        p.points[0].x = location_in_map[0];
        p.points[0].y = location_in_map[1];
        p.points[0].z = location_in_map[2];
        transformPointCloud(p, p, map_to_model_tf);
        Eigen::Vector4f map_loc_model_frame (p.points[0].x, p.points[0].y, p.points[0].z, 0);
        // Read in the octree from file
        OcTree tree (emap._octree_file);
        // Go through every point in the emap cloud and find if it is visible from the location
        PointCloud<PointT> result;
        if (tree.size() > 0)
        {
            // Downsample the point cloud
            //VoxelGrid<PointT> grid;
            //grid.setLeafSize ((float)_DOWNSAMPLE_FOR_VIS_CHECKING, (float)_DOWNSAMPLE_FOR_VIS_CHECKING, (float)_DOWNSAMPLE_FOR_VIS_CHECKING);
            PointCloud<PointT>::Ptr ds_cloud (new PointCloud<PointT>(emap._instance_cloud));
            //grid.setInputCloud (ds_cloud);
            //grid.filter (*ds_cloud);

            // Compute the key that each point belongs to
            map<OcTreeKey,vector<int>,compare_octree_key_struct> map_key_to_points;
            for (size_t i = 0; i < ds_cloud->size(); ++i)
            {
                OcTreeKey key;
                if (tree.coordToKeyChecked(ds_cloud->points[i].x, ds_cloud->points[i].y, ds_cloud->points[i].z, key))
                    map_key_to_points[key].push_back(i);
            }

            vector<int> visible_indices = get_visible_points_in_cloud(tree, map_loc_model_frame, *ds_cloud, map_key_to_points);
            // Transform the cloud back to the map frame
            if (visible_indices.size() > 0)
            {
                Eigen::Matrix4f model_to_map_tf = instance_to_map_tfs * emap._transforms[emap_ix].inverse();
                transformPointCloud(*ds_cloud, visible_indices, result, model_to_map_tf);
            }
        }

        return result;
    }

    //vector<int> get_visible_points_in_cloud(const OcTree &tree, const Eigen::Vector4f &origin, const PointCloud<PointT> &cloud,
    //                                        const map<OcTreeKey,vector<int>,compare_octree_key_struct> &keys_to_points,
    //                                        const vector<OcTreeKey> &ignore_keys)
    //{
    //    // Find the visible points in the cloud that are not occluded in the octree
    //    vector<int> result;
    //    // Keep a sorted list of the ignore_keys for faster look up
    //    vector<OcTreeKey> sorted_ignore_keys = ignore_keys;
    //    if (sorted_ignore_keys.size() > 0)
    //    {
    //        // Sort
    //        sort(sorted_ignore_keys.begin(), sorted_ignore_keys.end(), compare_octree_key);
    //        // Remove duplicates
    //        sorted_ignore_keys.erase(unique(sorted_ignore_keys.begin(), sorted_ignore_keys.end(), equal_octree_key), sorted_ignore_keys.end());
    //    }
    //    // Compute the key of the origin
    //    point3d origin_coord (origin[0], origin[1], origin[2]);
    //    OcTreeKey origin_key;
    //    bool check_origin_key = tree.coordToKeyChecked(origin_coord, origin_key);
    //    double tree_res = tree.getResolution();
    //    cout << "Casting ray and finding visible points in cloud with size " << cloud.size() << endl;
    //    cout << "Tree has size " << tree.size() << endl;
    //    cout << "Tree has number of leaves " << tree.getNumLeafNodes() << endl;
    //    cout << "Tree resolution is " << tree.getResolution() << endl;
    //    cout << "Tree node size is " << tree.getNodeSize(tree.getTreeDepth()) << endl;
    //    cout << "Keys to points size is " << keys_to_points.size() << endl;
    //    // Cast a ray from origin to each point in the cloud
    //    for (size_t i = 0; i < cloud.size(); ++i)
    //    {
    //        // Store a boolean for if the point is visible
    //        bool point_is_visible = true;
    //        // To a octomap::point3d
    //        point3d cloud_coord (cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    //        // Compute the key of the current point
    //        OcTreeKey cloud_key;
    //        bool check_cloud_key = tree.coordToKeyChecked(cloud_coord, cloud_key);
    //        // Cast a ray from the origin to the cloud point
    //        KeyRay kr;
    //        if (tree.computeRayKeys(origin_coord, cloud_coord, kr))
    //        {
    //            // Check each key and and if all are not occupied then this point is visible from the origin
    //            vector<OcTreeKey> keys_passed;
    //            for (KeyRay::iterator kit = kr.begin(), end = kr.end(); kit != end; ++kit)
    //            {
    //                // Ignore the voxel of the origin and the voxel of the query point
    //                bool check_occupancy = true;
    //                if (check_origin_key)
    //                {
    //                    if (equal_octree_key(origin_key, *kit))
    //                        check_occupancy = false;
    //                }
    //                if (check_cloud_key)
    //                {
    //                    if (equal_octree_key(cloud_key, *kit))
    //                        check_occupancy = false;
    //                }
    //                // Ignore keys in the ignore_keys list
    //                if (sorted_ignore_keys.size() > 0)
    //                {
    //                    // Check if the current key is within the ignore list by set overlap
    //                    vector<OcTreeKey> this_key_list;
    //                    this_key_list.push_back(*kit);
    //                    vector<OcTreeKey> overlap (ignore_keys.size() + 1);
    //                    vector<OcTreeKey>::iterator int_it = set_intersection (sorted_ignore_keys.begin(), sorted_ignore_keys.end(),
    //                                                                           this_key_list.begin(), this_key_list.end(),
    //                                                                           overlap.begin(), compare_octree_key);
    //                    overlap.resize(int_it - overlap.begin());
    //                    if (overlap.size() > 0)
    //                        check_occupancy = false;
    //                }
    //                // If need to check this key
    //                if (check_occupancy)
    //                {
    //                    OcTreeNode *node = tree.search(*kit);
    //                    if (node)
    //                    {
    //                        // If the node is occupied then break,
    //                        // nodes are returned in increasing distance so any after this are not
    //                        // visible because of the first occupied cell that is encountered
    //                        if (node->getOccupancy() > 0.9)
    //                        {
    //                            point_is_visible = false;
    //                            break;
    ////                            keys_passed.push_back(*kit);
    ////                            // If more than two voxels passed, then this point is not visible
    ////                            if (keys_passed.size() > 2)
    ////                            {
    ////                                point_is_visible = false;
    ////                                break;
    ////                            }
    ////                            // If two voxels are passed then check if they are neighbours
    ////                            // If they are neighbours then do not reject yet
    ////                            else if (keys_passed.size() == 2)
    ////                            {
    ////                                // Compute distance between the two keys
    ////                                point3d p0 = tree.keyToCoord(keys_passed[0]);
    ////                                point3d p1 = tree.keyToCoord(keys_passed[1]);
    ////                                // Compute the distance
    ////                                if (p0.distance(p1) > tree_res)
    ////                                {
    ////                                    point_is_visible = true;
    ////                                    break;
    ////                                }
    ////                            }
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //        else
    //        {
    //            printf("WARN octomap_utils::get_visible_points_in_cloud : error casting ray to [%.2f %.2f %.2f]\n",
    //                     cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    //            point_is_visible = false;
    //        }
    //        // If the point is visible then add the index to the result
    //        if (point_is_visible)
    //            result.push_back(i);
    //    }
    //    // Return
    //    return result;
    //}

    vector<int> get_visible_points_in_cloud(const OcTree &tree, const Eigen::Vector4f &origin, const PointCloud<PointT> &cloud,
                                            const map<OcTreeKey,vector<int>,compare_octree_key_struct> &keys_to_points,
                                            const vector<OcTreeKey> &ignore_keys)
    {
        // Find the visible points in the cloud that are not occluded in the octree
        vector<int> result;
        // Keep a sorted list of the ignore_keys for faster look up
        vector<OcTreeKey> sorted_ignore_keys = ignore_keys;
        if (sorted_ignore_keys.size() > 0)
        {
            // Sort
            sort(sorted_ignore_keys.begin(), sorted_ignore_keys.end(), compare_octree_key);
            // Remove duplicates
            sorted_ignore_keys.erase(unique(sorted_ignore_keys.begin(), sorted_ignore_keys.end(), equal_octree_key), sorted_ignore_keys.end());
        }
        // Compute the key of the origin
        point3d origin_coord (origin[0], origin[1], origin[2]);
        OcTreeKey origin_key;
        bool check_origin_key = tree.coordToKeyChecked(origin_coord, origin_key);
    //    double tree_res = tree.getResolution();
    //    cout << "Casting ray and finding visible points in cloud with size " << cloud.size() << endl;
    //    cout << "Tree has size " << tree.size() << endl;
    //    cout << "Tree has number of leaves " << tree.getNumLeafNodes() << endl;
    //    cout << "Tree resolution is " << tree.getResolution() << endl;
    //    cout << "Tree node size is " << tree.getNodeSize(tree.getTreeDepth()) << endl;
    //    cout << "Keys to points size is " << keys_to_points.size() << endl;
        // Cast a ray from origin to each point in the cloud
        map<OcTreeKey,vector<int>,compare_octree_key_struct>::const_iterator it = keys_to_points.begin();
        map<OcTreeKey,vector<int>,compare_octree_key_struct>::const_iterator it_end = keys_to_points.end();
        for (it; it != it_end; ++it)
        {
            // Store a boolean for if the point is visible
            bool point_is_visible = true;
            // To a octomap::point3d
            point3d voxel_coord = tree.keyToCoord(it->first);
            // Cast a ray from the origin to the cloud point
            KeyRay kr;
            if (tree.computeRayKeys(origin_coord, voxel_coord, kr))
            {
                // Check each key and and if all are not occupied then this point is visible from the origin
                for (KeyRay::iterator kit = kr.begin(), end = kr.end(); kit != end; ++kit)
                {
                    // Ignore the voxel of the origin and the voxel of the query point
                    bool check_occupancy = true;
                    if (equal_octree_key(it->first, *kit))
                        check_occupancy = false;
                    if (check_origin_key)
                    {
                        if (equal_octree_key(origin_key, *kit))
                            check_occupancy = false;
                    }
                    // Ignore keys in the ignore_keys list
                    if (sorted_ignore_keys.size() > 0)
                    {
                        // Check if the current key is within the ignore list by set overlap
                        vector<OcTreeKey> this_key_list;
                        this_key_list.push_back(*kit);
                        vector<OcTreeKey> overlap (ignore_keys.size() + 1);
                        vector<OcTreeKey>::iterator int_it = set_intersection (sorted_ignore_keys.begin(), sorted_ignore_keys.end(),
                                                                               this_key_list.begin(), this_key_list.end(),
                                                                               overlap.begin(), compare_octree_key);
                        overlap.resize(int_it - overlap.begin());
                        if (overlap.size() > 0)
                            check_occupancy = false;
                    }
                    // If need to check this key
                    if (check_occupancy)
                    {
                        OcTreeNode *node = tree.search(*kit);
                        if (node)
                        {
                            // If the node is occupied then break,
                            // nodes are returned in increasing distance so any after this are not
                            // visible because of the first occupied cell that is encountered
                            if (node->getOccupancy() > 0.5)
                            {
                                point_is_visible = false;
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                printf(ANSI_COLOR_YELLOW "WARN utils::get_visible_points_in_cloud : error casting ray to [%.2f %.2f %.2f]" ANSI_COLOR_RESET "\n",
                         voxel_coord.x(), voxel_coord.y(), voxel_coord.z());
                point_is_visible = false;
            }
            // If the point is visible then add the index to the result
            if (point_is_visible)
                result.insert(result.end(), it->second.begin(), it->second.end());
        }
        // Sort the indices
        sort(result.begin(), result.end());
        // Remove duplicates
        result.erase(unique(result.begin(), result.end()), result.end());
        // Return
        return result;
    }

    double percentage_visible_points(const OcTree &tree, const Eigen::Vector4f &location, const InstLookUp &inst_dir, const EntMap &emap,
                                     const Eigen::Matrix4f &transform, const vector<OcTreeKey> &ignore_keys,
                                     PointCloud<PointT> &expected_cloud, vector<int> &visible_indices)
    {
        expected_cloud.clear();
        visible_indices.clear();

        // Get the transform into the model frame
        int view_ix = atoi(inst_dir._ix.c_str()); // most likely viewpoint
        int vec_ix = 0; // vector index
        vector<int>::const_iterator it = find(emap._ixs.begin(), emap._ixs.end(), view_ix);
        if (it != emap._ixs.end())
        {
            vec_ix = distance<vector<int>::const_iterator>(emap._ixs.begin(), it);
            // Get the point cloud from the model
            //cout << "Num points in model cloud = " << emap._instance_cloud.size() << endl;
            expected_cloud = get_expected_point_cloud(location, emap, vec_ix, transform);
            //cout << "Num points in expected cloud = " << exp_pc.size() << endl;
            // Compute the key that each point belongs to
            map<OcTreeKey,vector<int>,compare_octree_key_struct> map_key_to_points;
            for (size_t kk = 0; kk < expected_cloud.size(); ++kk)
            {
                OcTreeKey key;
                if (tree.coordToKeyChecked(expected_cloud.points[kk].x, expected_cloud.points[kk].y, expected_cloud.points[kk].z, key))
                    map_key_to_points[key].push_back(kk);
            }
            visible_indices = get_visible_points_in_cloud(tree, location, expected_cloud, map_key_to_points, ignore_keys);
            //cout << "Num points visible in expected cloud = " << vis_ix.size() << endl;
            return (double)visible_indices.size() / (double)expected_cloud.size();
        }
        else
        {
            printf(ANSI_COLOR_YELLOW "WARN utils::percentage_visible_points : cannot find view index %u in entropy map %s/%s" ANSI_COLOR_RESET "\n",
                   view_ix, inst_dir._class_type.c_str(), inst_dir._instance_name.c_str());
            return 1.0;
        }
    }

    vector<int> keys_to_point_indices(const OcTree &tree, const vector<OcTreeKey> &keys,
                                      const map<OcTreeKey,vector<int>,compare_octree_key_struct> &map_key_to_points, const vector<OcTreeKey> &ignore_keys)
    {
        vector<int> result;

        map<OcTreeKey,vector<int>,compare_octree_key_struct> copy_key_to_points = map_key_to_points;

        for (vector<OcTreeKey>::size_type i = 0; i < keys.size(); ++i)
        {
            OcTreeNode *node = tree.search(keys[i]);
            if (node)
            {
                if (node->getOccupancy() > 0.5)
                {
                    vector<OcTreeKey> v;
                    if (ignore_keys.size() > 0)
                    {
                        v.resize(ignore_keys.size() + 1);
                        vector<OcTreeKey> temp;
                        temp.push_back(keys[i]);
                        vector<OcTreeKey>::iterator iter = set_intersection (ignore_keys.begin(), ignore_keys.end(),
                                                                             temp.begin(), temp.end(), v.begin(), compare_octree_key);
                        v.resize(iter-v.begin());
                    }
                    if (v.size() == 0)
                    {
                        vector<int> ixx = copy_key_to_points[keys[i]];
                        if (ixx.size() > 0)
                            result.insert(result.end(), ixx.begin(), ixx.end());
                    }
                }
            }
        }
        // If there are points found
        if (result.size() > 0)
        {
            sort(result.begin(), result.end());
            result.erase(unique(result.begin(), result.end()), result.end());
        }

        return result;
    }

    /* === TRANSFORMATION === */

    bool transform_pose(const Pose &in, Pose &out, const Eigen::Matrix4f &transform)
    {
        // If the input is invalid
        if (!in.is_valid())
        {
            ROS_WARN("active_exploration_utils::transform_pose : input pose is invalid");
            out = in;
            return false;
        }
        // If it is an identity matrix
        if (transform.isIdentity())
        {
            ROS_WARN("active_exploration_utils::transform_pose : input transform is an identity matrix");
            out = in;
            return true;
        }
        // Otherwise do the transformation
        Eigen::Vector4f centroid = transform_eigvec(in.get_centroid(), transform);
        Eigen::Vector4f bb_min = transform_eigvec(in.get_bb_min(), transform);
        Eigen::Vector4f bb_max = transform_eigvec(in.get_bb_max(), transform);
        out = Pose (centroid, bb_min, bb_max);

        return true;
    }

    bool transform_pose(const vector<Pose> &in, vector<Pose> &out, const Eigen::Matrix4f &transform)
    {
        // If it is an identity matrix
        if (transform.isIdentity())
        {
            ROS_WARN("active_exploration_utils::transform_pose : input transform is an identity matrix");
            out = in;
            return true;
        }
        // Otherwise transform each element
        out.clear();
        out.resize(in.size());
        bool success = true;
        for (vector<Pose>::size_type i = 0; i < in.size(); ++i)
        {
            Pose p;
            if (transform_pose(in[i], p, transform))
            {
                out[i] = p;
            }
            else
            {
                ROS_WARN("active_exploration_utils::transform_pose : pose %lu is invalid", i);
                out[i] = in[i];
                success = false;
            }
        }

        return success;
    }

    /* === DATA TYPE CONVERSION === */

    bool fromROSMsg(const squirrel_object_perception_msgs::EntropyMap &in, EntMap &out)
    {
        try
        {
            // Clear the data
            out._ixs.clear();
            out._clouds.clear();
            out._transforms.clear();
            out._centroids.clear();
            out._camera_poses.clear();
            out._camera_orientations.clear();
            out._surface_areas.clear();
            out._class_entropies.clear();
            out._recognition_probabilities.clear();
            // Set the cloud
            PointCloud<PointT> total_cloud;
            pcl::fromROSMsg(in.response.cloud, total_cloud);
            out._instance_cloud = total_cloud;
            // Get the octree filename
            out._octree_file = in.response.octree_file.data;
            // For each view
            for (size_t i = 0; i < in.response.entropy_map.size(); ++i)
            {
                // View index
                out._ixs.push_back(atoi(in.response.entropy_map[i].view_name.data.c_str()));
                // Point cloud
                PointCloud<PointT> cloud;
                pcl::fromROSMsg(in.response.entropy_map[i].cloud, cloud);
                out._clouds.push_back (cloud);
                // Transform to the model
                std_msgs::Float64MultiArray in_t = in.response.entropy_map[i].transform;
                Eigen::Matrix4f out_t = Eigen::Matrix4f::Identity();
                int r = 0, c = 0;
                for (int m = 0; m < in_t.data.size(); ++m)
                {
                    out_t(r,c) = in_t.data[m];
                    ++c;
                    if (c == 4)
                    {
                        ++r;
                        c = 0;
                    }
                }
                out._transforms.push_back(out_t);
                // Centroid
                Eigen::Vector4f centroid = Eigen::Vector4f(0,0,0,0);
                for (int j = 0; j < 4; ++j)
                    centroid[j] = in.response.entropy_map[i].centroid[0];
                out._centroids.push_back (centroid);
                // Camera Pose and orientation
                geometry_msgs::Pose ros_pos = in.response.entropy_map[i].camera_pose;
                Eigen::Vector4f camera_pos = Eigen::Vector4f(0,0,0,0);
                camera_pos[0] = ros_pos.position.x;
                camera_pos[1] = ros_pos.position.y;
                camera_pos[2] = ros_pos.position.z;
                out._camera_poses.push_back (camera_pos);
                Eigen::Quaternionf camera_ori (ros_pos.orientation.w, ros_pos.orientation.x,
                                               ros_pos.orientation.y, ros_pos.orientation.z);
                out._camera_orientations.push_back (camera_ori);
                // Surface area
                out._surface_areas.push_back(in.response.entropy_map[i].surface_area_proportion);
                // Class entropy
                out._class_entropies.push_back(in.response.entropy_map[i].class_entropy);
                // Recognition probability
                out._recognition_probabilities.push_back(in.response.entropy_map[i].recognition_probability);
            }
            out._valid = true;
            return true;
        }
        catch (...)
        {
            ROS_ERROR("active_exploration_utils::fromROSMsg : could not convert ros message to EntMap");
            return false;
        }
    }

    /* === VISUALIZATION === */

    bool set_visualization_image(const PointCloud<PointT> &cloud, const sensor_msgs::Image &image, ros::ServiceClient &viz_init_client)
    {
        ROS_INFO("active_exploration_utils::set_visualization_image : starting");
        if (cloud.size() > 0 && image.data.size() > 0)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(cloud, msg);
            squirrel_object_perception_msgs::SegmentVisualizationInit viz_init_srv;
            viz_init_srv.request.cloud = msg;
            viz_init_srv.request.saliency_map = image;
            if (!viz_init_client.call(viz_init_srv))
            {
                ROS_ERROR("active_exploration_utils::set_visualization_image : could not initialize visualizer");
                return false;
            }
        }
        else
        {
            ROS_WARN("active_exploration_utils::set_visualization_image : skipping visualization initialization, cloud has %lu points and image has %lu pixels",
                     cloud.size(), image.data.size());
        }

        ROS_INFO("active_exploration_utils::set_visualization_image : finished");
        return true;
    }

    bool visualize_segmentation(const OcTree &tree, const Hypothesis &hypothesis,
                                const map<OcTreeKey,vector<int>,compare_octree_key_struct> &map_key_to_points,
                                const vector<OcTreeKey> &current_ground_keys, ros::ServiceClient &viz_client)
    {
        ROS_INFO("active_exploration_utils::visualize_segmentation : starting");

        vector<vector<int> > segments = hypothesis._segments;
        vector<Classification> class_estimates = hypothesis._class_estimates;
        vector<Pose> poses = hypothesis._poses;
        vector<double> entropies = hypothesis._entropies;
        vector<int> entropy_ranking = hypothesis._entropy_ranking;
        vector<vector<OcTreeKey> > octree_keys = hypothesis._octree_keys;
        squirrel_object_perception_msgs::SegmentVisualizationOnce viz_srv;

        for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            // Wait for input
            bool false_points = false;
            cout << "Display segment ";
            cin.ignore();
            std_msgs::Int32MultiArray seg;
            if (segments[i].size() > 0)
            {
                seg.data = segments[i];
            }
            else
            {
                false_points = true;
                // Get the points that are located in the octomap
                vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(tree, octree_keys[i], map_key_to_points,
                                                                                             current_ground_keys);
                seg.data = unknown_seg_ix;
            }
            vector<std_msgs::Int32MultiArray> cluster_indices;
            cluster_indices.push_back (seg);
            viz_srv.request.clusters_indices = cluster_indices;
            if (viz_client.call(viz_srv))
            {
                // Print the size of the segment
                ROS_INFO("Segment %lu : %lu", i, segments[i].size());
                if (false_points)
                    ROS_WARN("points are false");
                // Print the pose
                if (poses.size() > 0 && i < poses.size())
                {
                    ROS_INFO("Pose :");
                    Eigen::Vector4f centroid = poses[i].get_centroid();
                    ROS_INFO("  centroid %.2f %.2f %.2f", centroid[0], centroid[1], centroid[2]);
                    Eigen::Vector4f bb_min = poses[i].get_bb_min();
                    ROS_INFO("  min      %.2f %.2f %.2f", bb_min[0], bb_min[1], bb_min[2]);
                    Eigen::Vector4f bb_max = poses[i].get_bb_max();
                    ROS_INFO("  max      %.2f %.2f %.2f", bb_max[0], bb_max[1], bb_max[2]);
                }
                // Print the number of octree voxels for this segment
                if (octree_keys.size() > 0 && i < octree_keys.size())
                {
                    ROS_INFO("Octree keys :");
                    ROS_INFO("   num %lu", octree_keys[i].size());
                }
                // Print the classification result
                if (class_estimates.size() > 0 && i < class_estimates.size())
                {
                    ROS_INFO("Classification :");
                    for (size_t j = 0; j < class_estimates[i].class_type.size(); ++j)
                    {
                        ROS_INFO("  %-15s %.2f", class_estimates[i].class_type[j].data.c_str(),
                                                 class_estimates[i].confidence[j]);
                        ROS_INFO("     pose file %s", class_estimates[i].pose[j].data.c_str());
                    }
                }
                // Print the entropy
                if (entropies.size() > 0 &&  i < entropies.size())
                {
                    ROS_INFO("Entropy :");
                    ROS_INFO("  entropy = %.2f", entropies[i]);
                }
            }
            else
            {
                ROS_INFO("active_exploration_utils::visualize_segmentation : could not visualize segment %lu", i);
                return false;
            }
        }
        cout << "Continue with processing " << endl;
        cin.ignore();
        // Print the entropy rankings of the segments
        ROS_INFO("The 10 highest entropies:");
        if (entropy_ranking.size() > 0)
        {
            for (int i = 0; i < 10; ++i)
            {
                // Check this is a valid index
                if (i <= (entropy_ranking.size()-1) &&
                    i <= (entropies.size()-1) &&
                    i <= (class_estimates.size()-1) &&
                    i <= (segments.size()-1))
                {
                    int ix = entropy_ranking[i];
                    printf("  segment %d - points %lu, class %s, confidence %.2f, entropy %.2f\n",
                             ix, segments[ix].size(), class_estimates[ix].class_type[0].data.c_str(),
                             class_estimates[ix].confidence[0], entropies[ix]);
                }
            }
        }

        ROS_INFO("active_exploration_utils::visualize_segmentation : finished");
        return true;
    }

    bool visualize_background(const PointCloud<PointT> &cloud, const vector<vector<int> > &segments, const vector<int> &planes,
                              const vector<int> &invalids, ros::ServiceClient &viz_client, const float &ground_height)
    {
        ROS_INFO("active_exploration_utils::visualize_background : starting");
        vector<int> non_segments;
        // Visualize planes
        printf(ANSI_COLOR_GREEN  "PLANES \n"  ANSI_COLOR_RESET);
        cin.ignore();
        squirrel_object_perception_msgs::SegmentVisualizationOnce viz_srv;
        for (vector<int>::size_type i = 0; i < planes.size(); ++i)
        {
            non_segments.push_back (planes[i]);
            std_msgs::Int32MultiArray seg;
            seg.data = segments[planes[i]];
            vector<std_msgs::Int32MultiArray> cluster_indices;
            cluster_indices.push_back (seg);
            viz_srv.request.clusters_indices = cluster_indices;
            if (viz_client.call(viz_srv))
            {
                // Print the size of the segment
                cout << "(segment " << planes[i] << ")" << endl;
                ROS_INFO("plane %lu : %lu", i, segments[planes[i]].size());
                Eigen::Vector4f centroid;
                compute3DCentroid (cloud, segments[planes[i]], centroid);
                cout << "distance from sensor " << distance3D(0,centroid[0],0,centroid[1],0,centroid[2]) << endl;
                PointCloud<PointT> temp;
                copyPointCloud(cloud, segments[planes[i]], temp);
                PointT min_pt, max_pt;
                getMinMax3D (temp, min_pt, max_pt);
                if (ground_height != numeric_limits<float>::infinity())
                    //cout << "height from ground " << max_pt.data[2] - ground_height << endl;
                cout << "dimensions " << fabs(max_pt.data[0] - min_pt.data[0]) << " "
                                      << fabs(max_pt.data[1] - min_pt.data[1]) << " "
                                      << fabs(max_pt.data[2] - min_pt.data[2]) << endl;
                // Wait for input
                cin.ignore();
            }
            else
            {
                ROS_INFO("active_exploration_utils::visualize_background : could not visualize plane %lu", i);
                return false;
            }
        }
        // Visualize invalids
        printf(ANSI_COLOR_YELLOW  "INVALIDS \n"  ANSI_COLOR_RESET);
        cin.ignore();
        for (vector<int>::size_type i = 0; i < invalids.size(); ++i)
        {
            non_segments.push_back (invalids[i]);
            std_msgs::Int32MultiArray seg;
            seg.data = segments[invalids[i]];
            vector<std_msgs::Int32MultiArray> cluster_indices;
            cluster_indices.push_back (seg);
            viz_srv.request.clusters_indices = cluster_indices;
            if (viz_client.call(viz_srv))
            {
                // Print the size of the segment
                cout << "(segment " << invalids[i] << ")" << endl;
                ROS_INFO("invalid %lu : %lu", i, segments[invalids[i]].size());
                Eigen::Vector4f centroid;
                compute3DCentroid (cloud, segments[invalids[i]], centroid);
                cout << "distance from sensor " << distance3D(0,centroid[0],0,centroid[1],0,centroid[2]) << endl;
                PointCloud<PointT> temp;
                copyPointCloud(cloud, segments[invalids[i]], temp);
                PointT min_pt, max_pt;
                getMinMax3D (temp, min_pt, max_pt);
                if (ground_height != numeric_limits<float>::infinity())
                    cout << "height from ground " << max_pt.data[2] - ground_height << endl;
                copyPointCloud(cloud, segments[invalids[i]], temp);
                getMinMax3D (temp, min_pt, max_pt);
                cout << "dimensions " << fabs(max_pt.data[0] - min_pt.data[0]) << " "
                                      << fabs(max_pt.data[1] - min_pt.data[1]) << " "
                                      << fabs(max_pt.data[2] - min_pt.data[2]) << endl;
                // Wait for input
                cin.ignore();
            }
            else
            {
                ROS_INFO("active_exploration_utils::visualize_background : could not visualize invalid %lu", i);
                return false;
            }
        }
        // Visualize normal segments
        sort(non_segments.begin(), non_segments.end());
        non_segments.erase(unique(non_segments.begin(), non_segments.end()), non_segments.end());
        printf(ANSI_COLOR_CYAN  "SEGMENTS"  ANSI_COLOR_RESET);
        cin.ignore();
        for (vector<std_msgs::Int32MultiArray>::size_type i = 0; i < segments.size(); ++i)
        {
            if (find(non_segments.begin(), non_segments.end(), i) == non_segments.end())
            {
                std_msgs::Int32MultiArray seg;
                seg.data = segments[i];
                vector<std_msgs::Int32MultiArray> cluster_indices;
                cluster_indices.push_back (seg);
                viz_srv.request.clusters_indices = cluster_indices;
                if (viz_client.call(viz_srv))
                {
                    // Print the size of the segment
                    ROS_INFO("segment %lu : %lu", i, segments[i].size());
                    Eigen::Vector4f centroid;
                    compute3DCentroid (cloud, segments[i], centroid);
                    cout << "distance from sensor " << distance3D(0,centroid[0],0,centroid[1],0,centroid[2]) << endl;
                    PointCloud<PointT> temp;
                    PointT min_pt, max_pt;
                    copyPointCloud(cloud, segments[i], temp);
                    getMinMax3D (temp, min_pt, max_pt);
    //                if (ground_height != numeric_limits<float>::infinity())
    //                    cout << "height from ground " << max_pt.data[2] - ground_height << endl;
                    cout << "dimensions " << fabs(max_pt.data[0] - min_pt.data[0]) << " "
                                          << fabs(max_pt.data[1] - min_pt.data[1]) << " "
                                          << fabs(max_pt.data[2] - min_pt.data[2]) << endl;
                    // Wait for input
                    cin.ignore();
                }
                else
                {
                    ROS_INFO("active_exploration_utils::visualize_background : could not visualize segment %lu", i);
                    return false;
                }
            }
            else
            {
                ROS_INFO("active_exploration_utils::visualize_background : segment %lu is not a valid segment", i);
            }
        }

        ROS_INFO("active_exploration_utils::visualize_background : finished");
        return true;
    }

    bool visualize_planning_world(const PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const vector<vector<int> > &segments,
                                  const vector<vector<InstLookUp> > &instance_directories, const vector<vector<InstToMapTF> > &instance_to_map_tfs,
                                  const vector<vector<EntMap> > &emaps, const Eigen::Vector4f &best_location,
                                  const vector<Eigen::Vector4f> &map_locations, const SIM_TYPE &sim)
    {
        ROS_INFO("active_exploration_utils::visualize_planning_world : starting");

        // Start the visualization
        ROS_INFO("active_exploration_utils::visualize_planning_world : viewing the planning world");
        visualization::PCLVisualizer *viewer  = new visualization::PCLVisualizer("Scene");
        viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

        enum UTIL_TYPE {AREA = 0, ENTROPY = 1, PROBABILITY = 2};
        UTIL_TYPE u_type = AREA;
        if (sim == MIN_CLASS_ENTROPY || sim == MIN_CLASS_ENTROPY_UNOCCLUDED || sim == NEAREST_MIN_ENTROPY)
            u_type = ENTROPY;
        else if (sim == MAX_CLASS_PROB || sim == MAX_CLASS_PROB_UNOCCLUDED || sim == NEAREST_MAX_CLASS_PROB)
            u_type = PROBABILITY;

        // Get the camera position
        Eigen::Vector4f camera_pose = extract_camera_position (cloud);
        Eigen::Vector4f cam_transformed = transform_eigvec(camera_pose, transform);
        PointCloud<PointT> camera_pose_pt;
        camera_pose_pt.resize(1);
        camera_pose_pt.points[0].x = cam_transformed[0];
        camera_pose_pt.points[0].y = cam_transformed[1];
        camera_pose_pt.points[0].z = cam_transformed[2];
        viewer->addSphere (camera_pose_pt[0], 0.06, 1.0, 1.0, 1.0, "camera position");  // White

        // Add the map locations to the viewer
        string f;
        PointCloud<PointT> location_cloud;
        location_cloud.resize(1);
        for (vector<Eigen::Vector4f>::size_type i = 0; i < map_locations.size(); ++i)
        {
            f = "map position " + boost::lexical_cast<string>(i);
            location_cloud.points[0].x = map_locations[i][0];
            location_cloud.points[0].y = map_locations[i][1];
            location_cloud.points[0].z = map_locations[i][2];
            viewer->addSphere (location_cloud.points[0], 0.04, 0.5, 0.5, 0.5, f);  // Grey
        }

        // Show the next best location
        PointCloud<PointT> best_location_pt;
        best_location_pt.resize(1);
        best_location_pt.points[0].x = best_location[0];
        best_location_pt.points[0].y = best_location[1];
        best_location_pt.points[0].z = best_location[2];
        viewer->addSphere (best_location_pt[0], 0.06, 1.0, 0.3, 0.6, "next best view");  // Pink

        // Extract and transform the point clouds to visualize
        vector<vector<PointCloud<PointT> > > instance_clouds;
        instance_clouds.resize(segments.size());
        vector<vector<PointCloud<PointT> > > view_clouds;
        view_clouds.resize(segments.size());
        vector<vector<PointCloud<PointT> > > camera_clouds;
        camera_clouds.resize(segments.size());
        vector<vector<pair<int,int> > > view_indices;
        view_indices.resize(segments.size());
        for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            instance_clouds[i].resize(emaps[i].size());
            view_clouds[i].resize(emaps[i].size());
            camera_clouds[i].resize(emaps[i].size());
            view_indices[i].resize(emaps[i].size());
            for (vector<EntMap>::size_type j = 0; j < emaps[i].size(); ++j)
            {
                PointCloud<PointT> i_total;
                PointCloud<PointT> i_view;
                PointCloud<PointT> c_view;
                // Instance total cloud and the most likely view that matches
                int view_ix = atoi(instance_directories[i][j]._ix.c_str()); // most likely viewpoint
                int vec_ix = 0; // vector index
                vector<int>::const_iterator it = find(emaps[i][j]._ixs.begin(), emaps[i][j]._ixs.end(), view_ix);
                if (it != emaps[i][j]._ixs.end())
                    vec_ix = distance<vector<int>::const_iterator>(emaps[i][j]._ixs.begin(), it);
                else
                    ROS_WARN("active_exploration_utils::visualize_planning_world : cannot find view index %u in entropy map %lu (%s/%s)",
                             view_ix, i, instance_directories[i][j]._class_type.c_str(), instance_directories[i][j]._instance_name.c_str());

                // Transform the model cloud to map frame
                // itfs[i][vec_ix] gives the transform for the instance to the model frame
                // A) Bring the model to the instance frame using emaps[i][vec_ix].inverse()
                // B) Bring the model from the instance frame to the map frame using itfs[i][vec_ix]
                // Matrix multiplication works by:
                //     X*Y*v = X*(Y*v) --> from right to left --> last transform is applied first
                // Therefore want T = B*A

                //Eigen::Matrix4f i_transform = itfs[i][0] * _transforms_to_instances[i][0].inverse();  // CHANGED
                Eigen::Matrix4f i_transform = instance_to_map_tfs[i][j]._transform * emaps[i][j]._transforms[vec_ix].inverse();

                transformPointCloud(emaps[i][j]._instance_cloud, i_total, i_transform);
                // Transform the view points to the map frame
                transformPointCloud(emaps[i][j]._clouds[vec_ix], i_view, instance_to_map_tfs[i][j]._transform);

                // Camera views for this instance
                PointCloud<PointT> c_views;
                c_views.resize(emaps[i][j]._camera_poses.size());
                for (size_t k = 0; k < emaps[i][j]._camera_poses.size(); ++k)
                {
                    PointCloud<PointT> pt;
                    pt.resize(1);
                    pt.points[0].x = emaps[i][j]._camera_poses[k][0];
                    pt.points[0].y = emaps[i][j]._camera_poses[k][1];
                    pt.points[0].z = emaps[i][j]._camera_poses[k][2];
                    // Transform to model frame
                    transformPointCloud(pt, pt, emaps[i][j]._transforms[k]);
                    // Add to list
                    c_views[k] = pt.points[0];
                }
                transformPointCloud(c_view, c_view, i_transform);

                // Get the bext instance
                int best_ix = -1;
                if (u_type == ENTROPY)
                    best_ix = min_element(emaps[i][j]._class_entropies.begin(),
                                          emaps[i][j]._class_entropies.end()) -
                                          emaps[i][j]._class_entropies.begin();
                else if (u_type == PROBABILITY)
                    best_ix = min_element(emaps[i][j]._recognition_probabilities.begin(),
                                          emaps[i][j]._recognition_probabilities.end()) -
                                          emaps[i][j]._recognition_probabilities.begin();
                else
                    best_ix = max_element(emaps[i][j]._surface_areas.begin(),
                                          emaps[i][j]._surface_areas.end()) -
                                          emaps[i][j]._surface_areas.begin();

                // Add to the vectors
                instance_clouds[i][j] = i_total;
                view_clouds[i][j] = i_view;
                camera_clouds[i][j] = c_view;
                view_indices[i][j] = make_pair(vec_ix, best_ix);
            }
        }

        // Threads to simultaneously 1) keep viewer open and 2) display clouds and wait for next input to update the clouds
        boost::thread view_thread = boost::thread (&active_exploration_utils::run_viewer, viewer);
        // View each segment with its overlapping recognised instance and potenttial views
        PointCloud<PointT> transformed_cloud;
        transformPointCloud(cloud, transformed_cloud, transform);
        boost::thread cloud_update_thread = boost::thread(&active_exploration_utils::run_view_segment, viewer, transformed_cloud, segments,
                                                          instance_clouds, view_clouds, camera_clouds, view_indices);
        // Join the threads
        view_thread.join();
        cloud_update_thread.join();
        // Finished, wait for thread to exit
        ROS_INFO("active_exploration_utils::visualize_planning_world : finished viewing all point clouds");
        // Close the window
        if (viewer)
            delete viewer;

        ROS_INFO("active_exploration_utils::visualize_planning_world : finished");
        return true;
    }

    bool visualize_in_model_frame(const PointCloud<PointT> &cloud, const Eigen::Matrix4f &transform, const Eigen::Vector4f &location,
                                  const EntMap &emap, const Eigen::Matrix4f &instance_to_map_tf, const Eigen::Matrix4f &instance_to_model_tf)
    {
        // Visualize this location in the entropy map
        visualization::PCLVisualizer viewer("Point clouds");
        int v1 = 0;
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.setBackgroundColor(0.05, 0.05, 0.05, v1); // Setting background to a dark grey
        int v2 = 1;
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(0.05, 0.05, 0.05, v2); // Setting background to a dark grey

        PointCloud<PointT> transformed_cloud;
        transformPointCloud(cloud, transformed_cloud, transform);
        PointCloud<PointT>::Ptr transformed_cloud_ptr (new PointCloud<PointT>(transformed_cloud));
        visualization::PointCloudColorHandlerCustom<PointT> cloud_handler (transformed_cloud_ptr, 255, 255, 255);  // White
        viewer.addPointCloud (transformed_cloud_ptr, cloud_handler, "transformed_cloud", v1);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud", v1);

        Eigen::Vector4f camera_pose = extract_camera_position (cloud);
        PointCloud<PointT>::Ptr camera_pose_pt (new PointCloud<PointT>());
        camera_pose_pt->resize(1);
        camera_pose_pt->points[0].x = camera_pose[0];
        camera_pose_pt->points[0].y = camera_pose[1];
        camera_pose_pt->points[0].z = camera_pose[2];
        transformPointCloud(*camera_pose_pt, *camera_pose_pt, transform);
        visualization::PointCloudColorHandlerCustom<PointT> camera_pt_handler (camera_pose_pt, 255, 0, 0);  // Red
        viewer.addPointCloud (camera_pose_pt, camera_pt_handler, "camera_pt", v1);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "camera_pt", v1);

        PointCloud<PointT>::Ptr location_cloud (new PointCloud<PointT>());
        location_cloud->resize(1);
        location_cloud->points[0].x = location[0];
        location_cloud->points[0].y = location[1];
        location_cloud->points[0].z = location[2];
        visualization::PointCloudColorHandlerCustom<PointT> location_pt_handler (location_cloud, 0, 255, 0);  // Green
        viewer.addPointCloud (location_cloud, location_pt_handler, "location_pt", v1);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "location_pt", v1);

        Eigen::Matrix4f i_transform = instance_to_map_tf * instance_to_model_tf.inverse();  // CHANGED
        PointCloud<PointT>::Ptr model_cloud (new PointCloud<PointT>());
        transformPointCloud(emap._instance_cloud, *model_cloud, i_transform);
        visualization::PointCloudColorHandlerCustom<PointT> model_handler (model_cloud, 255, 0, 0);  // Red
        viewer.addPointCloud (model_cloud, model_handler, "model_pt", v1);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model_pt", v1);

        PointCloud<PointT>::Ptr view_pts_cloud (new PointCloud<PointT>());
        view_pts_cloud->resize(emap._camera_poses.size());
        for (int c = 0; c < emap._camera_poses.size(); ++c)
        {
            PointCloud<PointT> v_pt;
            v_pt.resize(1);
            v_pt.points[0].x = emap._camera_poses[c][0];
            v_pt.points[0].y = emap._camera_poses[c][1];
            v_pt.points[0].z = emap._camera_poses[c][2];
            transformPointCloud(v_pt, v_pt, emap._transforms[c]);
            view_pts_cloud->points[c] = v_pt.points[0];
        }
        transformPointCloud(*view_pts_cloud, *view_pts_cloud, i_transform);
        visualization::PointCloudColorHandlerCustom<PointT> views_handler (view_pts_cloud, 0, 0, 255);  // Blue
        viewer.addPointCloud (view_pts_cloud, views_handler, "views_pt", v1);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "views_pt", v1);

        // Visualize the location in the model frame
        PointCloud<PointT>::Ptr model_cloud_orig (new PointCloud<PointT>(emap._instance_cloud));
        visualization::PointCloudColorHandlerCustom<PointT> model_handler_orig (model_cloud_orig, 255, 0, 0);  // Red
        viewer.addPointCloud (model_cloud_orig, model_handler_orig, "model_pt_orig", v2);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model_pt_orig", v2);

        PointCloud<PointT>::Ptr view_pts_cloud_orig (new PointCloud<PointT>());
        view_pts_cloud_orig->resize(emap._camera_poses.size());
        for (int c = 0; c < emap._camera_poses.size(); ++c)
        {
            PointCloud<PointT> v_pt;
            v_pt.resize(1);
            v_pt.points[0].x = emap._camera_poses[c][0];
            v_pt.points[0].y = emap._camera_poses[c][1];
            v_pt.points[0].z = emap._camera_poses[c][2];
            transformPointCloud(v_pt, v_pt, emap._transforms[c]);
            view_pts_cloud_orig->points[c] = v_pt.points[0];
        }
        visualization::PointCloudColorHandlerCustom<PointT> views_handler_orig (view_pts_cloud_orig, 0, 0, 255);  // Blue
        viewer.addPointCloud (view_pts_cloud_orig, views_handler_orig, "views_pt_orig", v2);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "views_pt_orig", v2);

        // Transform the location into the model frame

        //Eigen::Matrix4f i_transform = itfs[seg_ix][k] * _transforms_to_instances[seg_ix][k].inverse();
        // Transform model to map: x_map = (A*B^-1)x
        //                               = A(B^-1x)
        // Transform map to model:     A^-1x_map = B^-1x
        //                          B(A^-1x_map) = x
        //                         (B*A^-1)x_map = x
        // General rule (A*B)^-1 = B^-1*A^-1
        // So        (A*B^-1)^-1 = B*A^-1
        Eigen::Matrix4f m_transform = instance_to_model_tf * instance_to_map_tf.inverse();

        camera_pose_pt->clear();
        camera_pose_pt->resize(1);
        camera_pose_pt->points[0].x = camera_pose[0];
        camera_pose_pt->points[0].y = camera_pose[1];
        camera_pose_pt->points[0].z = camera_pose[2];
        transformPointCloud(*camera_pose_pt, *camera_pose_pt, transform);
        transformPointCloud(*camera_pose_pt, *camera_pose_pt, m_transform);
        visualization::PointCloudColorHandlerCustom<PointT> camera_pt_handler_orig (camera_pose_pt, 255, 0, 0);  // Red
        viewer.addPointCloud (camera_pose_pt, camera_pt_handler_orig, "camera_pt_orig", v2);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "camera_pt_orig", v2);

        location_cloud->clear();
        location_cloud->resize(1);
        location_cloud->points[0].x = location[0];
        location_cloud->points[0].y = location[1];
        location_cloud->points[0].z = location[2];
        transformPointCloud(*location_cloud, *location_cloud, m_transform);
        visualization::PointCloudColorHandlerCustom<PointT> location_pt_handler_orig (location_cloud, 0, 255, 0);  // Green
        viewer.addPointCloud (location_cloud, location_pt_handler_orig, "location_pt_orig", v2);
        viewer.setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "location_pt_orig", v2);

        while (!viewer.wasStopped())
            viewer.spinOnce();

        return true;
    }

    bool visualize_segment_overlap(const OcTree &tree, const vector<vector<int> > &overlaps,
                                   const map<OcTreeKey,vector<int>,compare_octree_key_struct> &map_key_to_points,
                                   const vector<OcTreeKey> &current_ground_keys,
                                   const PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments,
                                   const vector<Pose> &poses, const vector<vector<OcTreeKey> > &octree_keys,
                                   const PointCloud<PointT> &previous_cloud, const vector<vector<int> > &previous_segments,
                                   const vector<Pose> &previous_poses, const vector<vector<OcTreeKey> > &previous_octree_keys)
    {
        ROS_INFO("active_exploration_utils::visualize_segment_overlap : starting");

        // Print out the associations
        ROS_INFO("Associations :");
        for (vector<vector<int> >::size_type i = 0; i < overlaps.size(); ++i)
        {
            string f = boost::lexical_cast<string>(i);
            if (overlaps[i].size() > 0)
            {
                f += " [";
                for (vector<int>::size_type j = 0; j < overlaps[i].size(); ++j)
                    f += boost::lexical_cast<string>(overlaps[i][j]) + ",";
                f = f.substr(0, f.size()-1);  // Remove the last comma
                f += "]";
            }
            ROS_INFO("   %s", f.c_str());
        }

        // Start the visualization
        visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer ("Overlaps");
        viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        // Show the transformed point clouds
        PointCloud<PointT>::Ptr cloud1 (new PointCloud<PointT>(transformed_cloud));
        visualization::PointCloudColorHandlerCustom<PointT> cloud1_handler (cloud1, 230, 230, 20);  // Yellow
        viewer->addPointCloud (cloud1, cloud1_handler, "cloud1");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
        PointCloud<PointT>::Ptr cloud2 (new PointCloud<PointT>(previous_cloud));
        visualization::PointCloudColorHandlerCustom<PointT> cloud2_handler (cloud2, 20, 230, 20);  // Green
        viewer->addPointCloud (cloud2, cloud2_handler, "cloud2");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");

        // Show each segment, its bounding box and the corresponding overlap with the segment in the other cloud
        PointCloud<PointT>::Ptr first_centroids (new PointCloud<PointT>());
        first_centroids->resize(segments.size());
        string f;
        PointCloud<PointT>::Ptr seg (new PointCloud<PointT>());
        for (vector<vector<int> >::size_type i = 0; i < segments.size(); ++i)
        {
            // Show the segment
            if (segments[i].size() > 0)
            {
                copyPointCloud(transformed_cloud, segments[i], *seg);
                visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 230, 20, 20);  // Red
                f = "segment" + boost::lexical_cast<string>(i);
                viewer->addPointCloud (seg, seg_handler, f);
                viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
            }
            else
            {
                // Get the points that are located in the octomap
                vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(tree, octree_keys[i], map_key_to_points,
                                                                                             current_ground_keys);
                // If there are points found
                if (unknown_seg_ix.size() > 0)
                {
                    copyPointCloud(transformed_cloud, unknown_seg_ix, *seg);
                    visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 200, 100, 100);  // Pink
                    f = "segment" + boost::lexical_cast<string>(i);
                    viewer->addPointCloud (seg, seg_handler, f);
                    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
                }
            }
            // Show the bounding box
            // CUBE = (xmin, xmax, ymin, ymax, zmin, zmax, r, g, b, id, viewport) // rgb = 0-1
            Eigen::Vector4f bb_min = poses[i].get_bb_min();
            Eigen::Vector4f bb_max = poses[i].get_bb_max();
            Eigen::Vector4f centroid = poses[i].get_centroid();
            f = "cube" + boost::lexical_cast<string>(i);
            viewer->addCube(bb_min[0], bb_max[0], bb_min[1], bb_max[1], bb_min[2], bb_max[2], 1, 0, 1, f); // Magenta

            first_centroids->points[i].x = centroid[0];
            first_centroids->points[i].y = centroid[1];
            first_centroids->points[i].z = centroid[2];

    //        if (i < _segment_octree_keys.size())
    //            ROS_INFO("Current segment %lu has %lu voxels", i, _segment_octree_keys[i].size());
    //        else
    //            ROS_WARN("Current segment %lu has invalid octree keys vector entry", i);
        }
        visualization::PointCloudColorHandlerCustom<PointT> first_centroid_handler (first_centroids, 230, 20, 230);  // Megenta
        f = "centroids";
        viewer->addPointCloud (first_centroids, first_centroid_handler, f);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 12, f);

        PointCloud<PointT>::Ptr second_centroids (new PointCloud<PointT>());
        second_centroids->resize(previous_segments.size());
        for (vector<vector<int> >::size_type i = 0; i < previous_segments.size(); ++i)
        {
            // Show the segments
            if (previous_segments[i].size() > 0)
            {
                copyPointCloud(previous_cloud, previous_segments[i], *seg);
                visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 20, 20, 230);  // Blue
                f = "psegment" + boost::lexical_cast<string>(i);
                viewer->addPointCloud (seg, seg_handler, f);
                viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
            }
            else
            {
                // Get the points that are located in the octomap
                vector<int> unknown_seg_ix = active_exploration_utils::keys_to_point_indices(tree, previous_octree_keys[i], map_key_to_points,
                                                                                             current_ground_keys);
                // If there are points found
                if (unknown_seg_ix.size() > 0)
                {
                    copyPointCloud(transformed_cloud, unknown_seg_ix, *seg);
                    visualization::PointCloudColorHandlerCustom<PointT> seg_handler (seg, 102, 178, 255);  // Light blue
                    f = "psegment" + boost::lexical_cast<string>(i);
                    viewer->addPointCloud (seg, seg_handler, f);
                    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, f);
                }
            }
            // Show the bounding box
            // CUBE = (xmin, xmax, ymin, ymax, zmin, zmax, r, g, b, id, viewport) // rgb = 0-1
            Eigen::Vector4f bb_min = previous_poses[i].get_bb_min();
            Eigen::Vector4f bb_max = previous_poses[i].get_bb_max();
            Eigen::Vector4f centroid = previous_poses[i].get_centroid();
            f = "pcube" + boost::lexical_cast<string>(i);
            viewer->addCube(bb_min[0], bb_max[0], bb_min[1], bb_max[1], bb_min[2], bb_max[2], 0, 1, 1, f); // Cyan

            second_centroids->points[i].x = centroid[0];
            second_centroids->points[i].y = centroid[1];
            second_centroids->points[i].z = centroid[2];
        }
        visualization::PointCloudColorHandlerCustom<PointT> second_centroid_handler (second_centroids, 20, 230, 230);  // Cyan
        f = "pcentroids";
        viewer->addPointCloud (second_centroids, second_centroid_handler, f);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 12, f);

        // Visualize the lines connecting the segments and visualize the new bounding boxes
        PointCloud<PointT>::Ptr merged_centroids (new PointCloud<PointT>());
        merged_centroids->resize(segments.size());
        for (vector<vector<int> >::size_type i = 0; i < overlaps.size(); ++i)
        {
            Eigen::Vector4f first_centroid = poses[i].get_centroid();
            Eigen::Vector4f first_bb_min = poses[i].get_bb_min();
            Eigen::Vector4f first_bb_max = poses[i].get_bb_max();
            // Draw a line from this point to every corresponding point
            for (vector<int>::size_type j = 0; j < overlaps[i].size(); ++j)
            {
                Eigen::Vector4f second_centroid = previous_poses[overlaps[i][j]].get_centroid();
                Eigen::Vector4f second_bb_min = previous_poses[overlaps[i][j]].get_bb_min();
                Eigen::Vector4f second_bb_max = previous_poses[overlaps[i][j]].get_bb_max();
                f = "line" + boost::lexical_cast<string>(i) + boost::lexical_cast<string>(j);
                //addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
                viewer->addLine(first_centroids->points[i], second_centroids->points[overlaps[i][j]], 1, 1, 1, f);
                // Sum the centroid
                for (size_t k = 0; k < 4; ++k)
                {
                    first_centroid[k] += second_centroid[k];
                    first_bb_min[k] = min(first_bb_min[k], second_bb_min[k]);
                    first_bb_max[k] = max(first_bb_max[k], second_bb_max[k]);
                }
            }
            // If there were correspondences
            if (overlaps[i].size() > 0)
            {
                for (size_t k = 0; k < 4; ++k)
                    first_centroid[k] = first_centroid[k] / (float) (overlaps[i].size()+1);
            }

            merged_centroids->points[i].x = first_centroid[0];
            merged_centroids->points[i].y = first_centroid[1];
            merged_centroids->points[i].z = first_centroid[2];
            // Show new bounding box
            f = "mcube" + boost::lexical_cast<string>(i);
            viewer->addCube(first_bb_min[0], first_bb_max[0],
                            first_bb_min[1], first_bb_max[1],
                            first_bb_min[2], first_bb_max[2],
                            1, 1, 1, f); // White
        }
        visualization::PointCloudColorHandlerCustom<PointT> merged_centroid_handler (merged_centroids, 255, 255, 255);  // White
        f = "mcentroids";
        viewer->addPointCloud (merged_centroids, merged_centroid_handler, f);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 12, f);

        // Display the visualiser until 'q' key is pressed
        while (!viewer->wasStopped ())
            viewer->spinOnce ();

        if (viewer)
            delete viewer;
    }

    bool visualize_expected_clouds(const PointCloud<PointT> &transformed_cloud, const Eigen::Vector4f &location,
                                   const vector<vector<int> > &segments, const vector<int> &seg_indices,
                                   const vector<vector<InstLookUp> > &instance_directories, const vector<vector<EntMap> > &emaps,
                                   const vector<vector<InstToMapTF> > &instance_to_map_tfs,
                                   const vector<vector<PointCloud<PointT> > > &expected_clouds, const vector<vector<vector<int> > > &visible_indices)
    {
        // Start the visualization
        visualization::PCLVisualizer* viewer = new visualization::PCLVisualizer ("Visible Points");

        // Create the instance clouds
        vector<vector<PointCloud<PointT> > > instance_clouds;
        instance_clouds.resize(seg_indices.size());
        for (vector<int>::size_type i = 0; i < seg_indices.size(); ++i)
        {
            // Get the segments, instances and views of instances
            int seg_ix = seg_indices[i];
            instance_clouds[i].resize(emaps[seg_ix].size());

            for (vector<EntMap>::size_type j = 0; j < emaps[seg_ix].size(); ++j)
            {
                PointCloud<PointT> i_cloud;
                // Instance total cloud
                int view_ix = atoi(instance_directories[seg_ix][j]._ix.c_str()); // most likely viewpoint
                int vec_ix = 0; // vector index
                vector<int>::const_iterator it = find(emaps[seg_ix][j]._ixs.begin(), emaps[seg_ix][j]._ixs.end(), view_ix);
                if (it != emaps[seg_ix][0]._ixs.end())
                    vec_ix = distance<vector<int>::const_iterator>(emaps[seg_ix][0]._ixs.begin(), it);
                else
                    ROS_WARN("active_exploration_utils::visualize_expected_clouds : cannot find view index %u in entropy map %u (%s/%s)",
                             view_ix, seg_ix, instance_directories[seg_ix][j]._class_type.c_str(), instance_directories[seg_ix][j]._instance_name.c_str());
                // Transform the model cloud to map frame
                // itfs[i][vec_ix] gives the transform for the instance to the model frame
                // A) Bring the model to the instance frame using emaps[i][vec_ix].inverse()
                // B) Bring the model from the instance frame to the map frame using itfs[i][vec_ix]
                // Matrix multiplication works by:
                //     X*Y*v = X*(Y*v) --> from right to left --> last transform is applied first
                // Therefore want T = B*A
                Eigen::Matrix4f i_transform = instance_to_map_tfs[seg_ix][j]._transform * emaps[seg_ix][j]._transforms[vec_ix].inverse();
                transformPointCloud(emaps[seg_ix][j]._instance_cloud, i_cloud, i_transform);
                // Add to the vector
                instance_clouds[i][j] = i_cloud;
            }
        }

        // Threads to simultaneously 1) keep viewer open and 2) display clouds and wait for next input to update the clouds
        boost::thread view_thread = boost::thread (&active_exploration_utils::run_viewer, viewer);
        // View each segment with its overlapping recognised instance and potenttial views
        boost::thread cloud_update_thread = boost::thread(&active_exploration_utils::run_view_expected_clouds, viewer, transformed_cloud, location,
                                                          segments, seg_indices, instance_clouds, expected_clouds, visible_indices);
        // Join the threads
        view_thread.join();
        cloud_update_thread.join();
        // Finished, wait for thread to exit
        ROS_INFO("active_exploration_utils::visualize_expected_clouds : finished viewing all point clouds");
        // Close the window
        if (viewer)
            delete viewer;

        ROS_INFO("active_exploration_utils::visualize_expected_clouds : finished");
        return true;
    }

    void run_view_segment(visualization::PCLVisualizer *viewer, const PointCloud<PointT> &transformed_cloud, const vector<vector<int> > &segments,
                          const vector<vector<PointCloud<PointT> > > &instance_clouds, const vector<vector<PointCloud<PointT> > > &view_clouds,
                          const vector<vector<PointCloud<PointT> > > &camera_clouds, const vector<vector<pair<int,int> > > &view_indices)
    {
        ROS_INFO("active_exploration_utils::run_view_segment : starting");

        // Add the current cloud
        PointCloud<PointT>::Ptr scene_cloud (new PointCloud<PointT>(transformed_cloud));
        visualization::PointCloudColorHandlerCustom<PointT> scene_pts_handler (scene_cloud, 255, 255, 255);  // White
        viewer->addPointCloud (scene_cloud, scene_pts_handler, "scene_cloud");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_cloud");

        // Display the segment point clouds one at a time
        string s_name, i_total_name, i_view_name, c_name, m_name, e_name;
        char input;
        PointCloud<PointT>::Ptr seg (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr i_total (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr i_view (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr c_view (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr m_view (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr e_view (new PointCloud<PointT>());
        for (size_t i = 0; i < segments.size(); ++i)
        {
            // Next input
            cout << "Enter for next view or q to quit ...";
            input = cin.get();
            if (input == 'q' || input == 'Q')
            {
                ROS_WARN("active_exploration_utils::run_view_segment : exiting loop after %lu/%lu viewpoints", i+1, segments.size());
                break;
            }

            // Get the segments, instances and views of instances

            // Segment
            seg->clear();
            s_name = "segment_cloud_" + boost::lexical_cast<string>(i);
            copyPointCloud(*scene_cloud, segments[i], *seg);

            // Instance total cloud and the most likely view that matches
            i_total->clear();
            i_view->clear();
            i_total_name = "instance_cloud_" + boost::lexical_cast<string>(i);
            i_view_name = "view_cloud_" + boost::lexical_cast<string>(i);
            copyPointCloud(instance_clouds[i][0], *i_total);
            copyPointCloud(view_clouds[i][0], *i_view);

            // Camera views for this instance
            c_view->clear();
            c_name = "camera_positions_" + boost::lexical_cast<string>(i);
            copyPointCloud(camera_clouds[i][0], *c_view);

            // The camera position for this particular view
            m_view->clear();
            m_name = "model_position_" + boost::lexical_cast<string>(i);
            m_view->push_back (c_view->points[view_indices[i][0].first]);

            // The location with the largest visible surface area or the lowest entropy
            int best_ix = view_indices[i][0].second;
            if (best_ix > 0)
            {
                e_view->clear();
                e_name = "best_entropy_position_" + boost::lexical_cast<string>(i);
                e_view->push_back (c_view->points[best_ix]);
            }

            // Visualize
            // Segment
            visualization::PointCloudColorHandlerCustom<PointT> segment_handler (seg, 230, 20, 20); // Red
            viewer->addPointCloud (seg, segment_handler, s_name);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, s_name);
            printf(ANSI_COLOR_RED  "SEGMENT"  ANSI_COLOR_RESET);
            cin.ignore();
            // View cloud
            visualization::PointCloudColorHandlerCustom<PointT> instance_view_handler (i_view, 20, 230, 20); // Green
            viewer->addPointCloud (i_view, instance_view_handler, i_view_name);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, i_view_name);
            // Camera position for model
            visualization::PointCloudColorHandlerCustom<PointT> model_position_handler (m_view, 20, 230, 20); // Green
            viewer->addPointCloud (m_view, model_position_handler, m_name);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, m_name);
            printf(ANSI_COLOR_GREEN  "INSTANCE VIEW POINT"  ANSI_COLOR_RESET);
            cin.ignore();
            // Instance cloud
            visualization::PointCloudColorHandlerCustom<PointT> instance_handler (i_total, 230, 230, 20); // Yellow
            viewer->addPointCloud (i_total, instance_handler, i_total_name);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, i_total_name);
            printf(ANSI_COLOR_YELLOW  "FULL INSTANCE CLOUD"  ANSI_COLOR_RESET "\n");
            // Camera positions
            visualization::PointCloudColorHandlerCustom<PointT> camera_positions_handler (c_view, 20, 20, 230); // Blue
            viewer->addPointCloud (c_view, camera_positions_handler, c_name);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 8, c_name);
            printf(ANSI_COLOR_BLUE  "TRAINING VIEWS"  ANSI_COLOR_RESET "\n");
            // Camera position for lowest entropy location
            printf(ANSI_COLOR_MAGENTA  "LOWEST ENTROPY VIEW POINT"  ANSI_COLOR_RESET);
            if (best_ix > 0 && e_view->size() > 0)
            {
                visualization::PointCloudColorHandlerCustom<PointT> entropy_position_handler (e_view, 255, 0, 255); // Magenta
                viewer->addPointCloud (e_view, entropy_position_handler, e_name);
                viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, e_name);
            }
            else
            {
                ROS_WARN("active_exploration_utils::run_view_segment : cannot display best index %i and points %lu", best_ix, e_view->size());
            }
            cin.ignore();
        }

        ROS_INFO("active_exploration_utils::run_view_segment : finished");
    }

    void run_view_expected_clouds(visualization::PCLVisualizer *viewer, const PointCloud<PointT> &transformed_cloud, const Eigen::Vector4f &location,
                                  const vector<vector<int> > segments, const vector<int> &seg_indices,
                                  const vector<vector<PointCloud<PointT> > > &instance_clouds,
                                  const vector<vector<PointCloud<PointT> > > &expected_clouds, const vector<vector<vector<int> > > &visible_indices)
    {
        ROS_INFO("active_exploration_utils::run_view_expected_clouds : starting");

        int v1 = 0;
        viewer->createViewPort(0.0, 0.0, 0.6, 1.0, v1);
        viewer->setBackgroundColor(0.05, 0.05, 0.05, v1); // Setting background to a dark grey
        int v2 = 1;
        viewer->createViewPort(0.6, 0.5, 1.0, 1.0, v2);
        viewer->setBackgroundColor(0.05, 0.05, 0.05, v2); // Setting background to a dark grey
        int v3 = 2;
        viewer->createViewPort(0.6, 0.0, 1.0, 0.5, v3);
        viewer->setBackgroundColor(0.05, 0.05, 0.05, v3); // Setting background to a dark grey

        // Add the current cloud
        PointCloud<PointT>::Ptr scene_cloud (new PointCloud<PointT>(transformed_cloud));
        visualization::PointCloudColorHandlerCustom<PointT> scene_pts_handler (scene_cloud, 255, 255, 255);  // White
        viewer->addPointCloud (scene_cloud, scene_pts_handler, "scene_cloud", v1);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud", v1);

        // Add the map location
        PointCloud<PointT>::Ptr location_cloud (new PointCloud<PointT>());
        location_cloud->resize(1);
        location_cloud->points[0].x = location[0];
        location_cloud->points[0].y = location[1];
        location_cloud->points[0].z = location[2];
        visualization::PointCloudColorHandlerCustom<PointT> location_pts_handler (scene_cloud, 20, 230, 230);  // Cyan
        viewer->addPointCloud (location_cloud, location_pts_handler, "location", v1);
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "location", v1);

        // Display the segment point clouds one at a time with their predicted views
        string s_name, i_total_name, e_name, e_name2, v_name, v_name2;
        char input;
        PointCloud<PointT>::Ptr seg (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr i_total (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr e_view (new PointCloud<PointT>());
        PointCloud<PointT>::Ptr v_view (new PointCloud<PointT>());
        for (vector<int>::size_type i = 0; i < seg_indices.size(); ++i)
        {
            // Next input
            cout << "Enter for next view or q to quit ...";
            input = cin.get();
            if (input == 'q' || input == 'Q')
            {
                ROS_WARN("active_exploration_utils::run_view_expected_clouds : exiting loop after %lu/%lu viewpoints", i+1, segments.size());
                break;
            }

            // Get the segments, instances and views of instances
            int seg_ix = seg_indices[i];

            // Segment
            seg->clear();
            s_name = "segment_cloud_" + boost::lexical_cast<string>(seg_ix);
            copyPointCloud(*scene_cloud, segments[seg_ix], *seg);

            // The instance cloud
            i_total->clear();
            i_total_name = "instance_cloud_" + boost::lexical_cast<string>(seg_ix);
            copyPointCloud(instance_clouds[i][0], *i_total);

            // The expected point cloud
            e_view->clear();
            e_name = "expected_cloud_" + boost::lexical_cast<string>(seg_ix);
            e_name2 = "expected_cloud_" + boost::lexical_cast<string>(seg_ix) + "_2";
            copyPointCloud(expected_clouds[i][0], *e_view);

            // The visible points in the world
            v_view->clear();
            v_name = "visible_cloud_" + boost::lexical_cast<string>(seg_ix);
            v_name2 = "visible_cloud_" + boost::lexical_cast<string>(seg_ix) + "_2";
            copyPointCloud(expected_clouds[i][0], visible_indices[i][0], *v_view);

            // Visualize
            // Segment
            visualization::PointCloudColorHandlerCustom<PointT> segment_handler (seg, 230, 20, 20); // Red
            viewer->addPointCloud (seg, segment_handler, s_name, v1);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, s_name, v1);
            printf(ANSI_COLOR_RED  "SEGMENT"  ANSI_COLOR_RESET);
            cin.ignore();
            // Instance cloud
            visualization::PointCloudColorHandlerCustom<PointT> instance_handler (i_total, 230, 230, 20); // Yellow
            viewer->addPointCloud (i_total, instance_handler, i_total_name, v1);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, i_total_name, v1);
            printf(ANSI_COLOR_YELLOW  "FULL INSTANCE CLOUD (%lu)"  ANSI_COLOR_RESET "\n", i_total->size());
            cin.ignore();
            // Expected cloud
            // View port 1
            visualization::PointCloudColorHandlerCustom<PointT> expected_cloud_handler (e_view, 20, 230, 230); // Cyan
            viewer->addPointCloud (e_view, expected_cloud_handler, e_name, v1);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, e_name, v1);
            // View port 2
            visualization::PointCloudColorHandlerCustom<PointT> expected_cloud_handler2 (e_view, 20, 230, 230); // yan
            viewer->addPointCloud (e_view, expected_cloud_handler2, e_name2, v2);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, e_name2, v2);
            printf(ANSI_COLOR_CYAN "EXPECTED CLOUD (%lu)"  ANSI_COLOR_RESET "\n", e_view->size());
            cin.ignore();
            // Visible cloud
            // View port 1
            visualization::PointCloudColorHandlerCustom<PointT> visible_cloud_handler (v_view, 255, 0, 255); // Magenta
            viewer->addPointCloud (v_view, visible_cloud_handler, v_name, v1);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 3, v_name, v1);
            // View port 3
            visualization::PointCloudColorHandlerCustom<PointT> visible_cloud_handler2 (v_view, 255, 0, 255); // Magenta
            viewer->addPointCloud (v_view, visible_cloud_handler2, v_name2, v3);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, v_name, v3);
            printf(ANSI_COLOR_MAGENTA "VISIBLE CLOUD (%lu)"  ANSI_COLOR_RESET "\n", v_view->size());
            cin.ignore();
        }

        ROS_INFO("active_exploration_utils::run_view_expected_clouds : finished");
    }

    void run_viewer(visualization::PCLVisualizer *viewer)
    {
        ROS_INFO("active_exploration_utils::run_viewer : starting");
        // Display the visualiser until 'q' key is pressed
        while (!viewer->wasStopped ())
            viewer->spinOnce ();
        ROS_INFO("active_exploration_utils::run_viewer : finished");
    }

// End name space
}

/* === IO === */

vector<int> load_entropy_order_file(const string &entropy_file, const string &test_dir, const SIM_TYPE &sim)
{
    vector<int> result;
    // Open the file for reading
    ifstream myfile (entropy_file.c_str());
    if (myfile.is_open())
    {
        // Read the first line and check that it matches the test_dir
        string line;
        if (getline(myfile, line))
        {
            stringstream linestream (line);
            string dir_name;
            linestream >> dir_name;
            if (strcmp(dir_name.c_str(), test_dir.c_str()) != 0)
            {
                ROS_ERROR("active_exploration_utils::load_entropy_order_file : header %s does not match test directory %s",
                          dir_name.c_str(), test_dir.c_str());
                return result;
            }
        }
        else
        {
            ROS_ERROR("active_exploration_utils::load_entropy_order_file : could not read header in entropy file %s", entropy_file.c_str());
            return result;
        }
        // Load the indices, entropy values and class probabilities
        vector<pair<int,double> > int_ent;
        vector<pair<int,double> > int_pro;
        double prob_sum = 0;
        while (getline(myfile, line))
        {
            stringstream linestream (line);
            int ix;
            double eval;
            double cprob;
            linestream >> ix >> eval >> cprob;
            int_ent.push_back(make_pair(ix,eval));
            int_pro.push_back(make_pair(ix,cprob));
            prob_sum += cprob;
        }
        myfile.close();
        // If all probability values are zero then set this to entropy values
        bool valid_prob_vals = false;
        for (vector<pair<int,double> >::size_type i = 0; i < int_pro.size(); ++i)
        {
            if (int_pro[i].second > 0)
            {
                valid_prob_vals = true;
                break;
            }
        }
        if (!valid_prob_vals)
            int_pro = int_ent;
        // Sort the values
        sort(int_ent.begin(), int_ent.end(), compare<double>);
        sort(int_pro.begin(), int_pro.end(), compare<double>);
        // Extract the indices for the probabilities
        if (valid_prob_vals && (sim == WORST_TO_BEST_PROB || sim == BEST_TO_WORST_PROB))
        {
            // If probabilities have been recorded then use these to sort the order
            if (prob_sum > 0 && int_pro.size() > 0)
            {
                // While the value is not 0
                double p = int_pro.back().second;
                while (p > 0)
                {
                    result.push_back(int_pro.back().first);
                    int_pro.pop_back();
                    if (int_pro.size() > 0)
                        p = int_pro.back().second;
                    else
                        break;
                    // Continue if p > 0
                }
            }
        }
        // If result is not the size of int_ent
        if (result.size() > int_ent.size())
        {
            ROS_ERROR("active_exploration_utils::load_entropy_order_file : result contains %lu elements but file only contains %lu elements",
                      result.size(), int_ent.size());
            result.clear();
            return result;
        }
        // Check if result has not been filled all the way
        if (result.size() < int_ent.size())
        {
            // Need to add the remaining locations in the entropy ordr
            for (vector<pair<int,double> >::size_type i = 0; i < int_ent.size(); ++i)
            {
                // Check if this index has not already been added
                if (find(result.begin(), result.end(), int_ent[i].first) == result.end())
                    result.push_back(int_ent[i].first);
                // Otherwise skip
            }

            return result;
        }
        // Return
        return result;
    }
    else
    {
        ROS_ERROR("active_exploration_utils::load_entropy_order_file : could not open %s", entropy_file.c_str());
        return result;
    }
}

bool load_views_limit_from_file(const string &views_file, const string &test_dir, int &num_views)
{
    // Open the file for reading
    ifstream myfile (views_file.c_str());
    if (myfile.is_open())
    {
        // Read each line
        string check_dir = rem_backslash(test_dir);
        bool success = false;
        string line;
        while (getline(myfile, line))
        {
            stringstream linestream (line);
            string dataset_name;
            int n;
            linestream >> dataset_name >> n;
            if (strcmp(check_dir.c_str(), rem_backslash(dataset_name).c_str()) == 0)
            {
                // Found a match to the data directory
                success = true;
                num_views = n;
                break;
            }
        }
        myfile.close();

        // Return success
        return success;
    }
    else
    {
        ROS_ERROR("active_exploration_utils::load_views_limit_from_file : could not open %s", views_file.c_str());
        return false;
    }
}
