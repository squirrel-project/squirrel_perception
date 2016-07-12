#include "squirrel_active_exploration/io_utils.h"

using namespace std;
using namespace pcl;

bool read_tf_file(const string &filename, Eigen::Matrix4f &tr)
{
    // Open the file
    ifstream myfile (filename.c_str());
    bool valid_file = false;
    if (myfile.is_open())
    {
        valid_file = true;
        int i = 0, j = 0;
        string word;
        while (myfile >> word)
        {
            // Some transform files have a header, so must skip the first word
            try
            {
                float num = boost::lexical_cast<float>(word);
                tr(i,j) = num;
                ++j;
                if (j == 4)
                {
                    ++i;
                    j = 0;
                }
            }
            catch(boost::bad_lexical_cast&)
            {
                printf(ANSI_COLOR_YELLOW  "WARN io_utils::read_tf_file : %s is not a number!"  ANSI_COLOR_RESET "\n", word.c_str());
            }
        }
        myfile.close();
    }
    else
    {
        printf(ANSI_COLOR_RED "ERROR io_utils::read_tf_file : could not open file %s"  ANSI_COLOR_RESET "\n", filename.c_str());
    }
    return valid_file;
}

bool write_to_file(const string &filename, const double &val)
{
    // Print warning if file already exists
    if (boost::filesystem::exists(filename))
        printf(ANSI_COLOR_YELLOW  "WARN io_utils::write_to_file : file %s already exists!"  ANSI_COLOR_RESET "\n", filename.c_str());
    // Write to file
    ofstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        myfile << val;
        myfile.close();
    }
    else
    {
        printf(ANSI_COLOR_RED  "ERROR io_utils::write_to_file : unable to open %s"  ANSI_COLOR_RESET "\n", filename.c_str());
        return false;
    }
}

bool write_to_file(const string &filename, const float &val)
{
    // Print warning if file already exists
    if (boost::filesystem::exists(filename))
        printf(ANSI_COLOR_YELLOW  "WARN io_utils::write_to_file : file %s already exists!"  ANSI_COLOR_RESET "\n", filename.c_str());
    // Write to file
    ofstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        myfile << val;
        myfile.close();
    }
    else
    {
        printf(ANSI_COLOR_RED  "ERROR io_utils::write_to_file : unable to open %s"  ANSI_COLOR_RESET "\n", filename.c_str());
        return false;
    }
}

bool load_test_directory(const string &dir, const bool &invert_transform, vector<Eigen::Vector4f> &poses, vector<PointCloud<PointT> > &clouds,
                         vector<Eigen::Matrix4f> &transforms, vector<int> &ix_order)
{
    //ROS_INFO("io_utils::load_test_directory : loading %s", dir.c_str());
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR("io_utils::load_test_directory : could not open %s", dir.c_str());
        return false;
    }

    poses.clear();
    clouds.clear();
    transforms.clear();
    ix_order.clear();

    vector<pair<int,int> > map_ix_order;

    int counter = 0;
    while ((dirp = readdir(dp)) != NULL)
    {
        bool success = true;
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") == 0 || strcmp(f.c_str(),"..") == 0)
            success = false;

        // Get the dot
        size_t dot = f.find_last_of('.');
        if (success)
        {
            if (dot == string::npos)
            {
                ROS_WARN("io_utils::load_test_directory : could not read extension of file %s", f.c_str());
                success = false;
            }
        }
        // Get the index name
        int ix = -1;
        if (success)
        {
            size_t underscore = f.find_last_of('_');
            if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
            {
                int str_len = dot - underscore;
                string str_ix = f.substr(underscore+1,str_len-1);
                ix = atoi(str_ix.c_str());
            }
            else
            {
                ROS_WARN("io_utils::load_test_directory : could not read the index in file %s", f.c_str());
                success = false;
            }
        }
        // If the extension is .pcd
        string ext = f.substr(dot);
        if (success)
        {
            if (strcmp(ext.c_str(),".pcd") != 0)
                success = false;
        }
        // Check it is a point cloud and not an indices file
        if (strncmp(f.c_str(),_CLOUD_PREFIX, sizeof(_CLOUD_PREFIX)-1) == 0)
            success = true;
        else if (strncmp(f.c_str(),_INDICES_PREFIX, sizeof(_INDICES_PREFIX)-1) == 0)
            success = false;
        else
            success = false;
        // Load the point cloud
        string full_filename = add_backslash(dir) + f;
        PointCloud<PointT> cloud;
        if (success)
        {
            if (io::loadPCDFile<PointT> (full_filename.c_str(), cloud) == -1)
            {
                ROS_WARN("io_utils::load_test_directory : could not load point cloud %s", full_filename.c_str());
                success = false;
            }
        }
        // Transform the point cloud
        if (success)
        {
            PointCloud<PointT> transformed_cloud;
            Eigen::Vector4f position;
            Eigen::Matrix4f transform;
            if (!transform_cloud_from_file(dir, f, cloud, transformed_cloud, position, transform))
            {
                ROS_WARN("io_utils::load_test_directory : error transforming point cloud from file %s", full_filename.c_str());
                success = false;
            }
            else
            {
                // Add to the lists
                poses.push_back(position);
                //clouds.push_back(transformed_cloud);
                clouds.push_back(cloud);
                Eigen::Matrix4f transform_inv = transform;
                if (invert_transform)
                    transform_inv = transform.inverse();
                transform = transform_inv;
                transforms.push_back(transform);
            }
        }
        // Save the index order
        if (success && ix >= 0)
        {
            map_ix_order.push_back (make_pair(counter, ix));
            ++counter;
        }
    }
    // Unwrap the index map into a vector
    sort(map_ix_order.begin(), map_ix_order.end(), compare<int>);
    for (vector<pair<int,int> >::const_iterator it = map_ix_order.begin(), end = map_ix_order.end(); it != end; ++it)
        ix_order.push_back(it->first);

    // Return success if transformed files
    if (poses.size() <= 0 || clouds.size() <= 0 || transforms.size() < 0 || ix_order.size() <= 0)
        return false;
    int num_clouds = clouds.size();
    if (poses.size() != num_clouds || transforms.size() != num_clouds || ix_order.size() != num_clouds)
        return false;

    ROS_INFO("io_utils::load_test_directory : loaded %lu clouds", clouds.size());
    return true;
}

bool load_test_directory(const string &dir, const bool &invert_transform, vector<Eigen::Vector4f> &poses, vector<PointCloud<PointT> > &clouds,
                         vector<Eigen::Matrix4f> &transforms)
{
    vector<int> ix_order;
    return load_test_directory(dir, invert_transform, poses, clouds, transforms, ix_order);
}

bool load_test_directory(const string &dir, const bool &invert_transform,
                         vector<Eigen::Vector4f> &poses, vector<sensor_msgs::PointCloud2> &clouds,
                         vector<Eigen::Matrix4f> &transforms, vector<int> &ix_order)
{
    // Call the function with pcl::PointClouds
    vector<PointCloud<PointT> > pcl_clouds;
    if (!load_test_directory(dir, invert_transform, poses, pcl_clouds, transforms, ix_order))
    {
        ROS_ERROR("io_utils::load_test_directory : could not load test directory %s", dir.c_str());
        return false;
    }
    // Otherwise convert each to sensor_msgs::PointCloud2Ptr
    for (vector<PointCloud<PointT> >::size_type i = 0; i < pcl_clouds.size(); ++i)
    {
        sensor_msgs::PointCloud2 c;
        toROSMsg(pcl_clouds[i], c);
        clouds.push_back (c);
    }
    return true;
}

bool load_test_directory(const string &dir, const bool &invert_transform,
                         vector<Eigen::Vector4f> &poses, vector<sensor_msgs::PointCloud2> &clouds, vector<Eigen::Matrix4f> &transforms)
{
    vector<int> ix_order;
    return load_test_directory(dir, invert_transform, poses, clouds, transforms, ix_order);
}

bool load_test_directory(const string &dir, const bool &invert_transform, vector<pcl_CloudTX> &pose_cloud_tf)
{
    // Get the vectors
    vector<Eigen::Vector4f> poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<int> ix_order;

    if (!load_test_directory(dir, invert_transform, poses, clouds, transforms, ix_order))
    {
        ROS_ERROR("io_utils::load_test_directory : could not load test directory %s", dir.c_str());
        return false;
    }
    // Otherwise, create the pcl_CloudTX structures
    pose_cloud_tf.clear();
    for (vector<Eigen::Vector4f>::size_type i = 0; i < poses.size(); ++i)
    {
        pcl_CloudTX ctx;
        ctx._camera_position = poses[i];
        ctx._cloud = clouds[i];
        ctx._transform = transforms[i];
        ctx._ix = ix_order[i];
        pose_cloud_tf.push_back (ctx);
    }

    return true;
}

bool load_test_directory(const string &dir, const bool &invert_transform, vector<ros_CloudTX> &pose_cloud_tf)
{
    // Get the vectors
    vector<Eigen::Vector4f> poses;
    vector<PointCloud<PointT> > clouds;
    vector<Eigen::Matrix4f> transforms;
    vector<int> ix_order;

    if (!load_test_directory(dir, invert_transform, poses, clouds, transforms, ix_order))
    {
        ROS_ERROR("io_utils::load_test_directory : could not load test directory %s", dir.c_str());
        return false;
    }
    // Otherwise, create the ros_CloudTX structures
    pose_cloud_tf.clear();
    for (vector<Eigen::Vector4f>::size_type i = 0; i < poses.size(); ++i)
    {
        sensor_msgs::PointCloud2 c;
        toROSMsg(clouds[i], c);
        ros_CloudTX ctx;
        ctx._camera_position = poses[i];
        ctx._cloud = c;
        ctx._transform = transforms[i];
        ctx._ix = ix_order[i];
        pose_cloud_tf.push_back (ctx);
        //ctx._ix = i;
        //pose_cloud_tf[poses[i]] = ctx;
    }

    return true;
}

//bool load_test_directory_with_segment_indices(const string &dir, const bool &invert_transform,
//                                              vector<Eigen::Vector4f> &poses, vector<PointCloud<PointT> > &clouds,
//                                              vector<Eigen::Matrix4f> &transforms, vector<vector<vector<int> > > &indices, const int &index)
//{
//    // Load the files
//    DIR *dp;
//    struct dirent *dirp;
//    if ((dp  = opendir(dir.c_str())) == NULL)
//    {
//        ROS_ERROR("io_utils::load_test_directory_with_segment_indices : could not open %s", dir.c_str());
//        return false;
//    }

//    poses.clear();
//    clouds.clear();
//    transforms.clear();
//    indices.clear();

//    map<int,Eigen::Vector4f*> pose_map;
//    map<int,PointCloud<PointT>::Ptr> cloud_map;
//    map<int,Eigen::Matrix4f*> transform_map;
//    map<int,vector<PointIndices>*> ix_map;

//    while ((dirp = readdir(dp)) != NULL)
//    {
//        string f = string(dirp->d_name);
//        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
//        {
//            cout << f << endl;
//            // Get the number
//            size_t underscore = f.find_last_of('_');
//            size_t dot = f.find_last_of('.');
//            if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
//            {
//                int str_len = dot - underscore;
//                string str_ix = f.substr(underscore+1,str_len-1);
//                int ix = atoi(str_ix.c_str());
//                string filename = add_backslash(dir) + f;
//                cout << ix << endl;

//                if (strncmp(f.c_str(),_CLOUD_PREFIX, sizeof(_CLOUD_PREFIX)-1) == 0)
//                {
//                    PointCloud<PointT>::Ptr in_cloud (new PointCloud<PointT>());
//                    if (io::loadPCDFile<PointT> (filename.c_str(), *in_cloud) == -1)
//                        ROS_WARN("io_utils::load_test_directory_with_segment_indices : could not read point cloud file %s", filename.c_str());
//                    else
//                        cloud_map[ix] = in_cloud;
//                    // Get the transform and the pose
//                    PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
//                    Eigen::Vector4f position;
//                    Eigen::Matrix4f transform;
//                    if (!transform_cloud_from_file(dir, f, *in_cloud, *transformed_cloud, position, transform))
//                    {
//                        ROS_WARN("io_utils::load_test_directory_with_segment_indices : error transforming point cloud from file %s",
//                                  filename.c_str());
//                        transform_map[ix] = NULL;
//                        pose_map[ix] = NULL;
//                    }
//                    else
//                    {
//                        // WARNING : these files the transform is defined in the opposite way
//                        // The transform is correct as it brings all clouds into alignment
//                        // Take the inverse because it is reversed in the transform_cloud_from_file function
//                        //Eigen::Matrix4f tri = transform.inverse();  // CHANGE 16 JUNE - removed inverse
//                        Eigen::Matrix4f tri = transform;
//                        if (invert_transform)
//                            tri = tri.inverse();
//                        Eigen::Vector4f *ps_ptr (new Eigen::Vector4f(position));
//                        pose_map[ix] = ps_ptr;
//                        Eigen::Matrix4f *tr_ptr (new Eigen::Matrix4f(tri));
//                        transform_map[ix] = tr_ptr;
//                    }
//                }
//                else if (strncmp(f.c_str(),_INDICES_PREFIX, sizeof(_INDICES_PREFIX)-1) == 0)
//                {
//                    PointCloud<IndexPoint> in_cloud;
//                    if (io::loadPCDFile<IndexPoint>(filename.c_str(), in_cloud) == -1)
//                    {
//                        ROS_WARN("io_utils::load_test_directory_with_segment_indices : could not read index file");
//                    }
//                    else
//                    {
//                        PointIndices indices;
//                        indices.indices.resize(in_cloud.points.size());
//                        for (size_t i = 0; i < in_cloud.points.size(); ++i)
//                            indices.indices[i] = in_cloud.points[i].idx;
//                        if (ix_map[ix] == NULL)
//                            ix_map[ix] = new vector<PointIndices>();
//                        ix_map[ix]->push_back(indices);
//                        cout << "adding segment to index " << ix << endl;
//                    }
//                }
//            }
//        }
//    }
//    closedir(dp);

//    // Extract the points belonging to the object in each view and store to the entropy map
//    vector<int> ix_order;
//    for(map<int,PointCloud<PointT>::Ptr>::iterator it = cloud_map.begin(); it != cloud_map.end(); ++it)
//    {
//        // it->first = key
//        // it->second = value

//        ix_order.push_back(it->first);

//        // Point cloud
//        clouds.push_back(*it->second);
//        // Poe
//        if (pose_map[it->first] != NULL)
//            poses.push_back(*pose_map[it->first]);
//        else
//            poses.push_back(Eigen::Vector4f(0,0,0,0));
//        // Transform
//        if (transform_map[it->first] != NULL)
//            transforms.push_back (*transform_map[it->first]);
//        else
//            transforms.push_back (Eigen::Matrix4f::Identity());
//        // Segmentation indices
//        vector<vector<int> > segments;
//        if (ix_map[it->first] != NULL)
//        {
//            vector<PointIndices> ix = *ix_map[it->first];
//            for (size_t i = 0; i < ix.size(); ++i)
//                segments.push_back(ix[i].indices);
//        }
//        indices.push_back (segments);
//    }

//    // If index order is specified then just return this particular element
//    if (index < 0)
//        return true;
//    if (index >= clouds.size())
//    {
//        ROS_ERROR("io_utils::load_test_directory_with_segment_indices : index %u is too large for vector which has %lu elements", index, clouds.size());
//        return false;
//    }
//    // Otherwise extract only this index
//    vector<Eigen::Vector4f> temp_poses = poses;
//    vector<PointCloud<PointT> > temp_clouds = clouds;
//    vector<Eigen::Matrix4f> temp_transforms = transforms;
//    vector<vector<vector<int> > > temp_indices = indices;
//    poses.clear();
//    clouds.clear();
//    transforms.clear();
//    indices.clear();
//    poses.push_back(temp_poses[index]);
//    clouds.push_back(temp_clouds[index]);
//    transforms.push_back(temp_transforms[index]);
//    indices.push_back(temp_indices[index]);

//    return true;
//}

bool load_test_directory_with_segment_indices(const string &dir, const bool &invert_transform,
                                              vector<Eigen::Vector4f> &poses, vector<PointCloud<PointT> > &clouds,
                                              vector<Eigen::Matrix4f> &transforms, vector<vector<vector<int> > > &indices, const int &index)
{
    // Load the files
    DIR *dp;
    struct dirent *dirp;
    if ((dp  = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR("io_utils::load_test_directory_with_segment_indices : could not open %s", dir.c_str());
        return false;
    }

    poses.clear();
    clouds.clear();
    transforms.clear();
    indices.clear();

    // Get all the clouds
    vector<pair<string,int> > cloud_files;
    vector<pair<int,int> > cloud_ix_map;
    int count = 0;
    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            //cout << f << endl;
            // Get the number
            size_t underscore = f.find_last_of('_');
            size_t dot = f.find_last_of('.');
            if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
            {
                int str_len = dot - underscore;
                string str_ix = f.substr(underscore+1,str_len-1);
                int ix = atoi(str_ix.c_str());

                if (strncmp(f.c_str(),_CLOUD_PREFIX, sizeof(_CLOUD_PREFIX)-1) == 0)
                {
                    // Save the filename
                    cloud_files.push_back(make_pair(f,ix));
                    cloud_ix_map.push_back(make_pair(count,ix));
                    ++count;
                }
            }
        }
    }
    closedir(dp);

    // Sort the filenames from smallest to largest
    sort(cloud_ix_map.begin(), cloud_ix_map.end(), compare<int>);
    vector<pair<string,int> > temp_cloud_files = cloud_files;
    cloud_files.clear();
    for (vector<pair<int,int> >::size_type i = 0; i < cloud_ix_map.size(); ++i)
        cloud_files.push_back(temp_cloud_files[cloud_ix_map[i].first]);

    // Go through each file and find the transform, pose and segment indices
    vector<int> remove_indices;
    vector<int> vector_ix_mapping;
    for (vector<pair<string,int> >::size_type i = 0; i < cloud_files.size(); ++i)
    {
        int ix = cloud_files[i].second;
        string ix_str = boost::lexical_cast<string>(ix);
        while (ix_str.size() < 10)
            ix_str = "0" + ix_str;
        string filename = add_backslash(dir) + cloud_files[i].first;

        // Get the cloud
        vector_ix_mapping.push_back(ix);
        PointCloud<PointT> in_cloud;
        if (io::loadPCDFile<PointT> (filename.c_str(), in_cloud) == -1)
            ROS_WARN("io_utils::load_test_directory_with_segment_indices : could not read point cloud file %s", filename.c_str());
        else
            clouds.push_back(in_cloud);
        // Get the transform and the pose
        PointCloud<PointT> transformed_cloud;
        Eigen::Vector4f position;
        Eigen::Matrix4f transform;
        if (!transform_cloud_from_file(dir, cloud_files[i].first, in_cloud, transformed_cloud, position, transform))
        {
            ROS_WARN("io_utils::load_test_directory_with_segment_indices : error transforming point cloud from file %s",
                      filename.c_str());
            remove_indices.push_back(i);
        }
        // WARNING : these files the transform is defined in the opposite way
        // The transform is correct as it brings all clouds into alignment
        // Take the inverse because it is reversed in the transform_cloud_from_file function
        Eigen::Matrix4f tri = transform;
        if (invert_transform)
            tri = transform.inverse();
        poses.push_back(position);
        transforms.push_back(tri);
        // Get the segment indices
        string segment_indices_str;
        int seg_count = 0;
        vector<vector<int> > segment_indices;
        while (true)
        {
            string count_str = boost::lexical_cast<string>(seg_count);
            while (count_str.size() < 2)
                count_str = "0" + count_str;
            segment_indices_str = add_backslash(dir) + _INDICES_PREFIX + count_str + "_" + ix_str + ".pcd";
            // If valid file then load it
            if (boost::filesystem::exists(segment_indices_str))
            {
                PointCloud<IndexPoint> in_cloud;
                if (io::loadPCDFile<IndexPoint>(segment_indices_str.c_str(), in_cloud) == -1)
                {
                    ROS_WARN("io_utils::load_test_directory_with_segment_indices : could not read index file");
                    break;
                }
                else
                {
                    vector<int> in_indices;
                    in_indices.resize(in_cloud.points.size());
                    for (size_t j = 0; j < in_cloud.points.size(); ++j)
                        in_indices[j] = in_cloud.points[j].idx;
                    segment_indices.push_back(in_indices);
                }
            }
            else
            {
                break;
            }
            ++seg_count;
        }
        indices.push_back(segment_indices);
    }

    // Remove elements that were invalid
    sort(remove_indices.begin(), remove_indices.end());
    reverse(remove_indices.begin(), remove_indices.end());
    for (vector<int>::size_type i = 0; i < remove_indices.size(); ++i)
    {
        clouds.erase(clouds.begin() + remove_indices[i]);
        poses.erase(poses.begin() + remove_indices[i]);
        transforms.erase(transforms.begin() + remove_indices[i]);
        indices.erase(indices.begin() + remove_indices[i]);
        vector_ix_mapping.erase(vector_ix_mapping.begin() + remove_indices[i]);
    }

    // If index order is specified then just return this particular element
    if (index < 0)
        return true;
    if (index >= clouds.size())
    {
        ROS_ERROR("io_utils::load_test_directory_with_segment_indices : index %u is too large for vector which has %lu elements", index, clouds.size());
        return false;
    }
    // Otherwise extract only this index
    vector<Eigen::Vector4f> temp_poses = poses;
    vector<PointCloud<PointT> > temp_clouds = clouds;
    vector<Eigen::Matrix4f> temp_transforms = transforms;
    vector<vector<vector<int> > > temp_indices = indices;
    poses.clear();
    clouds.clear();
    transforms.clear();
    indices.clear();
    int vec_ix = -1;
    vector<int>::const_iterator it = find(vector_ix_mapping.begin(), vector_ix_mapping.end(), index);
    if (it != vector_ix_mapping.end())
        vec_ix = distance<vector<int>::const_iterator>(vector_ix_mapping.begin(), it);
    else
        return false;
    if (vec_ix < 0 || vec_ix >= temp_clouds.size())
    {
        ROS_ERROR("io_utils::load_test_directory_with_segment_indices : invalid vector index %u", vec_ix);
        return false;
    }
    poses.push_back(temp_poses[vec_ix]);
    clouds.push_back(temp_clouds[vec_ix]);
    transforms.push_back(temp_transforms[vec_ix]);
    indices.push_back(temp_indices[vec_ix]);

    return true;
}

bool load_model_training_directory(const string &dir, const bool &invert_transform,
                                   vector<Eigen::Vector4f> &poses, vector<PointCloud<PointT> > &clouds,
                                   vector<Eigen::Matrix4f> &transforms, vector<vector<int> > &indices)
{
    // Read the cloud file, object indices file and pose file

    //ROS_INFO("io_utils::load_model_training_directory : loading file in directory %s ...", dir.c_str());
    DIR *dp;
    struct dirent *dirp;
    if ((dp  = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR("io_utils::load_model_training_directory : could not open %s", dir.c_str());
        return false;
    }

    poses.clear();
    clouds.clear();
    transforms.clear();
    indices.clear();

    map<int,Eigen::Vector4f*> pose_map;
    map<int,PointCloud<PointT>::Ptr> cloud_map;
    map<int,Eigen::Matrix4f*> transform_map;
    map<int,PointIndices*> ix_map;

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            //cout << f << endl;
            // Get the number
            size_t underscore = f.find_last_of('_');
            size_t dot = f.find_last_of('.');
            if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
            {
                int str_len = dot - underscore;
                string str_ix = f.substr(underscore+1,str_len-1);
                int ix = atoi(str_ix.c_str());
                string filename = add_backslash(dir) + f;

                if (strncmp(f.c_str(),_CLOUD_PREFIX, sizeof(_CLOUD_PREFIX)-1) == 0)
                {
                    PointCloud<PointT>::Ptr in_cloud (new PointCloud<PointT>());
                    if (io::loadPCDFile<PointT> (filename.c_str(), *in_cloud) == -1)
                        ROS_WARN("io_utils::load_model_training_directory : could not read point cloud file %s", filename.c_str());
                    else
                        cloud_map[ix] = in_cloud;
                    // Get the transform and the pose
                    PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
                    Eigen::Vector4f position;
                    Eigen::Matrix4f transform;
                    if (!transform_cloud_from_file(dir, f, *in_cloud, *transformed_cloud, position, transform))
                    {
                        ROS_WARN("io_utils::load_test_directory : error transforming point cloud from file %s",
                                  filename.c_str());
                        transform_map[ix] = NULL;
                        pose_map[ix] = NULL;
                    }
                    else
                    {
                        // WARNING : these files the transform is defined in the opposite way
                        // The transform is correct as it brings all clouds into alignment
                        // Take the inverse because it is reversed in the transform_cloud_from_file function
                        //Eigen::Matrix4f tri = transform.inverse();  // CHANGE 16 JUNE - removed inverse
                        Eigen::Matrix4f tri = transform;
                        if (invert_transform)
                            tri = tri.inverse();
                        Eigen::Vector4f *ps_ptr (new Eigen::Vector4f(position));
                        pose_map[ix] = ps_ptr;
                        Eigen::Matrix4f *tr_ptr (new Eigen::Matrix4f(tri));
                        transform_map[ix] = tr_ptr;
                    }
                }
                else if (strncmp(f.c_str(),_INDICES_PREFIX, sizeof(_INDICES_PREFIX)-1) == 0)
                {
                    PointCloud<IndexPoint>::Ptr in_cloud (new PointCloud<IndexPoint>);
                    if (io::loadPCDFile<IndexPoint>(filename.c_str(), *in_cloud) == -1)
                    {
                        ROS_WARN("io_utils::load_model_training_directory : could not read index file");
                    }
                    else
                    {
                        PointIndices *indices  = new PointIndices();
                        indices->indices.resize(in_cloud->points.size());
                        for (size_t i = 0; i < in_cloud->points.size(); ++i)
                          indices->indices[i] = in_cloud->points[i].idx;
                        ix_map[ix] = indices;
                    }
                }
            }
        }
    }
    closedir(dp);

    // Extract the points belonging to the object in each view and store to the entropy map
    //ROS_INFO("io_utils::load_model_training_directory : extracting points for models and transforming");
    for(map<int,PointCloud<PointT>::Ptr>::iterator it = cloud_map.begin(); it != cloud_map.end(); ++it)
    {
        // it->first = key
        // it->second = value

        // Point cloud
        clouds.push_back(*it->second);
        // Poe
        if (pose_map[it->first] != NULL)
            poses.push_back(*pose_map[it->first]);
        else
            poses.push_back(Eigen::Vector4f(0,0,0,0));
        // Transform
        if (transform_map[it->first] != NULL)
            transforms.push_back (*transform_map[it->first]);
        else
            transforms.push_back (Eigen::Matrix4f::Identity());
        // Indices
        vector<int> ix;
        if (ix_map[it->first] != NULL)
            ix = ix_map[it->first]->indices;
        indices.push_back (ix);
    }

    return true;
}

int get_next_write_index(const string &dir, const string &prefix)
{
    int next_index = -1;

    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        ROS_ERROR("io_utils::get_next_write_index : could not open %s", dir.c_str());
        return next_index;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            // If the first letters match _CONFIG_FILE
             if (strncmp(f.c_str(),prefix.c_str(), sizeof(prefix.c_str())-1) == 0)
             {
                 // Get the number
                 size_t underscore = f.find_last_of('_');
                 size_t dot = f.find_last_of('.');
                 if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
                 {
                     int str_len = dot - underscore;
                     string str_ix = f.substr(underscore+1,str_len-1);
                     int ix = atoi(str_ix.c_str());
                     if (ix > next_index)
                         next_index = ix;
                 }
             }
        }
    }
    // Return +1 of the largest index found
    ++next_index;
    return next_index;
}
