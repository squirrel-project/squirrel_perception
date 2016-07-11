#include "squirrel_active_exploration/pc_utils.h"

using namespace std;
using namespace pcl;
using namespace octomap;

//template<typename T>
//Eigen::Matrix4f get_transfrom_to_origin_frame(const pcl::PointCloud<T> &cloud)
//{
//    // Compute the centroid
//    Eigen::Vector4f centroid;
//    compute3DCentroid (cloud, centroid);
//    // Compute the principle axis
//    float d_max = 0;
//    int best_ix = 0;
//    for (typename pcl::PointCloud<T>::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
//    {
//        // If distance is larger
//        float d = distance3D(centroid[0], centroid[1], centroid[2], it->x, it->y, it->z);
//        if (d > d_max)
//        {
//            d_max = d;
//            best_ix = distance<typename pcl::PointCloud<T>::const_iterator>(cloud.begin(),it);
//        }
//    }
//    pair<float,float> a = angle3D(centroid[0], centroid[1], centroid[2], cloud[best_ix].x, cloud[best_ix].y, cloud[best_ix].z, d_max);
//    float pax_theta = a.first;
//    float pax_phi = a.second;
//    // Convert to a transformation matrix
//    Eigen::Matrix4f transform;

//    return transform;
//}

//template<typename T>
//void organize_point_cloud(const PointCloud<T> &cloud, PointCloud<T> &cloud_organized)
//{
//    // Transform cloud to organized point cloud and then to image
//    int height = 480;
//    int width = 640;
//    cloud_organized.clear();
//    cloud_organized.width = width;
//    cloud_organized.height = height;
//    cloud_organized.is_dense = true;
//    cloud_organized.points.resize(height*width);
//    for (size_t i = 0; i < cloud_organized.points.size(); ++i)
//    {
//        cloud_organized.points[i].x = cloud_organized.points[i].y = cloud_organized.points[i].z =
//                std::numeric_limits<float>::quiet_NaN();
//        cloud_organized.points[i].r = cloud_organized.points[i].g = cloud_organized.points[i].b = 0;
//    }

//    float f = 525.f;
//    float cx = (static_cast<float> (width) / 2.f - 0.5f);
//    float cy = (static_cast<float> (height) / 2.f - 0.5f);

//    int ws2 = 1;
//    for (size_t i = 0; i < cloud.points.size (); ++i)
//    {
//        float x = cloud.points[i].x;
//        float y = cloud.points[i].y;
//        float z = cloud.points[i].z;
//        int u = static_cast<int> (f * x / z + cx);
//        int v = static_cast<int> (f * y / z + cy);

//        for (int j = (u-ws2); j < (u+ws2); ++j)
//        {
//            for (int k = (v-ws2); k < (v+ws2); ++k)
//            {
//                // Not out of bounds
//                if ((j >= static_cast<int> (width)) || (k >= static_cast<int> (height)) || (i < 0) || (j < 0))
//                    continue;

//                float z_oc = cloud_organized.at(j,k).z;

//                if (pcl_isnan(z_oc))
//                {
//                    cloud_organized.at(j,k) = cloud.points[i];
//                }
//                else
//                {
//                    if (z < z_oc)
//                        cloud_organized.at(j,k) = cloud.points[i];
//                }
//            }
//        }
//    }
//}

Eigen::Vector4f extract_camera_position(const PointCloud<PointT> &cloud, const double &radius)
{
    // Calculate the mean of the point cloud
    Eigen::Vector4f centroid;
    compute3DCentroid (cloud, centroid);
    // Get the spherical coordinate system angles, given that the camera is at origin (0,0,0)
    // (values are negative to get the angle from the centroid to the camera)
    double theta = acos(-centroid[2] / sqrt(pow(centroid[0],2)+pow(centroid[1],2)+pow(centroid[2],2)));
    double phi = atan(centroid[1]/centroid[0]);
    // Get a the sensor location that is at a certain radius in direction from centroid towards origin
    Eigen::Vector4f cam_pos;
    cam_pos[0] = centroid[0] + radius*sin(theta)*cos(phi);
    cam_pos[1] = centroid[1] + radius*sin(theta)*sin(phi);
    cam_pos[2] = centroid[2] + radius*cos(theta);
    cam_pos[3] = 0;

    return cam_pos;
}

bool icp(const PointCloud<PointT>::Ptr source, const PointCloud<PointT>::Ptr target, Eigen::Matrix4f &transform, double &score, const bool &downsample)
{
    //cout << "utils::icp : starting icp" << endl;
    // Set the transform to the identity matrix
    transform = Eigen::Matrix4f::Identity();
    // Set the score to infinity
    score = numeric_limits<double>::infinity();

    //
    // Downsample for consistency and speed
    // enable this for large datasets
    PointCloud<PointT>::Ptr src (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr tgt (new PointCloud<PointT>());

    VoxelGrid<PointT> grid;
    double start_res = 0.01;

    if (downsample && source->size() > _ICP_POINT_MAX)
    {
        grid.setLeafSize (start_res, start_res, start_res);
        grid.setInputCloud (source);
        grid.filter (*src);
        if (src->size() > _ICP_POINT_MAX)
        {
            grid.setLeafSize (start_res*5.0, start_res*5.0, start_res*5.0);
            grid.setInputCloud (source);
            grid.filter (*src);
        }
        else if (src->size() < _ICP_POINT_MIN)
        {
            grid.setLeafSize (start_res/2.0, start_res/2.0, start_res/2.0);
            grid.setInputCloud (source);
            grid.filter (*src);
        }
        if (src->size() < _ICP_POINT_MIN)
            src = source;
    }
    else
    {
        src = source;
    }
    if (downsample && target->size() > _ICP_POINT_MAX)
    {
        grid.setLeafSize (start_res, start_res, start_res);
        grid.setInputCloud (target);
        grid.filter (*tgt);
        if (tgt->size() > _ICP_POINT_MAX)
        {
            grid.setLeafSize (start_res*5.0, start_res*5.0, start_res*5.0);
            grid.setInputCloud (target);
            grid.filter (*tgt);
        }
        else if (tgt->size() < _ICP_POINT_MIN)
        {
            grid.setLeafSize (start_res/2.0, start_res/2.0, start_res/2.0);
            grid.setInputCloud (target);
            grid.filter (*tgt);
        }
        if (tgt->size() < _ICP_POINT_MIN)
            tgt = target;
    }
    else
    {
        tgt = target;
    }
    //cout << "source " << src->size() << " (" << source->size() << ")" << endl;
    //cout << "target " << tgt->size() << " (" << target->size() << ")" << endl;

    // Compute surface normals and curvature
    PointCloud<PointNormal>::Ptr points_with_normals_src (new PointCloud<PointNormal>);
    PointCloud<PointNormal>::Ptr points_with_normals_tgt (new PointCloud<PointNormal>);

    NormalEstimation<PointT,PointNormal> norm_est;
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    //cout << "source with normals " << points_with_normals_src->size() << endl;
    copyPointCloud (*src, *points_with_normals_src);
    //cout << "source with normals " << points_with_normals_src->size() << endl;
    //cout << "source " << src->size() << endl;

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    //cout << "target with normals " << points_with_normals_tgt->size() << endl;
    copyPointCloud (*tgt, *points_with_normals_tgt);
    //cout << "target with normals " << points_with_normals_tgt->size() << endl;
    //cout << "target " << tgt->size() << endl;

    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    // Align
    IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
    reg.setTransformationEpsilon (1e-5);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.5);
    // Set the point representation
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    PointCloud<PointNormal>::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations (2);

//    // Print out the source and target
//    cout << "SOURCE" << endl;
//    for (int i = 0; i < points_with_normals_src->size(); ++i)
//        cout << points_with_normals_src->points[i].x << " " << points_with_normals_src->points[i].y << " " << points_with_normals_src->points[i].z << " "
//             << points_with_normals_src->points[i].normal_x << " " << points_with_normals_src->points[i].normal_y << " " << points_with_normals_src->points[i].normal_z << endl;
//    cout << "TARGET" << endl;
//    for (int i = 0; i < points_with_normals_tgt->size(); ++i)
//        cout << points_with_normals_tgt->points[i].x << " " << points_with_normals_tgt->points[i].y << " " << points_with_normals_tgt->points[i].z << " "
//             << points_with_normals_tgt->points[i].normal_x << " " << points_with_normals_tgt->points[i].normal_y << " " << points_with_normals_tgt->points[i].normal_z << endl;
//    cout << "Num points are: source = " << points_with_normals_src->size() << ", target = " << points_with_normals_tgt->size() << endl;
    try
    {
        for (int i = 0; i < 30; ++i)
        {
            //PCL_INFO ("Iteration Nr. %d.\n", i);

            // save cloud for visualization purpose
            points_with_normals_src = reg_result;

            // Estimate
            reg.setInputSource (points_with_normals_src);
            reg.align (*reg_result);

            //accumulate transformation between each Iteration
            Ti = reg.getFinalTransformation () * Ti;

            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
            if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
              reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

            prev = reg.getLastIncrementalTransformation ();
        }
    }
    catch(...)
    {
        printf(ANSI_COLOR_RED  "ERROR pc_utils::icp : could not perform icp!"  ANSI_COLOR_RESET "\n");
    }

    score = reg.getFitnessScore();
    transform = Ti;

//    cout << "has converged: " << reg.hasConverged() << " score: " << reg.getFitnessScore() << endl;
//    cout << reg.getFinalTransformation() << endl;

//    PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
//    transformPointCloud(*source, *transformed_cloud, transform);
//    string save_file = "source.pcd";
//    io::savePCDFileASCII (save_file, *source);
//    save_file = "target.pcd";
//    io::savePCDFileASCII (save_file, *target);
//    save_file = "transformed.pcd";
//    io::savePCDFileASCII (save_file, *transformed_cloud);

    return reg.hasConverged();
}

//bool get_ground_plane(const PointCloud<PointT>::Ptr &in_cloud, vector<int> &indices)
//{
//    SACSegmentation<PointT> seg;
//    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
//    PointIndices::Ptr inliers (new PointIndices);
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (SACMODEL_PLANE);
//    seg.setMethodType (SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.02);
//    seg.setInputCloud (in_cloud);
//    seg.segment (*inliers, *coefficients);
//    if (inliers->indices.size () == 0)
//    {
//        printf(ANSI_COLOR_RED  "ERROR utils::get_ground_plane : failed to get ground plane"  ANSI_COLOR_RESET "\n");
//        return false;
//    }
//    indices = inliers->indices;
//    return true;
//}

//bool get_ground_plane(const PointCloud<PointT> &in_cloud, vector<int> &indices)
//{
//    PointCloud<PointT>::Ptr cloud_ptr (new PointCloud<PointT>(in_cloud));
//    return get_ground_plane(cloud_ptr, indices);
//}

//bool get_ground_plane(const PointCloud<PointT>::Ptr &in_cloud, vector<int> &indices, PointCloud<PointT> &ground, PointCloud<PointT> &remaining)
//{
//    if (!get_ground_plane(in_cloud, indices))
//    {
//        printf(ANSI_COLOR_RED  "ERROR utils::get_ground_plane : failed to call get ground plane"  ANSI_COLOR_RESET "\n");
//        return false;
//    }

//    PointIndices::Ptr inliers (new PointIndices);
//    inliers->indices = indices;

//    // Extract the planar inliers from the input cloud
//    ExtractIndices<PointT> extract;
//    extract.setInputCloud (in_cloud);
//    extract.setIndices (inliers);
//    extract.setNegative (false);
//    extract.filter (ground);

//    // Extract the planer outliers
//    extract.setNegative (true);
//    extract.filter (remaining);

//    return true;
//}

//bool get_ground_plane(const PointCloud<PointT> &in_cloud, vector<int> &indices, PointCloud<PointT> &ground, PointCloud<PointT> &remaining)
//{
//    PointCloud<PointT>::Ptr cloud_ptr (new PointCloud<PointT>(in_cloud));
//    return get_ground_plane(cloud_ptr, indices, ground, remaining);
//}

bool find_planes(const PointCloud<PointT> &cloud, vector<int> &plane_indices)
{
    plane_indices.clear();

    // Check that a point cloud exists
    if (cloud.size() == 0)
    {
        printf(ANSI_COLOR_RED  "ERROR utils::find_planes : input cloud is empty"  ANSI_COLOR_RESET "\n");
        return false;
    }

    // Otherwise find the planes
    PointCloud<PointT>::Ptr in_cloud (new PointCloud<PointT>(cloud));
    SACSegmentation<PointT> seg;
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (in_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        printf(ANSI_COLOR_RED  "ERROR utils::find_planes : failed to get ground plane"  ANSI_COLOR_RESET "\n");
        return false;
    }

    // Add the points to the all_planes vector
    vector<int> all_planes;
    all_planes.insert(all_planes.end(), inliers->indices.begin(), inliers->indices.end());

    // Extract the planar inliers from the input cloud
    ExtractIndices<PointT> extract;
    extract.setInputCloud (in_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    PointCloud<PointT>::Ptr cloud_plane (new PointCloud<PointT> ());
    extract.filter (*cloud_plane);

    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
    tree->setInputCloud (in_cloud);
    vector<PointIndices> ec_cluster_indices;
    EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (cloud_plane->size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_plane);
    ec.extract (ec_cluster_indices);

    // Get size of largest cluster
    int max_cluster = 0;
    for (vector<PointIndices>::size_type i = 0; i < ec_cluster_indices.size(); ++i)
    {
        if (ec_cluster_indices[i].indices.size() > max_cluster)
            max_cluster = ec_cluster_indices[i].indices.size();
    }

    // Current index mapping
    vector<int> ix_mapping (in_cloud->size());
    for (int i = 0; i < ix_mapping.size(); ++i)
        ix_mapping[i] = i;

    PointCloud<PointT>::Ptr curr_cloud (new PointCloud<PointT>(*in_cloud));
    PointCloud<PointT>::Ptr curr_plane (new PointCloud<PointT> (*cloud_plane));
    int iteration = 0;
    while (max_cluster > 10000)
    {
        printf("utils::find_planes : iteration %u, largest plane with %u points\n", iteration, max_cluster);

        // Add the indices to the all_planes vector
        if (iteration > 0)
        {
            // curr_cloud is the cloud that has just been segmented
            // inliers->indices has the indices of the plane in this cloud
            // ix_mapping maps each point in curr_cloud to the corresponding point
            // in the original cloud in_cloud
            // Therefore, add each ix_mapping[inliers->indices[i]] to all_planes
            vector<int> original_plane;
            for (size_t i = 0; i < inliers->indices.size(); ++i)
            {
                all_planes.push_back(ix_mapping[inliers->indices[i]]);
                original_plane.push_back(ix_mapping[inliers->indices[i]]);
            }
        }

        // Store the indices of the cloud
        vector<int> ix_vec (curr_cloud->size());
        for (size_t i = 0; i < curr_cloud->size(); ++i)
            ix_vec[i] = i;

        // Extract the remaining cloud
        extract.setInputCloud (curr_cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*curr_cloud);

        // Get the indices into the original point cloud
        vector<int> outliers (ix_vec.size() + inliers->indices.size());
        vector<int>::iterator diff_itr = set_difference (ix_vec.begin(), ix_vec.end(),
                                                         inliers->indices.begin(), inliers->indices.end(),
                                                         outliers.begin());
        outliers.resize(diff_itr - outliers.begin());
        vector<int> temp = ix_mapping;
        ix_mapping.clear();
        ix_mapping.resize(outliers.size());
        for (vector<PointIndices>::size_type i = 0; i < outliers.size(); ++i)
            ix_mapping[i] = temp[outliers[i]];
        // ix_mapping is the same length as curr_cloud
        // such that curr_cloud[0] = in_cloud[ix_maping[0]]

        // Segment the largest planar component from the remaining cloud
        inliers->indices.clear();
        seg.setInputCloud (curr_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            printf(ANSI_COLOR_RED  "ERROR utils::find_planes : failed to get ground plane"  ANSI_COLOR_RESET "\n");
            return false;
        }

        // Extract the planar inliers from the input cloud
        extract.setInputCloud (curr_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*curr_plane);

        // Creating the KdTree object for the search method of the extraction
        tree->setInputCloud (curr_cloud);
        ec.setMaxClusterSize (curr_plane->size());
        ec.setSearchMethod (tree);
        ec.setInputCloud (curr_plane);
        ec_cluster_indices.clear();
        ec.extract (ec_cluster_indices);

        // Get size of largest cluster
        max_cluster = 0;
        for (vector<PointIndices>::size_type i = 0; i < ec_cluster_indices.size(); ++i)
        {
            if (ec_cluster_indices[i].indices.size() > max_cluster)
                max_cluster = ec_cluster_indices[i].indices.size();
        }

        // Next iteration
        ++iteration;
    }

    printf(ANSI_COLOR_YELLOW  "WARN utils::find_planes : end after iteration %u, largest plane with %u points"  ANSI_COLOR_RESET "\n",
           iteration, max_cluster);

    sort(all_planes.begin(), all_planes.end());
    all_planes.erase(unique(all_planes.begin(), all_planes.end()), all_planes.end());

    plane_indices = all_planes;

    return true;
}
