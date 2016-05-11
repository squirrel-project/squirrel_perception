#include "squirrel_active_exploration/transform_utils.h"

using namespace std;
using namespace pcl;

bool frame_to_frame_tf(const string &frame1, const string &frame2, Eigen::Matrix4f &transform)
{
    transform = Eigen::Matrix4f::Identity();
    tf::TransformListener listener;
    tf::StampedTransform tf;
    ros::Time now = ros::Time(0);

    ros::Rate rate(10.0);
    int count = 0;
    while (true)
    {
        try
        {
            ros::Duration(1.0).sleep();
            listener.lookupTransform(frame1, frame2, now, tf);
            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            ++count;
            if (count == 5)
            {
                ROS_ERROR("transform_utils::frame_to_frame_tf : could not get transformation");
                return false;
            }
        }

        rate.sleep();
    }

    // Extract the transform from the StampedTransform object
    // Get the rotation component
    tf::Matrix3x3 rotation = tf.getBasis();
    for (int r = 0; r < 3; ++r)
    {
        tf::Vector3 row = rotation[r];
        transform(r,0) = row.getX();
        transform(r,1) = row.getY();
        transform(r,2) = row.getZ();
    }
    // Get the translation component
    tf::Vector3 translation = tf.getOrigin();
    transform(0,3) = translation.getX();
    transform(1,3) = translation.getY();
    transform(2,3) = translation.getZ();

    return true;
}

bool frame_to_frame(const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud, const string &frame1, const string &frame2)
{
    // Update the transform if it has changed
    tf::TransformListener listener;
    ros::Time now = ros::Time(0);
    try
    {
        out_cloud.resize(in_cloud.size());
        // Copy the point cloud (will copy RGB values as well as header and size)
        copyPointCloud(in_cloud, out_cloud);
        ros::Duration(1.0).sleep();
        listener.waitForTransform(frame1, frame2, now, ros::Duration(0.5));
        for(size_t i = 0; i < in_cloud.points.size() ; i++)
        {
            geometry_msgs::PointStamped p, pb;
            p.point.x = in_cloud.points[i].x;
            p.point.y = in_cloud.points[i].y;
            p.point.z = in_cloud.points[i].z;
            p.header.frame_id = frame1;
            listener.transformPoint(frame2, p, pb);
            out_cloud.points[i].x = pb.point.x;
            out_cloud.points[i].y = pb.point.y;
            out_cloud.points[i].z = pb.point.z;
        }
        return true;
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("transform_utils::frame_to_frame : could not update TF %s to %s", frame1.c_str(), frame2.c_str());
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

bool frame_to_frame(const string &frame1, const string &frame2, const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud)
{
    return frame_to_frame (in_cloud, out_cloud, frame1, frame2);
}

bool frame_to_frame(const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud, const string &frame1, const string &frame2)
{
    // Convert to pcl::PointCloud
    PointCloud<PointT> pcl_in_cloud;
    fromROSMsg (in_cloud, pcl_in_cloud);
    PointCloud<PointT> pcl_out_cloud;
    // Call the transform function
    if (!(frame_to_frame(pcl_in_cloud, pcl_out_cloud, frame1, frame2)))
    {
        ROS_ERROR("transform_utils::frame_to_frame : error in transforming point cloud");
        return false;
    }
    // Convert back to ros message
    toROSMsg(pcl_out_cloud, out_cloud);
    return true;
}

bool frame_to_frame(const string &frame1, const string &frame2, const sensor_msgs::PointCloud2 &in_cloud, sensor_msgs::PointCloud2 &out_cloud)
{
    return frame_to_frame (in_cloud, out_cloud, frame1, frame2);
}

bool transform_cloud_from_file(const string &filename, const PointCloud<PointT> &in_cloud,
                               PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose, Eigen::Matrix4f &transform)
{
    //ROS_INFO("transform_utils::transform_cloud_from_file : starting");
    // Transformation matrix
    transform = Eigen::Matrix4f::Identity();
    // Read the file
    if (read_tf_file(filename, transform))
    {
        // Transform the point cloud
        //Eigen::Matrix4f tr_inv = transform.inverse();
        transformPointCloud (in_cloud, out_cloud, transform); // CHANGE 16 JUNE - removed inverse
        // Camera pose
        camera_pose = extract_camera_position (in_cloud);
        // Transform to model coordinate frame
        //camera_pose = tr_inv * camera_pose; // changed
        //ROS_INFO("transform_utils::transform_cloud_from_file : input cloud has %lu points", in_cloud.size());
        //ROS_INFO("transform_utils::transform_cloud_from_file : output cloud has %lu points", out_cloud.size());
    }
    else
    {
        ROS_ERROR("transform_utils::transform_cloud_from_file : could not load transformation file %s", filename.c_str());
        return false;
    }

    // Return the inverse transform
    //transform = transform.inverse();   // CHANGE 16 JUNE - removed inverse
    //ROS_INFO("transform_utils::transform_cloud_from_file : finished");
    return true;
}

bool transform_cloud_from_file(const string &filename, const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose)
{
    Eigen::Matrix4f transform;
    return transform_cloud_from_file (filename, in_cloud, out_cloud, camera_pose, transform);
}

bool transform_cloud_from_file(const string &filename, const PointCloud<PointT> &in_cloud,
                               PointCloud<PointT> &out_cloud, PointCloud<PointT> &camera_pose, Eigen::Matrix4f &transform)
{
    // Call with Eigen::Vector input
    Eigen::Vector4f eig_pos;
    if (!transform_cloud_from_file (filename, in_cloud, out_cloud, eig_pos, transform))
    {
        ROS_ERROR("transform_utils::transform_cloud_from_file : error when transforming point cloud");
        return false;
    }
    // Otherwise transform eig_pos to a point cloud
    camera_pose.clear();
    camera_pose = eig_to_pcl(eig_pos);
    return true;
}

bool transform_cloud_from_file(const string &filename, const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud, PointCloud<PointT> &camera_pose)
{
    Eigen::Matrix4f transform;
    return transform_cloud_from_file (filename, in_cloud, out_cloud, camera_pose, transform);
}

bool transform_cloud_from_file(const string &dir, const string &filename, const PointCloud<PointT> &in_cloud,
                               PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose, Eigen::Matrix4f &transform)
{
    //cout << "input is " << dir << " " << filename << endl;
    vector<string> prefixes;
    prefixes.push_back ("transformation_");
    prefixes.push_back ("transform_");
    prefixes.push_back ("trans_");
    prefixes.push_back ("transformation");
    prefixes.push_back ("transform");
    prefixes.push_back ("trans");
    prefixes.push_back ("pose_");
    prefixes.push_back ("pose");
    // Get the full path and filename for the transformation matrix
    // Find the '.' in the filename
    size_t dot = filename.find_last_of('.');
    if (dot == string::npos)
    {
        ROS_ERROR("transform_utils::transform_cloud_from_file: no extension in file %s", filename.c_str());
        return false;
    }
    // Otherwise extract the extension
    string ext = filename.substr(dot);
    // If the filename ends with .txt, then already the correct filename
    if (strcmp(ext.c_str(),".txt") == 0)
    {
        // Call the transformation function with dir/filename
        string full_filename = add_backslash(dir) + filename;
        return transform_cloud_from_file(full_filename, in_cloud, out_cloud, camera_pose, transform);
    }
    // Else if the filename is the point cloud name and ends with .pcd, then find the
    // corresponding transformation file
    else if (strcmp(ext.c_str(),".pcd") == 0)
    {
        string full_filename;
        ifstream myfile;
        bool valid_file;
        // Find remove extension .pcd and replace with .txt
        full_filename = add_backslash(dir) + filename.substr(0,dot) + ".txt";
        // Try this filename
        myfile.open(full_filename.c_str());
        valid_file = myfile.is_open();
        myfile.close();
        // If this is a valid file
        if (valid_file)
            return transform_cloud_from_file(full_filename, in_cloud, out_cloud, camera_pose, transform);
        // Otherwise try with prefixes
        for (vector<string>::size_type i = 0; i < prefixes.size(); ++i)
        {
            full_filename = add_backslash(dir) + prefixes[i] + filename.substr(0,dot) + ".txt";
            // Try this filename
            myfile.open(full_filename.c_str());
            valid_file = myfile.is_open();
            myfile.close();
            // If this is a valid file
            if (valid_file)
            {
                //ROS_INFO("transform_utils::transform_cloud_from_file : found transformation file %s", full_filename.c_str());
                return transform_cloud_from_file(full_filename, in_cloud, out_cloud, camera_pose, transform);
            }
        }
        // Otherwise try removing everything before the last underscore and replacing with a prefix
        size_t underscore = filename.find_last_of('_');
        if (underscore == string::npos || (dot - underscore) <= 0)
        {
            ROS_ERROR("transform_utils::transform_cloud_from_file : cannot find underscore and dot in file %s", filename.c_str());
            return false;
        }
        // Otherwise remove the characters before the last underscore
        int str_len = dot - underscore;
        string str_ix = filename.substr(underscore+1,str_len-1);
        // Try with prefixes
        for (vector<string>::size_type i = 0; i < prefixes.size(); ++i)
        {
            full_filename = add_backslash(dir) + prefixes[i] + str_ix + ".txt";
            // Try this filename
            myfile.open(full_filename.c_str());
            valid_file = myfile.is_open();
            myfile.close();
            // If this is a valid file
            if (valid_file)
            {
                //ROS_INFO("transform_utils::transform_cloud_from_file : found transformation file %s", full_filename.c_str());
                return transform_cloud_from_file(full_filename, in_cloud, out_cloud, camera_pose, transform);
            }
        }
    }
    else
    {
        ROS_ERROR("transform_utils::transform_cloud_from_file : could not understand extension of file %s", filename.c_str());
        return false;
    }
}

bool transform_cloud_from_file(const string &dir, const string &filename, const PointCloud<PointT> &in_cloud,
                               PointCloud<PointT> &out_cloud, Eigen::Vector4f &camera_pose)
{
    Eigen::Matrix4f transform;
    return transform_cloud_from_file (dir, filename, in_cloud, out_cloud, camera_pose, transform);
}

bool transform_cloud_from_file(const string &dir, const string &filename, const PointCloud<PointT> &in_cloud,
                               PointCloud<PointT> &out_cloud, PointCloud<PointT> &camera_pose, Eigen::Matrix4f &transform)
{
    // Call with Eigen::Vector input
    Eigen::Vector4f eig_pos;
    if (!transform_cloud_from_file (dir, filename, in_cloud, out_cloud, eig_pos, transform))
    {
        ROS_ERROR("transform_utils::transform_cloud_from_file : error when transforming point cloud");
        return false;
    }
    // Otherwise transform eig_pos to a point cloud
    camera_pose.clear();
    camera_pose = eig_to_pcl(eig_pos);
    return true;
}

bool transform_cloud_from_file(const string &dir, const string &filename, const PointCloud<PointT> &in_cloud,
                               PointCloud<PointT> &out_cloud, PointCloud<PointT> &camera_pose)
{
    Eigen::Matrix4f transform;
    return transform_cloud_from_file (dir, filename, in_cloud, out_cloud, camera_pose, transform);
}

//bool transform_cloud_to_cloud(const PointCloud<PointT> &source, const Eigen::Vector4f &source_pose,
//                              const PointCloud<PointT> &target, const Eigen::Vector4f &target_pose,
//                              Eigen::Matrix4f &transform, double &score)
bool transform_cloud_to_cloud(const PointCloud<PointT> &source, const PointCloud<PointT> &target, Eigen::Matrix4f &transform, double &score)
{
    /* http://pointclouds.org/documentation/tutorials/matrix_transform.php

       Reminder: how transformation matrices work :

               |-------> This column is the translation
        | T 0 0 x |  \
        | 0 T 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
        | 0 0 T z |  /
        | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    */

    // ========= NEW =========== //
    transform = Eigen::Matrix4f::Identity();
    score = numeric_limits<double>::infinity();

    // Compute the pose of the target
    Eigen::Vector4f target_pose;
    compute3DCentroid (target, target_pose);

    // Compute the difference in axis length
    PointT sourcemin, sourcemax, targetmin, targetmax;
    getMinMax3D(source, sourcemin, sourcemax);
    getMinMax3D(target, targetmin, targetmax);
    vector<double> sourcedim(3);
    sourcedim[0] = sourcemax.data[0] - sourcemin.data[0];
    sourcedim[1] = sourcemax.data[1] - sourcemin.data[1];
    sourcedim[2] = sourcemax.data[2] - sourcemin.data[2];
    sort(sourcedim.begin(), sourcedim.end());
    vector<double> targetdim(3);
    targetdim[0] = targetmax.data[0] - targetmin.data[0];
    targetdim[1] = targetmax.data[1] - targetmin.data[1];
    targetdim[2] = targetmax.data[2] - targetmin.data[2];
    sort(targetdim.begin(), targetdim.end());
    // Take the average
    double scale = (targetdim[0]/sourcedim[0] + targetdim[1]/sourcedim[1] + targetdim[2]/sourcedim[2])/3;
    //ROS_INFO("transform_utils::transform_cloud_to_cloud : applying scale change %.4f", scale);
    // Scale the point cloud
    Eigen::Matrix4f stf = Eigen::Matrix4f::Identity();
    stf.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * scale;
    PointCloud<PointT> source_copy;
    transformPointCloud (source, source_copy, stf);

    cout << "transform_cloud_to_cloud : after scaling, segment has " << source_copy.size() << " points" << endl;
    int nan_count = 0, valid_count = 0;
    for (size_t j = 0; j < source_copy.size(); ++j)
    {
        //cout << source_copy.points[j].x << " " << source_copy.points[j].y << " " << source_copy.points[j].z << endl;
        if (isnan(source_copy.points[j].x))
            nan_count++;
        else
            valid_count++;
    }
    cout << "nan = " << nan_count << ", valid = " << valid_count << endl;

    // Transform the source to the center of its coordinate system
    Eigen::Vector4f source_pose;
    compute3DCentroid (source_copy, source_pose);
    Eigen::Matrix4f ctf = Eigen::Matrix4f::Identity();
    // Transform the source to its centre coordinate frame
    ctf(0,3) = -source_pose[0];
    ctf(1,3) = -source_pose[1];
    ctf(2,3) = -source_pose[2];
    transformPointCloud (source_copy, source_copy, ctf);

    cout << "transform_cloud_to_cloud : after centering, segment has " << source_copy.size() << " points" << endl;
    nan_count = 0;
    valid_count = 0;
    for (size_t j = 0; j < source_copy.size(); ++j)
    {
        //cout << source_copy.points[j].x << " " << source_copy.points[j].y << " " << source_copy.points[j].z << endl;
        if (isnan(source_copy.points[j].x))
            nan_count++;
        else
            valid_count++;
    }
    cout << "nan = " << nan_count << ", valid = " << valid_count << endl;


    // Downsample the point clouds
    PointCloud<PointT> source_ds;
    if (!downsample_point_cloud(source_copy, source_ds))
    {
        ROS_ERROR("transform_utils::transform_cloud_to_cloud : could not downsample the source point cloud");
        source_ds = source_copy;
    }
    PointCloud<PointT> target_ds;
    if (!downsample_point_cloud(target, target_ds))
    {
        ROS_ERROR("transform_utils::transform_cloud_to_cloud : could not downsample the target point cloud");
        target_ds = target;
    }
    cout << "Downsampled clouds to: source = " << source_ds.size() << " target = " << target_ds.size() << endl;
    // Into pointers
    PointCloud<PointT>::Ptr source_ds_ptr (new PointCloud<PointT>(source_ds));
    PointCloud<PointT>::Ptr target_ptr (new PointCloud<PointT>(target_ds));


//    PointCloud<PointT>::Ptr target_ds (new PointCloud<PointT>());
//    if (target_ptr->size() > _ICP_POINT_MAX)
//    {
//        grid.setInputCloud (target_ptr);
//        grid.filter (*target_ds);
//        if (target_ds->size() > _ICP_POINT_MAX)
//        {
//            grid.setLeafSize (start_res*5.0, start_res*5.0, start_res*5.0);
//            grid.setInputCloud (target_ptr);
//            grid.filter (*target_ds);
//        }
//        else if (target_ds->size() < _ICP_POINT_MIN)
//        {
//            grid.setLeafSize (start_res/2.0, start_res/2.0, start_res/2.0);
//            grid.setInputCloud (target_ptr);
//            grid.filter (*target_ds);
//        }
//        if (target_ds->size() < _ICP_POINT_MIN)
//            target_ds = target_ptr;
//    }
//    else
//    {
//        target_ds = target_ptr;
//    }


    // Try many different rotations and return the best rotation
    PointCloud<PointT>::Ptr source_ptr (new PointCloud<PointT>());
    for (int i = 0; i < _NUM_ROTATIONS_ICP; ++i)
    {
        double theta = i*2*M_PI/_NUM_ROTATIONS_ICP;
        vector<Eigen::Matrix4f> rotations = rotation_matrix(theta);
        // Try the different rotations
        for (size_t j = 0; j < rotations.size(); ++j)
        {
            cout << "1. ICP with source_ds = " << source_ds_ptr->size() << endl;
            Eigen::Matrix4f rtf = rotations[j];
            transformPointCloud(*source_ds_ptr, *source_ptr, rtf);
            cout << "2. ICP with source = " << source_ptr->size() << endl;
            // Align the center of the cloud before doing icp
            Eigen::Vector4f rotated_pose;
            compute3DCentroid (*source_ptr, rotated_pose);
            Eigen::Matrix4f atf = Eigen::Matrix4f::Identity();
            // Transform the source to centre
            atf(0,3) = target_pose[0] - rotated_pose[0];
            atf(1,3) = target_pose[1] - rotated_pose[1];
            atf(2,3) = target_pose[2] - rotated_pose[2];
            transformPointCloud (*source_ptr, *source_ptr, atf);
            cout << "3. ICP with source = " << source_ptr->size() << endl;
            // ICP to refine the alignment
            Eigen::Matrix4f itf;
            double icp_score;
            cout << "4. ICP with source = " << source_ptr->size() << " target = " << target_ptr->size() << endl;
            if (!icp(source_ptr, target_ptr, itf, icp_score, false))
            {
                ROS_ERROR("transform_utils::transform_cloud_to_cloud : error in icp refinement");
                icp_score = numeric_limits<double>::infinity();
            }
            // If the score is better then set the new transform
            if (icp_score < score)
            {
                //transformPointCloud(*source_ptr, *source_ptr, itf);
                //io::savePCDFileASCII ("icp_best.pcd", *source_ptr);
                transform = itf * atf * rtf * ctf * stf;
                score = icp_score;
                //cout << "score: " << score << endl;
            }
        }
    }

    if (score == numeric_limits<double>::infinity())
        return false;
    else
        return true;


//    // Begin with identity matrix
//    transform = Eigen::Matrix4f::Identity();
//    // Transform the source to centre
//    Eigen::Vector4f source_pose;
//    compute3DCentroid (source, source_pose);
//    Eigen::Vector4f target_pose;
//    compute3DCentroid (target, target_pose);
//    transform(0,3) = target_pose[0] - source_pose[0];
//    transform(1,3) = target_pose[1] - source_pose[1];
//    transform(2,3) = target_pose[2] - source_pose[2];
//    PointCloud<PointT>::Ptr source_ptr (new PointCloud<PointT>());
//    transformPointCloud (source, *source_ptr, transform);
//    // Make a pointer copy of the target
//    PointCloud<PointT>::Ptr target_ptr (new PointCloud<PointT>(target));

////    io::savePCDFileASCII ("icp_target.pcd", *target);
////    io::savePCDFileASCII ("icp_source.pcd", *source);
////    io::savePCDFileASCII ("icp_centered.pcd", *source_copy);

//    // ICP to refine the alignment
//    Eigen::Matrix4f icp_transform;
//    if (!icp(source_ptr, target_ptr, icp_transform, score))
//    {
//        ROS_ERROR("transform_utils::get_transform : error in icp refinement");
//        return false;
//    }
//    // Update the transform
//    //transform *= icp_transform;
//    transform = icp_transform * transform;

////    PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
////    transformPointCloud(*source, *transformed_cloud, transform);
////    string save_file = "icp_transformed.pcd";
////    io::savePCDFileASCII (save_file, *transformed_cloud);

//    return true;
}

//bool transform_cloud_to_cloud(const PointCloud<PointT> &source, const Eigen::Vector4f &source_pose,
//                              const PointCloud<PointT> &target,
//                              Eigen::Matrix4f &transform, double &score)
//{
//    // Compute the pose of the target
//    Eigen::Vector4f target_pose;
//    compute3DCentroid (target, target_pose);
//    // Compute the transform
//    return transform_cloud_to_cloud (source, source_pose, target, target_pose, transform, score);
//}

//bool transform_cloud_to_cloud(const PointCloud<PointT> &source,
//                              const PointCloud<PointT> &target, const Eigen::Vector4f &target_pose,
//                              Eigen::Matrix4f &transform, double &score)
//{
//    // Compute the pose of the source
//    Eigen::Vector4f source_pose;
//    compute3DCentroid (source, source_pose);
//    // Compute the transform
//    return transform_cloud_to_cloud (source, source_pose, target, target_pose, transform, score);
//}

//bool transform_cloud_to_cloud(const PointCloud<PointT> &source, const PointCloud<PointT> &target,
//                              Eigen::Matrix4f &transform, double &score)
//{
//    // Compute the pose of the source
//    Eigen::Vector4f source_pose;
//    compute3DCentroid (source, source_pose);
//    // Compute the pose of the target
//    Eigen::Vector4f target_pose;
//    compute3DCentroid (target, target_pose);
//    // Compute the transform
//    return transform_cloud_to_cloud (source, source_pose, target, target_pose, transform, score);
//}

bool transform_cloud_to_cloud_nonrigid(const PointCloud<PointT> &source, const PointCloud<PointT> &target,
                                       Eigen::Matrix4f &transform, double &score, double &scale)
{
    // Borrowed PCA and scaling from
    // http://www.morethantechnical.com/2012/08/12/registering-point-clouds-rigidly-with-scale-using-pcl-wcode/

    // Scale less than 1 makes the point cloud smaller
    // Scale larger than 1 makes the point cloud larger

    // Try unscaled to get a feeling for the score
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
    double best_score = numeric_limits<double>::infinity();
    if (!transform_cloud_to_cloud(source, target, best_transform, best_score))
    {
        ROS_ERROR("transform_utils::transform_cloud_to_cloud_nonrigid : could not call icp function");
        return false;
    }
    PointCloud<PointT> write_cloud;
    transformPointCloud(source, write_cloud, best_transform);

    PointCloud<PointT>::Ptr source_ptr (new PointCloud<PointT>(source));
    PointCloud<PointT>::Ptr target_ptr (new PointCloud<PointT>(target));

    // Compute PCA to get an initial scale alignment
    PCA<PointT> pca;
    pca.setInputCloud(source_ptr);
    Eigen::Vector3f ev_A = pca.getEigenValues();
    pca.setInputCloud(target_ptr);
    Eigen::Vector3f ev_B = pca.getEigenValues();
    double pca_scale = sqrt(ev_B[0])/sqrt(ev_A[0]);
    //cout << "PCA scale " << pca_scale << endl;




    // Compute the difference in axis length
    PointT sourcemin, sourcemax, targetmin, targetmax;
    getMinMax3D(source, sourcemin, sourcemax);
    getMinMax3D(target, targetmin, targetmax);
    vector<double> sourcedim(3);
    sourcedim[0] = sourcemax.data[0] - sourcemin.data[0];
    sourcedim[1] = sourcemax.data[1] - sourcemin.data[1];
    sourcedim[2] = sourcemax.data[2] - sourcemin.data[2];
    sort(sourcedim.begin(), sourcedim.end());
    vector<double> targetdim(3);
    targetdim[0] = targetmax.data[0] - targetmin.data[0];
    targetdim[1] = targetmax.data[1] - targetmin.data[1];
    targetdim[2] = targetmax.data[2] - targetmin.data[2];
    sort(targetdim.begin(), targetdim.end());
    // Print out the difference in dimensions
    //cout << "0: " << targetdim[0]/sourcedim[0] << endl;
    //cout << "1: " << targetdim[1]/sourcedim[1] << endl;
    //cout << "2: " << targetdim[2]/sourcedim[2] << endl;
    double av_scale = (targetdim[0]/sourcedim[0] + targetdim[1]/sourcedim[1] + targetdim[2]/sourcedim[2])/3;
    //cout << "Average: " << av_scale << endl;

    // Scale the point cloud
    Eigen::Matrix4f T_scale = Eigen::Matrix4f::Identity();
    T_scale.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * av_scale;
    PointCloud<PointT> source_copy;
    transformPointCloud(source, source_copy, T_scale);

    // Apply many rotations and choose the best
    // Try some rotations
    // First center the point cloud
    Eigen::Vector4f source_pose;
    compute3DCentroid (source_copy, source_pose);
    Eigen::Matrix4f ctf = Eigen::Matrix4f::Identity();
    // Transform the source to centre
    ctf(0,3) = -source_pose[0];
    ctf(1,3) = -source_pose[1];
    ctf(2,3) = -source_pose[2];
    transformPointCloud (source_copy, source_copy, ctf);
    PointCloud<PointT> rotated;
    double best_rotation = 0;
    int best_rotation_direction = -1;
    for (int i = 0; i < 10; ++i)
    {
        double theta = i*2*M_PI/10;
        vector<Eigen::Matrix4f> rotations = rotation_matrix(theta);
        // Try the different rotations
        for (size_t j = 0; j < rotations.size(); ++j)
        {
            transformPointCloud(source_copy, rotated, rotations[j]);
            // Call icp
            Eigen::Matrix4f tf_scale = Eigen::Matrix4f::Identity();
            double score_scale = numeric_limits<double>::infinity();
            if (!transform_cloud_to_cloud(rotated, target, tf_scale, score_scale))
            {
                ROS_ERROR("transform_utils::transform_cloud_to_cloud_nonrigid : could not call icp function");
            }
            else
            {
                // If this is better than the current best then set this as the best
                if (score_scale < best_score)
                {
                    best_transform = tf_scale;
                    best_score = score_scale;
                    scale = av_scale;
                    best_rotation = theta;
                    best_rotation_direction = j;
                }
            }
        }
    }
    //cout << "Best rotation angle " << best_rotation << endl;
    //cout << "Best rotation direction " << best_rotation_direction << endl;

    transform = best_transform;
    score = best_score;
    scale = av_scale;

    //cout << transform << endl;

    return true;
}

vector<Eigen::Matrix4f> rotation_matrix(const double &theta)
{
    vector<Eigen::Matrix4f> rotations;

    // Rotation in x
    Eigen::Matrix4f rx = Eigen::Matrix4f::Identity();
    rx(0,0) = 1;
    rx(0,1) = 0;
    rx(0,2) = 0;
    rx(0,3) = 0;
    rx(1,0) = 0;
    rx(1,1) = cos(theta);
    rx(1,2) = -sin(theta);
    rx(1,3) = 0;
    rx(2,0) = 0;
    rx(2,1) = sin(theta);
    rx(2,2) = cos(theta);
    rx(2,3) = 0;
    rx(3,0) = 0;
    rx(3,1) = 0;
    rx(3,2) = 0;
    rx(3,3) = 1;
    rotations.push_back(rx);

    // Rotation in y
    Eigen::Matrix4f ry = Eigen::Matrix4f::Identity();
    ry(0,0) = cos(theta);
    ry(0,1) = 0;
    ry(0,2) = sin(theta);
    ry(0,3) = 0;
    ry(1,0) = 0;
    ry(1,1) = 1;
    ry(1,2) = 0;
    ry(1,3) = 0;
    ry(2,0) = -sin(theta);
    ry(2,1) = 0;
    ry(2,2) = cos(theta);
    ry(2,3) = 0;
    ry(3,0) = 0;
    ry(3,1) = 0;
    ry(3,2) = 0;
    ry(3,3) = 1;
    rotations.push_back(ry);

    // Rotation in z
    Eigen::Matrix4f rz = Eigen::Matrix4f::Identity();
    rz(0,0) = cos(theta);
    rz(0,1) = -sin(theta);
    rz(0,2) = 0;
    rz(0,3) = 0;
    rz(1,0) = sin(theta);
    rz(1,1) = cos(theta);
    rz(1,2) = 0;
    rz(1,3) = 0;
    rz(2,0) = 0;
    rz(2,1) = 0;
    rz(2,2) = 1;
    rz(2,3) = 0;
    rz(3,0) = 0;
    rz(3,1) = 0;
    rz(3,2) = 0;
    rz(3,3) = 1;
    rotations.push_back(rz);

//    // Combined
//    Eigen::Matrix4f r = rx * ry * rz;
//    rotations.push_back(r);

    // Return
    return rotations;
}

Eigen::Vector4f transform_eigvec(const Eigen::Vector4f &vec, const Eigen::Matrix4f &transform)
{
    // Convert to a point cloud
    PointCloud<PointT> cloud;
    cloud.resize(1);
    cloud.points[0].x = vec[0];
    cloud.points[0].y = vec[1];
    cloud.points[0].z = vec[2];
    // Transform
    transformPointCloud (cloud, cloud, transform);
    // Convert back to eigen vector
    Eigen::Vector4f ret;
    ret[0] = cloud.points[0].x;
    ret[1] = cloud.points[0].y;
    ret[2] = cloud.points[0].z;
    ret[3] = 0;
    // Return
    return ret;
}

PointCloud<PointT> transform_eigvec_to_cloud(const Eigen::Vector4f &vec, const Eigen::Matrix4f &transform)
{
//    // Convert to cloud first
//    PointCloud<PointT> cloud = eig_to_pcl<T>(vec);
//    // Transform the point cloud
//    transformPointCloud (cloud, cloud, transform);
//    // Return
//    return cloud;

    PointCloud<PointT> p;
    p.push_back(PointT(vec[0],vec[1],vec[2]));
    transformPointCloud(p,p,transform);
    return p;
}

Eigen::Vector4f pcl_to_eig(const PointCloud<PointT> &in_cloud)
{
    Eigen::Vector4f eigen;
    // If the point cloud is not empty
    if (in_cloud.size() > 0)
    {
        // Create an eigen vector
        eigen[0] = in_cloud.points[0].x;
        eigen[1] = in_cloud.points[0].y;
        eigen[2] = in_cloud.points[0].z;
        eigen[3] = 0;
        // Print a warning if there are multiple points in the cloud
        if (in_cloud.size() > 1)
            ROS_WARN("transform_utils::pc_to_eigen : input cloud has %lu elements, returning eigen vector for first point only", in_cloud.size());
        return eigen;
    }
    else
    {
        // Create a zero eigen vector
        eigen[0] = 0;
        eigen[1] = 0;
        eigen[2] = 0;
        eigen[3] = 0;
        // Pring a warning that the point cloud is empty
        ROS_WARN("transform_utils::pc_to_eigen : input cloud is empty, returning a 0 vector");
        return eigen;
    }
}

PointCloud<PointT> eig_to_pcl(const Eigen::Vector4f &in_eigen)
{
    PointCloud<PointT> cloud;
    cloud.resize(1);
    cloud.points[0].x = in_eigen[0];
    cloud.points[0].y = in_eigen[1];
    cloud.points[0].z = in_eigen[2];
    return cloud;
}

bool downsample_point_cloud(const PointCloud<PointT> &in_cloud, PointCloud<PointT> &out_cloud)
{
    if (in_cloud.size() < _ICP_POINT_MIN)
    {
        ROS_ERROR("transform_utils::downsample_point_cloud : input cloud is too small with only %lu points", in_cloud.size());
        out_cloud = in_cloud;
        return false;
    }

    // Hack to just get a point cloud in the ball park of the right number of points
    out_cloud.clear();
    int num = 25;
    if (in_cloud.size() <= num)
    {
        out_cloud = in_cloud;
        return true;
    }
    for (int i = 0; i < num; ++i)
    {
        int r = int_rand(0, in_cloud.size());
        out_cloud.push_back(in_cloud.points[r]);
    }
    cout << "final ds points = " << out_cloud.size() << endl;
    return true;

    // First iteration assumes now that the cloud is larger than _ICP_POINT_MIN

    VoxelGrid<PointT> grid;
    double ds_res = _DOWNSAMPLE_START_RESOLUTION;
    double res_step = ds_res / 2.0;
    PointCloud<PointT>::Ptr source_copy (new PointCloud<PointT>(in_cloud));
    PointCloud<PointT>::Ptr source_ds (new PointCloud<PointT>(*source_copy));
    int num_tries = 0;
    int num_in_too_small = 0;
    double best_res = _DOWNSAMPLE_START_RESOLUTION;
    double best_diff = source_copy->size();
    while (true)
    {
        grid.setLeafSize (ds_res, ds_res, ds_res);
        grid.setInputCloud (source_copy);
        grid.filter (*source_ds);
        cout << "ds points = " << source_ds->size() << endl;
        if (source_ds->size() <= _ICP_POINT_MAX && source_ds->size() >= _ICP_POINT_MIN)
            break;

        if (source_ds->size() > _ICP_POINT_MAX)
        {
            num_in_too_small = 0;
            // Count the difference
            double diff = source_ds->size() - _ICP_POINT_MAX;
            if (diff < best_diff)
            {
                best_diff = diff;
                best_res = ds_res;
            }
            ++num_tries;
            if (num_tries == _ICP_NUM_TRIES)
            {
                cout << "reached limit" << endl;
                // Choose the best so far
                grid.setLeafSize (best_res, best_res, best_res);
                grid.setInputCloud (source_copy);
                grid.filter (*source_ds);
                break;
            }
            else
            {
                if (diff < 100)
                    res_step = res_step / 2.0;
                ds_res = ds_res + res_step;
            }
        }
        else if (source_ds->size() < _ICP_POINT_MIN)
        {
            ds_res = ds_res - res_step;
            ++num_in_too_small;
            if (num_in_too_small == _ICP_NUM_TRIES)
            {
                cout << "reached limit of too many attempts at enlarging" << endl;
                // Choose the best so far
                grid.setLeafSize (best_res, best_res, best_res);
                grid.setInputCloud (source_copy);
                grid.filter (*source_ds);
                if (source_ds->size() < _ICP_POINT_MIN)
                    source_ds = source_copy;
                break;
            }

        }
        if (res_step < 0)
            res_step = 0.01;
        if (ds_res < 0)
        {
            grid.setLeafSize (res_step, res_step, res_step);
            grid.setInputCloud (source_copy);
            grid.filter (*source_ds);
            if (source_ds->size() < _ICP_POINT_MIN)
                source_ds = source_copy;
            break;
        }
    }
    // Return the point cloud
    out_cloud = *source_ds;
    cout << "final ds points = " << out_cloud.size() << endl;
    return true;
}

///* === TEMPLATE DEFINITIONS === */
//template Eigen::Vector4f transform_eigvec(const Eigen::Vector4f &vec, const Eigen::Matrix4f &transform);
//template Eigen::Vector4d transform_eigvec(const Eigen::Vector4d &vec, const Eigen::Matrix4f &transform);
//template PointCloud<PointT> transform_eigvec_to_cloud(const Eigen::Vector4f &vec, const Eigen::Matrix4f &transform);
//template PointCloud<PointT> transform_eigvec_to_cloud(const Eigen::Vector4d &vec, const Eigen::Matrix4f &transform);
//template Eigen::Vector4f pcl_to_eig(const PointCloud<PointT> &in_cloud);
//template Eigen::Vector4d pcl_to_eig(const PointCloud<PointT> &in_cloud);
//template PointCloud<PointT> eig_to_pcl(const Eigen::Vector4f &in_eigen);
//template PointCloud<PointT> eig_to_pcl(const Eigen::Vector4d &in_eigen);
