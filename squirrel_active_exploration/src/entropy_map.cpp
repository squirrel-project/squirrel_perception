#include "squirrel_active_exploration/entropy_map.h"

using namespace std;
using namespace pcl;
using namespace octomap;

/* ****************************
   *** INSTANCE ENTROPY MAP ***
   ****************************
*/

InstanceEntropyMap::InstanceEntropyMap()
    : _combined_cloud (new PointCloud<PointT>())
{}

InstanceEntropyMap::~InstanceEntropyMap()
{
    // Clear all data
    clear();
}

bool InstanceEntropyMap::initialize(const string &training_directory, const string &class_name,
                                    const string &instance_name, const string &descriptor_name)
{
    clear();
    _training_directory = training_directory;
    _class_name = class_name;
    _instance_name = instance_name;
    _descriptor_name = descriptor_name;
    // Load the data
    if (!load_data())
    {
        ROS_ERROR("InstanceEntropyMap::initialize : error loading data");
        return false;
    }
    return true;
}

void InstanceEntropyMap::clear()
{
    _training_directory.clear();
    _class_name.clear();
    _instance_name.clear();
    _descriptor_name.clear();
    _view_indices.clear();
    _point_clouds.clear();
    _cloud_transforms.clear();
    _camera_poses.clear();
    _surface_area_proportions.clear();
    _class_entropies.clear();
    _self_probabilities.clear();
    _combined_cloud->clear();
}

bool InstanceEntropyMap::load_data()
{
    ROS_INFO("InstanceEntropyMap::load_data : loading entropy map %s ...", _instance_name.c_str());
    DIR *dp;
    struct dirent *dirp;
    string ob_train_dir = add_backslash(_training_directory) +
                          add_backslash(_class_name) +
                          add_backslash(_instance_name) +
                          add_backslash(_descriptor_name);
    ROS_INFO("InstanceEntropyMap::load_data : loading training files in %s", ob_train_dir.c_str());
    if((dp  = opendir(ob_train_dir.c_str())) == NULL)
    {
        ROS_ERROR("InstanceEntropyMap::load_data : could not open %s", ob_train_dir.c_str());
        return false;
    }

    map<int,PointCloud<PointT>::Ptr> cloud_map;
    map<int,Eigen::Matrix4f*> transform_map;
    map<int,Eigen::Vector4f*> centroid_map;
    map<int,double*> surface_area_map;
    map<int,double*> class_entropy_map;
    map<int,double*> self_probability_map;
    bool loaded_octree = false;

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            //cout << "f: " << f << endl;
            // Get the number
            size_t underscore = f.find_last_of('_');
            size_t dot = f.find_last_of('.');
            if (underscore != string::npos && dot != string::npos && (dot - underscore) > 0)
            {
                int str_len = dot - underscore;
                string str_ix = f.substr(underscore+1,str_len-1);
                int ix = atoi(str_ix.c_str());
                string filename = ob_train_dir + f;

                if (strncmp(f.c_str(),_VIEW_PREFIX, sizeof(_VIEW_PREFIX)-1) == 0)
                {
                    PointCloud<PointT>::Ptr in_cloud (new PointCloud<PointT>());
                    if (io::loadPCDFile<PointT> (filename.c_str(), *in_cloud) == -1)
                        ROS_WARN("InstanceEntropyMap::load_data : could not read point cloud file %s", filename.c_str());
                    else
                        cloud_map[ix] = in_cloud;
                    //cout << ix << " read " << in_cloud->size() << " points" << endl;
                }
                if (strncmp(f.c_str(),_TRANSFORM_PREFIX, sizeof(_TRANSFORM_PREFIX)-1) == 0)
                {
                    // Open the file
                    ifstream myfile (filename.c_str());
                    if (myfile.is_open())
                    {
                        Eigen::Matrix4f tr = Eigen::Matrix4f::Identity();
                        int i = 0, j = 0;
                        string word;
                        while (myfile >> word)
                        {
                            tr(i,j) = atof(word.c_str());
                            ++j;
                            if (j == 4)
                            {
                                ++i;
                                j = 0;
                            }
                        }
                        // NOTE these are in the incorrect direction, need to take the inverse
                        Eigen::Matrix4f tri = tr.inverse();
                        Eigen::Matrix4f *tr_ptr (new Eigen::Matrix4f(tri));
                        transform_map[ix] = tr_ptr;
                        //cout << "*tr_ptr: " << *tr_ptr << endl;
                    }
                    myfile.close();
                }
                if (strncmp(f.c_str(),_CENTROID_PREFIX, sizeof(_CENTROID_PREFIX)-1) == 0)
                {
                    // Open the file
                    ifstream myfile (filename.c_str());
                    if (myfile.is_open())
                    {
                        Eigen::Vector4f cr;
                        int i = 0;
                        string word;
                        while (myfile >> word)
                        {
                            cr[i] = atof(word.c_str());
                            ++i;
                            if (i == 3)
                                break;
                        }
                        cr[3] = 0;
                        Eigen::Vector4f *cr_ptr (new Eigen::Vector4f(cr));
                        // Re read the integer because it is different for the centroids
                        // Index is between the two underscores
                        size_t underscore1 = f.find_first_of('_');
                        size_t underscore2 = f.find_last_of('_');
                        if (underscore1 != string::npos && underscore2 != string::npos && (underscore2 - underscore1) > 0)
                        {
                            int cr_str_len = underscore2 - underscore1;
                            string cr_str_ix = f.substr(underscore1+1,cr_str_len-1);
                            int cr_ix = atoi(cr_str_ix.c_str());
                            centroid_map[cr_ix] = cr_ptr;
                        }
                    }
                    myfile.close();
                }
                if (strncmp(f.c_str(),_SURFACE_AREA_PREFIX, sizeof(_SURFACE_AREA_PREFIX)-1) == 0)
                {
                    // Open the file
                    ifstream myfile (filename.c_str());
                    if (myfile.is_open())
                    {
                        string word;
                        // Only read one number on the file
                        myfile >> word;
                        double ve = atof(word.c_str());
                        double *ve_ptr (new double(ve));
                        surface_area_map[ix] = ve_ptr;
                    }
                    myfile.close();
                }
                if (strncmp(f.c_str(),_CLASS_ENTROPY_PREFIX, sizeof(_CLASS_ENTROPY_PREFIX)-1) == 0)
                {
                    // Open the file
                    ifstream myfile (filename.c_str());
                    if (myfile.is_open())
                    {
                        string word;
                        // Only read one number on the file
                        myfile >> word;
                        double ce = atof(word.c_str());
                        double *ce_ptr (new double(ce));
                        class_entropy_map[ix] = ce_ptr;
                    }
                    myfile.close();
                }
                if (strncmp(f.c_str(),_SELF_PROB_PREFIX, sizeof(_SELF_PROB_PREFIX)-1) == 0)
                {
                    // Open the file
                    ifstream myfile (filename.c_str());
                    if (myfile.is_open())
                    {
                        string word;
                        // Only read one number on the file
                        myfile >> word;
                        double pr = atof(word.c_str());
                        double *pr_ptr (new double(pr));
                        self_probability_map[ix] = pr_ptr;
                    }
                    myfile.close();
                }
            }
            // Check if it is the tree file
            else
            {
                if (strcmp(f.c_str(), _OCTREE_FILENAME) == 0)
                {
                    // Set the string name and the flag
                    _octree_file = ob_train_dir + _OCTREE_FILENAME;
                    loaded_octree = true;
                }
                else if (strcmp(f.c_str(), _BINARY_OCTREE_FILENAME) == 0)
                {
                    // Set the string name and the flag
                    _octree_file = ob_train_dir + _BINARY_OCTREE_FILENAME;
                    loaded_octree = true;
                }
            }
        }
    }
    closedir(dp);

    // Extract the points belonging to the object in each view and store to the entropy map
    ROS_INFO("InstanceEntropyMap::load_data : extracting points for models and transforming");
    for(map<int,PointCloud<PointT>::Ptr>::iterator it = cloud_map.begin(); it != cloud_map.end(); ++it)
    {
        // it->first = key
        // it->second = value

        PointCloud<PointT>::Ptr cloud_p (new PointCloud<PointT>());
        // Transform to model frame of reference
        //Eigen::Matrix4f tr = (*transform_map[it->first]).inverse();  // transform to pose
        transformPointCloud (*it->second, *cloud_p, *transform_map[it->first]);
        _combined_cloud->insert(_combined_cloud->end(), cloud_p->begin(), cloud_p->end());

        // Store results
        int *vi_ptr (new int(it->first));
        _view_indices.push_back(vi_ptr);
        //cout << "Adding index " << it->first << endl;
        // Point cloud
        _point_clouds.push_back(it->second);  // CHANGE to store the untransformed version of the point cloud
        // Transform
        if (transform_map[it->first] != NULL)
            _cloud_transforms.push_back (transform_map[it->first]);
        else
            _cloud_transforms.push_back (NULL);
        // Centroid
        if (centroid_map[it->first] != NULL)
            _cloud_centroids.push_back (centroid_map[it->first]);
        else
            _cloud_centroids.push_back (NULL);
        // Camera pose
        Eigen::Vector4f cam_pos = extract_camera_position (*it->second, _TRAINING_RADIUS);
        // Transform to model coordinate frame
        // cam_pos = tr * cam_pos;  // not transforming the camera pose
        SensorPose *sp = new SensorPose();
        sp->origin = cam_pos;
        sp->orientation = cloud_p->sensor_orientation_;
        _camera_poses.push_back(sp);
        // Surface area proportion
        if (surface_area_map[it->first] != NULL)
            _surface_area_proportions.push_back (surface_area_map[it->first]);
        else
            _surface_area_proportions.push_back (NULL);
        // Class entropy
        if (class_entropy_map[it->first] != NULL)
            _class_entropies.push_back (class_entropy_map[it->first]);
        else
            _class_entropies.push_back (NULL);
        // Self recognition probability
        if (self_probability_map[it->first] != NULL)
            _self_probabilities.push_back (self_probability_map[it->first]);
        else
            _self_probabilities.push_back (NULL);
    }
    // If the octree was not found then create it and save it
    if (!loaded_octree)
    {
        ROS_WARN("InstanceEntropyMap::load_data : generating new octree");
        OcTree tree (_OCTREE_RESOLUTION);
        // Transform the view point and the cloud to the model frame
        //VoxelGrid<PointT> sor;
        //sor.setLeafSize ((float)_DOWNSAMPLE_FOR_OCTREE, (float)_DOWNSAMPLE_FOR_OCTREE, (float)_DOWNSAMPLE_FOR_OCTREE);
        for (vector<PointCloud<PointT>::Ptr>::size_type i = 0; i < _point_clouds.size(); ++i)
        {
            if (_point_clouds[i] && _cloud_transforms[i] && _camera_poses[i])
            {
                // Point cloud
                PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
                transformPointCloud(*_point_clouds[i], *transformed_cloud, *_cloud_transforms[i]);
                // Downsample the cloud
                //sor.setInputCloud (transformed_cloud);
                //sor.filter (*transformed_cloud);
                // Add the viewpoint to the visualizer
                PointCloud<PointT> cam_pos;
                Eigen::Vector4f v = _camera_poses[i]->origin;
                //v = *_cloud_transforms[i] * v;
                cam_pos.resize(1);
                cam_pos.points[0].x = v[0];
                cam_pos.points[0].y = v[1];
                cam_pos.points[0].z = v[2];
                transformPointCloud(cam_pos, cam_pos, *_cloud_transforms[i]);
                // Convert to octomap structures
                point3d pos (cam_pos.points[0].x, cam_pos.points[0].y, cam_pos.points[0].z);
                sensor_msgs::PointCloud2 sm_cloud;
                pcl::toROSMsg(*transformed_cloud, sm_cloud);
                Pointcloud o_cloud;
                pointCloud2ToOctomap(sm_cloud, o_cloud);
                // Add to the octree
                tree.insertPointCloud(o_cloud, pos);
            }
        }
        // Save the octomap to file
        ROS_INFO("InstanceEntropyMap::load_data : saving octree ...");
        _octree_file = ob_train_dir + _BINARY_OCTREE_FILENAME;
        tree.writeBinary(_octree_file);
        ROS_INFO("InstanceEntropyMap::load_data : saved!");
    }

    return true;
}

bool InstanceEntropyMap::is_valid_classification_data()
{
    // Query if the class entropy has been computed
    if ((_class_entropies.size() != _point_clouds.size()) || (_self_probabilities.size() != _point_clouds.size()))
        return false;
    // Check each element if it is valid or null
    bool valid = true;
    for (vector<double*>::size_type i = 0; i < _class_entropies.size(); ++i)
    {
        if (!_class_entropies[i])
        {
            valid = false;
            break;
        }
        if (!_self_probabilities[i])
        {
            valid = false;
            break;
        }
    }
    return valid;
}

bool InstanceEntropyMap::compute_classification_data(ros::ServiceClient &class_client,
                                                     squirrel_object_perception_msgs::Classify &class_srv)
{
    ROS_INFO("InstanceEntropyMap::compute_classification_data : computing the classification data for %s/%s",
             _class_name.c_str(), _instance_name.c_str());
    // Compute the class entropy for the point clouds if it has not already been computed
    if (!is_valid_classification_data())
    {
//        std::vector<pcl::PointCloud<PointT>::Ptr> _point_clouds;
//        std::vector<Eigen::Matrix4f*> _cloud_transforms;
//        std::vector<Eigen::Vector4f*> _cloud_centroids;
//        std::vector<SensorPose*> _camera_poses;
//        std::vector<double*> _visible_entropies;
//        std::vector<double*> _class_entropies;
        // Make sure _class_entropies is the same size as the views
        if (_class_entropies.size() != _point_clouds.size())
        {
            ROS_WARN("InstanceEntropyMap::compute_classification_data : _class_entropies has %lu elements and _point_clouds has %lu elements",
                     _class_entropies.size(), _point_clouds.size());
            _class_entropies.clear();
            _self_probabilities.clear();
            for (vector<PointCloud<PointT>::Ptr>::size_type i = 0; i < _point_clouds.size(); ++i)
            {
                _class_entropies.push_back (NULL);
                _self_probabilities.push_back(NULL);
            }
        }
        // Name of class without backslash
        string self_name = rem_backslash(_class_name);
        // For writing
        string dir = add_backslash(_training_directory) +
                     add_backslash(_class_name) +
                     add_backslash(_instance_name) +
                     add_backslash(_descriptor_name);
        string f;
        // For each view
        for (vector<PointCloud<PointT>::Ptr>::size_type i = 0; i < _point_clouds.size(); ++i)
        {
            ROS_INFO("InstanceEntropyMap::compute_classification_data : computing classification data for %s/%s/view_%lu",
                     _class_name.c_str(), _instance_name.c_str(), i);
            // Pass the data to the message
            sensor_msgs::PointCloud2 r_cloud;
            toROSMsg(*_point_clouds[i], r_cloud);
            class_srv.request.cloud = r_cloud;
            std_msgs::Int32MultiArray object_indices;
            for (int j = 0; j < _point_clouds[i]->size(); ++j)
                object_indices.data.push_back(j);
            vector<std_msgs::Int32MultiArray> segment;
            segment.push_back(object_indices);
            class_srv.request.clusters_indices = segment;
            // Call the service
            if (class_client.call(class_srv))
            {
                ROS_INFO("InstanceEntropyMap::compute_classification_data : successfully classified the point cloud");
                squirrel_object_perception_msgs::Classification class_estimate = class_srv.response.class_results[0];
                // Get the class result
                double prob = 0;
                for (size_t j = 0; j < class_estimate.class_type.size(); ++j)
                {
                    if (strcmp(rem_backslash(class_estimate.class_type[j].data).c_str(),self_name.c_str()) == 0)
                        prob = class_estimate.confidence[j];
                }
                double *prob_ptr = &prob;
                _self_probabilities[i] = prob_ptr;
                // Compute the entropy
                double ent = (double)entropy(class_estimate.confidence);
                double *ent_ptr = &ent;
                _class_entropies[i] = ent_ptr;
                ROS_INFO("InstanceEntropyMap::compute_classification_data : successfully computed the entropy");
                // Write to file
                f = dir + _SELF_PROB_PREFIX + boost::lexical_cast<string>(i) + ".txt";
                if (write_to_file(f, prob))
                    ROS_INFO("InstanceEntropyMap::compute_classification_data : wrote file %s", f.c_str());
                else
                    ROS_WARN("InstanceEntropyMap::compute_classification_data : error writing to file %s", f.c_str());
                f = dir + _CLASS_ENTROPY_PREFIX + boost::lexical_cast<string>(i) + ".txt";
                if (write_to_file(f, ent))
                    ROS_INFO("InstanceEntropyMap::compute_classification_data : wrote file %s", f.c_str());
                else
                    ROS_WARN("InstanceEntropyMap::compute_classification_data : error writing to file %s", f.c_str());
            }
            else
            {
                ROS_ERROR("InstanceEntropyMap::compute_classification_data : could not call the classification service for %s/%s/view_%lu",
                          _class_name.c_str(), _instance_name.c_str(), i);
            }
        }
    }
    else
    {
        ROS_INFO("InstanceEntropyMap::compute_classification_data : all valid class entropies for views in for %s/%s",
                 _class_name.c_str(), _instance_name.c_str());
    }
    return true;
}

bool InstanceEntropyMap::visualize(const int &view_ix, const string &score_type)
{
    ROS_INFO("InstanceEntropyMap::visualize : viewing %s %s %i", _class_name.c_str(), _instance_name.c_str(), view_ix);
    string title = "Entropy Map " + _class_name + " " + _instance_name;
    visualization::PCLVisualizer *viewer  = new visualization::PCLVisualizer(title);
    //viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // Define R,G,B colors for the point cloud
    visualization::PointCloudColorHandlerCustom<PointT> combined_handler (_combined_cloud, 255, 255, 255);  // White
    // Add the point cloud to the viewer and pass the color handler
    viewer->addPointCloud (_combined_cloud, combined_handler, "combined_cloud");
    // Set the point cloud properties
    viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 2, "combined_cloud");
    // Threads to simultaneously 1) keep viewer open and 2) display cloud and wait for next input to update the clouds
    boost::thread view_thread = boost::thread (&InstanceEntropyMap::run_viewer, this, viewer);
    boost::thread cloud_update_thread = boost::thread(&InstanceEntropyMap::run_view_instance, this, viewer, view_ix, score_type);
    // Join the threads
    view_thread.join();
    cloud_update_thread.join();
    // Finished, wait for thread to exit
    ROS_INFO("InstanceEntropyMap::visualize : finished viewing all point clouds of %s/%s",_class_name.c_str(), _instance_name.c_str());
    // Close the window
    delete viewer;
    return true;
}

/* === GETTERS === */

string InstanceEntropyMap::get_training_directory()
{
    return _training_directory;
}

string InstanceEntropyMap::get_training_directory() const
{
    return _training_directory;
}

string InstanceEntropyMap::get_class_name()
{
    return _class_name;
}

string InstanceEntropyMap::get_class_name() const
{
    return _class_name;
}

string InstanceEntropyMap::get_instance_name()
{
    return _instance_name;
}

string InstanceEntropyMap::get_instance_name() const
{
    return _instance_name;
}

string InstanceEntropyMap::get_descriptor_name()
{
    return _descriptor_name;
}

string InstanceEntropyMap::get_descriptor_name() const
{
    return _descriptor_name;
}

vector<int*> InstanceEntropyMap::get_view_indices()
{
    return _view_indices;
}

vector<int*> InstanceEntropyMap::get_view_indices() const
{
    return _view_indices;
}

vector<PointCloud<PointT>::Ptr> InstanceEntropyMap::get_point_clouds()
{
    return _point_clouds;
}

vector<PointCloud<PointT>::Ptr> InstanceEntropyMap::get_point_clouds() const
{
    return _point_clouds;
}

vector<Eigen::Matrix4f*> InstanceEntropyMap::get_cloud_transforms()
{
    return _cloud_transforms;
}

vector<Eigen::Matrix4f*> InstanceEntropyMap::get_cloud_transforms() const
{
    return _cloud_transforms;
}

vector<Eigen::Vector4f*> InstanceEntropyMap::get_cloud_centroids()
{
    return _cloud_centroids;
}

vector<Eigen::Vector4f*> InstanceEntropyMap::get_cloud_centroids() const
{
    return _cloud_centroids;
}

vector<SensorPose*> InstanceEntropyMap::get_camera_poses()
{
    return _camera_poses;
}

vector<SensorPose*> InstanceEntropyMap::get_camera_poses() const
{
    return _camera_poses;
}

vector<double*> InstanceEntropyMap::get_surface_area_proportions()
{
    return _surface_area_proportions;
}

vector<double*> InstanceEntropyMap::get_surface_area_proportions() const
{
    return _surface_area_proportions;
}

vector<double*> InstanceEntropyMap::get_class_entropies()
{
    return _class_entropies;
}

vector<double*> InstanceEntropyMap::get_class_entropies() const
{
    return _class_entropies;
}

vector<double*> InstanceEntropyMap::get_self_probabilities()
{
    return _self_probabilities;
}

vector<double*> InstanceEntropyMap::get_self_probabilities() const
{
    return _self_probabilities;
}

PointCloud<PointT>::Ptr InstanceEntropyMap::get_combined_cloud()
{
    return _combined_cloud;
}

PointCloud<PointT>::Ptr InstanceEntropyMap::get_combined_cloud() const
{
    return _combined_cloud;
}

string InstanceEntropyMap::get_octree_file()
{
    return _octree_file;
}

string InstanceEntropyMap::get_octree_file() const
{
    return _octree_file;
}

size_t InstanceEntropyMap::size()
{
    return _view_indices.size();
}

size_t InstanceEntropyMap::size() const
{
    return _view_indices.size();
}

/* === PRIVATE FUNCTIONS === */

void InstanceEntropyMap::run_viewer(visualization::PCLVisualizer *viewer)
{
    ROS_INFO("InstanceEntropyMap::run_viewer : starting");
    // Display the visualiser until 'q' key is pressed
    while (!viewer->wasStopped ())
        viewer->spinOnce ();
    ROS_INFO("InstanceEntropyMap::run_viewer : finished");
}

void InstanceEntropyMap::run_view_instance(visualization::PCLVisualizer *viewer, const int &view_index, const string &score_type)
{
    ROS_INFO("InstanceEntropyMap::run_view_instance : running view cloud thread");
    // If in the input view index is less than 1 then display all views one at a time
    if (view_index < 0)
    {
        ROS_INFO("InstanceEntropyMap::run_view_instance : displaying all views");
        // Display the point clouds one at a time
        string f, g;
        char input;
        for (vector<PointCloud<PointT>::Ptr>::size_type i = 0; i < _point_clouds.size(); ++i)
        {
            f = "viewpoint_cloud_" + boost::lexical_cast<string>(i);
            g = "camera_position_" + boost::lexical_cast<string>(i);
            PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
            transformPointCloud(*_point_clouds[i], *transformed_cloud, *_cloud_transforms[i]);
            visualization::PointCloudColorHandlerCustom<PointT> single_view_handler (transformed_cloud, 230, 20, 20); // Red
            viewer->addPointCloud (transformed_cloud, single_view_handler, f);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, f);
            // Add the viewpoint to the visualizer
            PointCloud<PointT>::Ptr cam_pos (new PointCloud<PointT>());
            Eigen::Vector4f v = _camera_poses[i]->origin;
            //v = *_cloud_transforms[i] * v;
            cam_pos->resize(1);
            cam_pos->points[0].x = v[0];
            cam_pos->points[0].y = v[1];
            cam_pos->points[0].z = v[2];
            transformPointCloud(*cam_pos, *cam_pos, *_cloud_transforms[i]);
            visualization::PointCloudColorHandlerCustom<PointT> cam_pos_handler (cam_pos, 20, 20, 230);  // Blue
            viewer->addPointCloud (cam_pos, cam_pos_handler, g);
            viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 8, g);

            cout << "Camera position : " << v[0] << " " << v[1] << " " << v[2] << endl;
            cout << "Surface area : ";
            if (_surface_area_proportions[i])
                cout << *_surface_area_proportions[i] << endl;
            else
                cout << "NULL" << endl;
            cout << "Class entropy : ";
            if (_class_entropies[i])
                cout << *_class_entropies[i] << endl;
            else
                cout << "NULL" << endl;
            cout << "Self recognition : ";
            if (_self_probabilities[i])
                cout << *_self_probabilities[i] << endl;
            else
                cout << "NULL" << endl;
            cout << "Enter for next view or q to quit ...";
            input = cin.get();
            if (input == 'q' || input == 'Q')
            {
                ROS_WARN("InstanceEntropyMap::run_view_instance : exiting loop after %lu/%lu viewpoints", i+1, _point_clouds.size());
                break;
            }
        }
    }
    // Otherwise just display the single view
    else
    {
        // If the view index is larger than the available views then return an error
        int num_clouds = _point_clouds.size();
        if (view_index > num_clouds)
        {
            ROS_ERROR("InstanceEntropyMap::run_view_instance : requested view index %u is too large for the views %lu",
                      view_index, _point_clouds.size());
            return;
        }
        ROS_INFO("InstanceEntropyMap::run_view_instance : displaying view point %u", view_index);
        PointCloud<PointT>::Ptr all_views (new PointCloud<PointT>());
        for (vector<PointCloud<PointT>::Ptr>::size_type i = 0; i < _camera_poses.size(); ++i)
        {
            // Add the viewpoint to the visualizer
            PointCloud<PointT> cam_pos;
            Eigen::Vector4f v = _camera_poses[i]->origin;
            //v = *_cloud_transforms[i] * v;
            cam_pos.resize(1);
            cam_pos.points[0].x = v[0];
            cam_pos.points[0].y = v[1];
            cam_pos.points[0].z = v[2];
            transformPointCloud(cam_pos, cam_pos, *_cloud_transforms[i]);
            all_views->push_back(cam_pos.points[0]);
        }
        // Display all views
        visualization::PointCloudColorHandlerCustom<PointT> cam_pos_handler (all_views, 20, 20, 230);  // Blue
        viewer->addPointCloud (all_views, cam_pos_handler, "view_points");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 8, "view_points");
        // Display the requested point cloud
        PointCloud<PointT>::Ptr transformed_cloud (new PointCloud<PointT>());
        transformPointCloud(*_point_clouds[view_index], *transformed_cloud, *_cloud_transforms[view_index]);
        visualization::PointCloudColorHandlerCustom<PointT> single_view_handler (transformed_cloud, 230, 20, 20); // Red
        viewer->addPointCloud (transformed_cloud, single_view_handler, "view_point_cloud");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 4, "view_point_cloud");
        // Display the requested view
        PointCloud<PointT>::Ptr cam_cloud (new PointCloud<PointT>());
        cam_cloud->resize(1);
        Eigen::Vector4f v = _camera_poses[view_index]->origin;
        cam_cloud->resize(1);
        cam_cloud->points[0].x = v[0];
        cam_cloud->points[0].y = v[1];
        cam_cloud->points[0].z = v[2];
        transformPointCloud(*cam_cloud, *cam_cloud, *_cloud_transforms[view_index]);
        visualization::PointCloudColorHandlerCustom<PointT> cam_cloud_handler (cam_cloud, 20, 230, 20); // Green
        viewer->addPointCloud (cam_cloud, cam_cloud_handler, "camera_position");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 10, "camera_position");
        // Display the location with the highest surface area or lowest entropy (best view)
        vector<double> scores;
        scores.resize(_surface_area_proportions.size());
        for (vector<double*>::size_type i = 0; i < _surface_area_proportions.size(); ++i)
        {
            scores[i] = *_surface_area_proportions[i];
            if (strcmp(score_type.c_str(), "entropy") == 0 || strcmp(score_type.c_str(), "ENTROPY") == 0 ||
                strcmp(score_type.c_str(), "class") == 0 || strcmp(score_type.c_str(), "CLASS") == 0 ||
                strcmp(score_type.c_str(), "class_entropy") == 0 || strcmp(score_type.c_str(), "CLASS_ENTROPY") == 0)
            {
                // Check that the entropies are valid
                if (_class_entropies[i])
                    scores[i] = 1.0/(*_class_entropies[i]);
                else
                    ROS_WARN("InstanceEntropyMap::run_view_instance : _class_entropy[%lu] is not available", i);
            }
            else if (strcmp(score_type.c_str(), "self") == 0 || strcmp(score_type.c_str(), "SELF") == 0 ||
                     strcmp(score_type.c_str(), "probability") == 0 || strcmp(score_type.c_str(), "PROBABILITY") == 0 ||
                     strcmp(score_type.c_str(), "prob") == 0 || strcmp(score_type.c_str(), "PROB") == 0 ||
                     strcmp(score_type.c_str(), "self_probability") == 0 || strcmp(score_type.c_str(), "SELF_PROBABILITY") == 0 ||
                     strcmp(score_type.c_str(), "self_prob") == 0 || strcmp(score_type.c_str(), "SELF_PROB") == 0)
                 {
                     // Check that the probabilities are valid
                     if (_self_probabilities[i])
                         scores[i] = *_self_probabilities[i];
                     else
                         ROS_WARN("InstanceEntropyMap::run_view_instance : _self_probabilities[%lu] is not available", i);
                 }
        }
        int best_index = max_element(scores.begin(), scores.end()) - scores.begin();
        PointCloud<PointT>::Ptr best_cloud (new PointCloud<PointT>());
        best_cloud->resize(1);
        v = _camera_poses[best_index]->origin;
//        for (int j = 0; j < scores.size(); ++j)
//            cout << j << " -> " << scores[j] << endl;
        ROS_INFO("Index for best score is %i", best_index);
        ROS_INFO("Original location is [%.2f %.2f %.2f] ",v[0], v[1], v[2]);
        best_cloud->resize(1);
        best_cloud->points[0].x = v[0];
        best_cloud->points[0].y = v[1];
        best_cloud->points[0].z = v[2];
        transformPointCloud(*best_cloud, *best_cloud, *_cloud_transforms[best_index]);
        visualization::PointCloudColorHandlerCustom<PointT> best_cloud_handler (best_cloud, 230, 20, 230); // Magenta
        viewer->addPointCloud (best_cloud, best_cloud_handler, "best_camera_position");
        viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 11, "best_camera_position");
    }
    ROS_INFO("InstanceEntropyMap::run_view_instance: exiting view cloud thread");
}




/* *************************
   *** CLASS ENTROPY MAP ***
   *************************
*/

ClassEntropyMap::ClassEntropyMap()
{}

ClassEntropyMap::~ClassEntropyMap()
{
    clear();
}

bool ClassEntropyMap::initialize(const string &training_directory, const string &class_name, const string &descriptor_name)
{
    clear();
    _training_directory = training_directory;
    _class_name = class_name;
    _descriptor_name = descriptor_name;
    // Load the data
    if (!load_data())
    {
        ROS_ERROR("ClassEntropyMap::initialize : error loading data");
        return false;
    }
    return true;
}

void ClassEntropyMap::clear()
{
    _training_directory.clear();
    _class_name.clear();
    _descriptor_name.clear();
    _instance_maps.clear();
}

bool ClassEntropyMap::load_data()
{
    // Look up all the files in the training directory
    ROS_INFO("ClassEntropyMap::initialize : loading all instance entropy maps for class %s ...", _class_name.c_str());
    DIR *dp;
    struct dirent *dirp;
    string ob_train_dir = add_backslash(_training_directory) +
                          add_backslash(_class_name);
    ROS_INFO("ClassEntropyMap::initialize : loading training files in %s", ob_train_dir.c_str());
    if((dp  = opendir(ob_train_dir.c_str())) == NULL)
    {
        ROS_ERROR("ClassEntropyMap::initialize : could not open %s", ob_train_dir.c_str());
        return false;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            //cout << f << endl;
            // Create a new instance entropy map
            InstanceEntropyMap *iem = new InstanceEntropyMap();
            if (iem->initialize(_training_directory, _class_name, f, _descriptor_name))
            {
                // Add to the map
                _instance_maps[f] = iem;
            }
            else
            {
                // Could not initialize the object
                ROS_WARN("ClassEntropyMap::initialize : could not initialize instance in directory %s", f.c_str());
            }
        }
    }

    return true;
}

InstanceEntropyMap* ClassEntropyMap::extract_map(const string &key)
{
    // Remove the path if it exists
    string str = rem_backslash (key);  // remove the backslash if it exists
    size_t found = str.find_last_of("\\/");
    string search_key = str;
    if (found != string::npos)
        search_key = str.substr (found, str.length());
    ROS_INFO("ClassEntropyMap::extract_map :");
    ROS_INFO("   key %s", key.c_str());
    ROS_INFO("   str %s", str.c_str());
    ROS_INFO("   search_key %s", search_key.c_str());

    // Look up in the instance map
    return _instance_maps[search_key];
}

bool ClassEntropyMap::compute_classification_data(ros::ServiceClient &class_client, squirrel_object_perception_msgs::Classify &class_srv)
{
    // Classify each view in the instances for this class
    ROS_INFO("ClassEntropyMap::compute_classification_data : computing the class entropy for class %s ...", _class_name.c_str());
    // For each instance
    for(map<string,InstanceEntropyMap*>::iterator it = _instance_maps.begin(); it != _instance_maps.end(); ++it)
    {
        if (!it->second->compute_classification_data(class_client, class_srv))
            ROS_WARN("ClassEntropyMap::compute_classification_data : could not compute class entropy for instance %s",
                     it->second->get_instance_name().c_str());
    }
    return true;
}

/* === GETTERS === */

string ClassEntropyMap::get_training_directory()
{
    return _training_directory;
}

string ClassEntropyMap::get_training_directory() const
{
    return _training_directory;
}

string ClassEntropyMap::get_class_name()
{
    return _class_name;
}

string ClassEntropyMap::get_class_name() const
{
    return _class_name;
}

string ClassEntropyMap::get_descriptor_name()
{
    return _descriptor_name;
}

string ClassEntropyMap::get_descriptor_name() const
{
    return _descriptor_name;
}

map<string,InstanceEntropyMap*> ClassEntropyMap::get_instance_maps()
{
    return _instance_maps;
}

map<string,InstanceEntropyMap*> ClassEntropyMap::get_instance_maps() const
{
    return _instance_maps;
}




/* *******************
   *** ENTROPY MAP ***
   *******************
*/

EntropyMap::EntropyMap() : _do_classification (false), _do_inspect (false)
{}

EntropyMap::~EntropyMap()
{
    // Delete the ros node handle
    if (_n)
        delete _n;
    // Clear the data
    clear();
}

bool EntropyMap::initialize(int argc, char **argv)
{
    ROS_INFO("EntropyMap::initialize : starting");
    ros::init(argc, argv, "entropy_map_service");
    ros::NodeHandle *node (new ros::NodeHandle("~"));
    clear();
    initialize (node);

    ROS_INFO("EntropyMap::initialize : finished");
    return true;
}

bool EntropyMap::initialize(ros::NodeHandle *node)
{
    _n = node;

    // Subscribe to service
    _class_client = _n->serviceClient<squirrel_object_perception_msgs::Classify>("/squirrel_classify");

    // Read the input if it exists
    _n->getParam ( "training_directory", _training_directory);
    _n->getParam ( "descriptor", _descriptor_name);
    _n->getParam ( "classification", _do_classification);
    _n->getParam ( "inspect", _do_inspect);

    ROS_INFO("Input : ");
    ROS_INFO("_training_directory: %s", _training_directory.c_str());
    ROS_INFO("_descriptor_name:    %s", _descriptor_name.c_str());
    if (_do_classification)
        ROS_INFO("_do_classification:  TRUE");
    else
        ROS_INFO("_do_classification:  FALSE");
    if (_do_inspect)
        ROS_INFO("_do_inspect:         TRUE");
    else
        ROS_INFO("_do_inspect:         FALSE");

    // Load the data
    if (!load_data())
    {
        ROS_ERROR("EntropyMap::initialize : error loading data");
        return false;
    }

    // Offer services
    _service_entropy_map = _n->advertiseService("/squirrel_entropy_map", &EntropyMap::extract_entropy_map, this);
    _service_entropy_map_visualize = _n->advertiseService("/squirrel_entropy_map_visualize", &EntropyMap::visualize_entropy_map, this);
    ROS_INFO("EntropyMap : ready to receive service calls...");
    ros::spin();

    return true;
}

void EntropyMap::clear()
{
    _training_directory.clear();
    _descriptor_name.clear();
    _class_maps.clear();
}

bool EntropyMap::load_data()
{
    // Look up all the files in the training directory
    ROS_INFO("EntropyMap::load_data : loading all class entropy maps in directory %s ...", _training_directory.c_str());
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(_training_directory.c_str())) == NULL)
    {
        ROS_ERROR("EntropyMap::load_data : could not open %s", _training_directory.c_str());
        return false;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        string f = string(dirp->d_name);
        if (strcmp(f.c_str(),".") != 0 && strcmp(f.c_str(),"..") != 0)
        {
            //cout << "class " << f << endl;
            // Create a new class entropy map
            ClassEntropyMap *cem = new ClassEntropyMap();
            if (cem->initialize(_training_directory, f, _descriptor_name))
            {
                // Add to the map
                _class_maps[f] = cem;
            }
            else
            {
                // Could not initialize the object
                ROS_WARN("EntropyMap::load_data : could not initialize class in directory %s", f.c_str());
            }
        }
    }

    // Compute the class entropies
    if (_do_classification)
    {
        if (!compute_classification_data())
        {
            ROS_ERROR("EntropyMap::load_data : could not compute the class entropy");
            return false;
        }
    }

    // Inspect
    if (_do_inspect)
        inspect();

    ROS_INFO("EntropyMap::load_data : finished");
    return true;
}

ClassEntropyMap* EntropyMap::extract_class_map(const string &key)
{
    // Remove the path if it exists
    string str = rem_backslash (key);  // remove the backslash if it exists
    size_t found = str.find_last_of("\\/");
    string search_key = str;
    if (found != string::npos)
        search_key = str.substr (found, str.length());
    // Print
    ROS_INFO("EntropyMap::extract_class_map :");
    ROS_INFO("   key %s", key.c_str());
    ROS_INFO("   str %s", str.c_str());
    ROS_INFO("   search_key %s", search_key.c_str());

    // Look up in the instance map
    return _class_maps[search_key];
}

InstanceEntropyMap* EntropyMap::extract_instance_map(const string &key)
{
    InstanceEntropyMap *iem = NULL;
    // Key must be an absolute path that contains
    // path/class/instance
    string str = rem_backslash (key); // remove the backslash if it exists
    // Instance name is the string after the last back slash
    size_t found = str.find_last_of("\\/");
    if (found == string::npos)
    {
        ROS_ERROR("EntropyMap::extract_instance_map : could not get instance name in path %s", key.c_str());
        return iem;
    }
    string instance_name = str.substr (found, str.length());
    // Class name is the string between the second last and the last backslash
    string str2 = str.substr (0, found);
    found = str2.find_last_of("\\/");
    if (found == string::npos)
    {
        ROS_ERROR("EntropyMap::extract_instance_map : could not get class name in path %s", key.c_str());
        return iem;
    }
    string class_name = str2.substr (found, str2.length());
    // Print
    ROS_INFO("EntropyMap::extract_instance_map :");
    ROS_INFO("   key %s", key.c_str());
    ROS_INFO("   instance_name %s", instance_name.c_str());
    ROS_INFO("   class_name %s", class_name.c_str());

    // Lookup the class and instance
    return extract_instance_map (class_name, instance_name);
}

InstanceEntropyMap* EntropyMap::extract_instance_map(const string &class_key, const string &instance_key)
{
    // Remove the back slashes
    string class_key2 = rem_backslash(class_key);
    string instance_key2 = rem_backslash(instance_key);
    ROS_INFO("EntropyMap::extract_instance_map : looking up class %s and instance %s", class_key2.c_str(), instance_key2.c_str());
    ClassEntropyMap *cmap = _class_maps[class_key2];
    if (cmap != NULL)
    {
        return cmap->extract_map(instance_key2);
    }
    else
    {
        ROS_WARN("EntropyMap::extract_instance_map : returning NULL instance entropy map");
        return NULL;
    }
}

bool EntropyMap::compute_classification_data()
{
    // Classify each view for each class
    ROS_INFO("EntropyMap::extract_instance_map : starting");
    // For each class
    for(map<string,ClassEntropyMap*>::iterator it = _class_maps.begin(); it != _class_maps.end(); ++it)
    {
        if (!it->second->compute_classification_data(_class_client, _class_srv))
            ROS_WARN("EntropyMap::extract_instance_map : could not compute class entropy for class %s", it->second->get_class_name().c_str());
    }

    ROS_INFO("EntropyMap::extract_instance_map : finished");
    return true;
}

void EntropyMap::inspect()
{
    ROS_INFO("EntropyMap::inspect : starting");
    // For each class
    for(map<string,ClassEntropyMap*>::iterator c_it = _class_maps.begin(); c_it != _class_maps.end(); ++c_it)
    {
        ROS_INFO("Inspecting class %s", c_it->first.c_str());
        // Get the ClassEntropyMap
        ClassEntropyMap *cem = c_it->second;
        // Check if it is NULL
        if (!cem)
            ROS_WARN("EntropyMap::inspect : class entropy map for %s is NULL", c_it->first.c_str());
        // Otherwise inspect the instances in this class
        map<string,InstanceEntropyMap*> imaps = cem->get_instance_maps();
        for(map<string,InstanceEntropyMap*>::iterator i_it = imaps.begin(); i_it != imaps.end(); ++i_it)
        {
            ROS_INFO("EntropyMap::inspect : instance %s", i_it->first.c_str());
            // Inspect the instance map
            InstanceEntropyMap *iem = i_it->second;
            // Check if it is NULL
            if (!iem)
                ROS_WARN("EntropyMap::inspect : instance entropy map for %s/%s is NULL", c_it->first.c_str(), i_it->first.c_str());
            // Otherwise inspect the instance
            vector<pcl::PointCloud<PointT>::Ptr> point_clouds = iem->get_point_clouds();
            vector<Eigen::Matrix4f*> cloud_transforms = iem->get_cloud_transforms();
            vector<Eigen::Vector4f*> cloud_centroids = iem->get_cloud_centroids();
            vector<SensorPose*> camera_poses = iem->get_camera_poses();
            vector<double*> surface_areas = iem->get_surface_area_proportions();
            vector<double*> class_entropies = iem->get_class_entropies();
            vector<double*> self_probabilities = iem->get_self_probabilities();
            ROS_INFO("Point clouds      %lu", point_clouds.size());
            ROS_INFO("Cloud transforms  %lu", cloud_transforms.size());
            ROS_INFO("Centroids         %lu", cloud_centroids.size());
            ROS_INFO("Camera poses      %lu", camera_poses.size());
            ROS_INFO("Surface areas     %lu", surface_areas.size());
            ROS_INFO("Class entropies   %lu", class_entropies.size());
            ROS_INFO("Probabilities     %lu", self_probabilities.size());
            for (vector<pcl::PointCloud<PointT>::Ptr>::size_type i = 0; i < point_clouds.size(); ++i)
            {
                // Number of points
                cout << i << " : pts ";
                if (point_clouds[i])
                    cout << point_clouds[i]->size();
                else
                    cout << "N";
                // Diagonals of transform
                cout << " | tr ";
                if (cloud_transforms[i])
                    cout << "[" << (*cloud_transforms[i])(0,0) << "," << (*cloud_transforms[i])(1,1)<< ","
                         << (*cloud_transforms[i])(2,2) << "," << (*cloud_transforms[i])(3,3) << "]";
                else
                    cout << "N";
                // Centroid
                cout << " | ctr ";
                if (cloud_centroids[i])
                    cout << "[" << (*cloud_centroids[i])[0] << "," << (*cloud_centroids[i])[1] << ","
                         << (*cloud_centroids[i])[2] << "]";
                else
                    cout << "N";
                // Camera pose
                cout << " | cam ";
                if (camera_poses[i])
                    cout << "[" << (*camera_poses[i]).origin[0] << "," << (*camera_poses[i]).origin[1]  << ","
                         << (*camera_poses[i]).origin[2]  << "]";
                else
                    cout << "N";
                // Surface area
                cout << " | area ";
                if (surface_areas[i])
                    cout << *surface_areas[i];
                else
                    cout << "N";
                // Class entropy
                cout << " | ent ";
                if (class_entropies[i])
                    cout << *class_entropies[i];
                else
                    cout << "N";
                // Probability
                cout << " | prob ";
                if (self_probabilities[i])
                    cout << *self_probabilities[i];
                else
                    cout << "N";
                cout << endl;
            }


        }
        //cout << "... next class ... ";
        //cin.ignore();
    }

    ROS_INFO("EntropyMap::inspect : finished");
}

/* === VISUALIZE === */

//void EntropyMap::visualize(const int &class_ix, const int &instance_ix, const int &view_ix, const string &entropy_type)
//{
//    int class_request = class_ix;
//    int instance_request = instance_ix;
//    int view_request = view_ix;
//    // Print outs to know what will be visualized
//    if (class_ix < 0)
//        ROS_INFO("EntropyMap::visualize : visualizing all classes");
//    else
//        ROS_INFO("EntropyMap::visualize : visualizing class %u", class_ix);
//    if (instance_ix < 0)
//        ROS_INFO("EntropyMap::visualize : visualizing all instances");
//    else
//        ROS_INFO("EntropyMap::visualize : visualizing instance %u", instance_ix);
//    if (view_ix < 0)
//        ROS_INFO("EntropyMap::visualize : visualizing all views");
//    else
//        ROS_INFO("EntropyMap::visualize : visualizing view %u", view_ix);
//    // Warnings
//    if (class_ix >= _class_maps.size())
//    {
//        ROS_WARN("EntropyMap::visualize : class_ix %u is larger than the elements in _class_maps %lu", class_ix, _class_maps.size());
//        class_request = _class_maps.size()-1;
//    }
//    if (class_ix >= 0 && instance_ix >= 0)
//    {
//        int c_count = 0;
//        // Iterate through the class maps until the particular class is found
//        for (map<string,ClassEntropyMap*>::iterator c_it = _class_maps.begin(); c_it != _class_maps.end(); ++c_it)
//        {
//            if (c_count == class_ix)
//            {
//                ClassEntropyMap* cem = c_it->second;
//                if (cem)
//                {
//                    int num = cem->get_instance_maps().size();
//                    if (instance_ix >= num)
//                    {
//                        ROS_WARN("EntropyMap::visualize : instance_ix %u is larger than the elements in %s's _instance_maps %u",
//                                 instance_ix, c_it->first.c_str(), num);
//                        instance_request = num-1;
//                    }
//                    // Check the view
//                    if (view_ix >= 0)
//                    {
//                        int i_count = 0;
//                        // Iterate through the instance maps until the particular instance is found
//                        for (map<string,InstanceEntropyMap*>::iterator i_it = cem->get_instance_maps().begin(); i_it != cem->get_instance_maps().end(); ++i_it)
//                        {
//                            if (i_count == instance_ix)
//                            {
//                                if (i_it->second)
//                                {
//                                    num = i_it->second->get_view_indices().size();
//                                    if (view_ix >= num)
//                                    {
//                                        ROS_WARN("EntropyMap::visualize : view_ix %u is larger than the elements in %s/%s's _view_indices %u",
//                                                 view_ix, c_it->first.c_str(), i_it->first.c_str(), num);
//                                        view_request = num-1;
//                                    }
//                                }
//                                break;
//                            }
//                            ++i_count;
//                        }
//                    }
//                }
//                break;
//            }
//            ++c_count;
//        }
//    }

//    // Visualize
//    int class_count = 0;
//    for (map<string,ClassEntropyMap*>::iterator c_it = _class_maps.begin(); c_it != _class_maps.end(); ++c_it)
//    {
//        bool vis_class = false;
//        if (class_request < 0)
//            vis_class = true;
//        else if (class_count == class_request)
//            vis_class = true;
//        // If this is a class to visualize
//        if (vis_class)
//        {
//            // Get the class entropy map
//            ClassEntropyMap *cem = c_it->second;
//            if (cem)
//            {
//                int inst_count = 0;
//                map<string,InstanceEntropyMap*> imaps = cem->get_instance_maps();
//                for(map<string,InstanceEntropyMap*>::iterator i_it = imaps.begin(); i_it != imaps.end(); ++i_it)
//                {
//                    bool vis_inst = false;
//                    if (instance_request < 0)
//                        vis_inst = true;
//                    else if (inst_count == instance_request)
//                        vis_inst = true;
//                    // If this is an instance to visualize
//                    if (vis_inst)
//                    {
//                        // Get the instance entropy map
//                        InstanceEntropyMap *iem = i_it->second;
//                        if (iem)
//                        {
//                            if (!iem->visualize(view_request, entropy_type))
//                            {
//                                ROS_ERROR("EntropyMap::visualize : exiting");
//                                return;
//                            }
//                        }
//                        else
//                        {
//                            ROS_WARN("EntropyMap::visualize : instance entropy map %s is NULL", i_it->first.c_str());
//                        }
//                    }
//                    ++inst_count;
//                }
//            }
//            else
//            {
//                ROS_WARN("EntropyMap::visualize : class entropy map %s is NULL", c_it->first.c_str());
//            }
//        }
//        ++class_count;
//    }
//}

//void EntropyMap::visualize(const string &class_name, const string &instance_name, const int &view_index, const string &entropy_type)
//{
//    int class_ix = -1;
//    int instance_ix = -1;
//    int view_ix = view_index;
//    // Find the indices of the requested class name and instance name
//    if (class_name.size() > 0)
//    {
//        bool found = false;
//        int count = 0;
//        for (map<string,ClassEntropyMap*>::iterator it = _class_maps.begin(); it != _class_maps.end(); ++it)
//        {
//            if (strcmp(it->first.c_str(),class_name.c_str()) == 0)
//            {
//                class_ix = count;
//                found = true;
//                break;
//            }
//            ++count;
//        }
//        // If not found
//        if (!found)
//            ROS_WARN("EntropyMap::visualize : cannot find class name %s", class_name.c_str());
//    }
//    else
//    {
//        ROS_WARN("EntropyMap::visualize : empty class name");
//    }
//    if (instance_name.size() > 0)
//    {
//        if (class_ix >= 0)
//        {
//            ClassEntropyMap *cem = _class_maps[class_name];
//            if (cem)
//            {
//                bool found = false;
//                int count = 0;
//                cout << "Number of instances " << cem->get_instance_maps().size() << endl;
//                for (map<string,InstanceEntropyMap*>::iterator it = cem->get_instance_maps().begin(); it != cem->get_instance_maps().end(); ++it)
//                {
//                    cout << "count " << count << endl;
//                    cout << "comparing " << it->first << " " << instance_name << endl;
////                    if (strcmp(it->first.c_str(),instance_name.c_str()) == 0)
////                    {
////                        cout << "match!" << endl;
////                        instance_ix = count;
////                        found = true;
////                        // Check if the index is valid
////                        if (it->second)
////                        {
////                            int n_views = it->second->get_view_indices().size();
////                            if (view_index >= n_views)
////                            {
////                                ROS_WARN("EntropyMap::visualize : view index %u is larger than the number of views %u",
////                                         view_index, n_views);
////                                view_ix = -1;
////                            }
////                        }
////                        break;
////                    }
//                    ++count;
//                }
//                // If not found
//                if (!found)
//                {
//                    ROS_WARN("EntropyMap::visualize : cannot find instance name %s", instance_name.c_str());
//                    view_ix = -1;
//                }
//            }
//            else
//            {
//                ROS_WARN("EntropyMap::visualize : class entropy map for class %s is NULL", class_name.c_str());
//            }
//        }
//        else
//        {
//            ROS_WARN("EntropyMap::visualize : invalid class name %s so cannot search instance name %s",
//                     class_name.c_str(), instance_name.c_str());
//        }
//    }
//    else
//    {
//        ROS_WARN("EntropyMap::visualize : empty instance name");
//    }
//    // Call the visualization function
//    visualize (class_ix, instance_ix, view_ix, entropy_type);
//}

void EntropyMap::visualize(const string &class_name, const string &instance_name, const int &view_index, const string &score_type)
{
    // Find the indices of the requested class name and instance name
    if (class_name.size() == 0)
    {
        ROS_ERROR("EntropyMap::visualize : invalid class name %s", class_name.c_str());
        return;
    }
    // Get this class
    ClassEntropyMap* cem = _class_maps[class_name];
    if (!cem)
    {
        ROS_ERROR("EntropyMap::visualize : invalid class object from name %s", class_name.c_str());
        return;
    }
    // Get the instance
    if (instance_name.size() == 0)
    {
        ROS_WARN("EntropyMap::visualize : invalid instance name %s, viewing all instances", instance_name.c_str());
        for (map<string,InstanceEntropyMap*>::iterator it = cem->get_instance_maps().begin(); it != cem->get_instance_maps().end(); ++it)
        {
            int n = it->second->get_view_indices().size();
            if (view_index < n)
            {
                it->second->visualize(view_index, score_type);
            }
            else
            {
                ROS_WARN("EntropyMap::visualize : invalid view index %u, instance only has %u views", view_index, n);
                n = n - 1;
                it->second->visualize(n, score_type);
            }
        }
        return;
    }
    // Get the instance
    InstanceEntropyMap* iem = cem->get_instance_maps()[instance_name];
    if (!iem)
    {
        ROS_ERROR("EntropyMap::visualize : invalid instance object from name %s", instance_name.c_str());
        return;
    }
    // View this instance
    int n = iem->get_view_indices().size();
    if (view_index < n)
    {
        iem->visualize(view_index, score_type);
    }
    else
    {
        ROS_WARN("EntropyMap::visualize : invalid view index %u, instance only has %u views", view_index, n);
        n = n - 1;
        iem->visualize(n, score_type);
    }
}

/* === GETTERS === */

string EntropyMap::get_training_directory()
{
    return _training_directory;
}

string EntropyMap::get_training_directory() const
{
    return _training_directory;
}

string EntropyMap::get_descriptor_name()
{
    return _descriptor_name;
}

string EntropyMap::get_descriptor_name() const
{
    return _descriptor_name;
}

map<string,ClassEntropyMap*> EntropyMap::get_class_maps()
{
    return _class_maps;
}

map<string,ClassEntropyMap*> EntropyMap::get_class_maps() const
{
    return _class_maps;
}


/* === ROS SERVICE CALLBACKS === */

bool EntropyMap::extract_entropy_map(squirrel_object_perception_msgs::EntropyMap::Request &req,
                                     squirrel_object_perception_msgs::EntropyMap::Response &response)
{
    // Get the class name and instance name
    string class_name = req.class_type.data;
    string instance_name = req.instance_name.data;
    // Get the InstanceEntropyMap
    InstanceEntropyMap* iem = extract_instance_map(class_name, instance_name);
    // Check it is valid
    if (!iem)
    {
        ROS_ERROR("EntropyMap::extract_entropy_map : could not get valid instance entropy map for class type %s and instance name %s",
                  class_name.c_str(), instance_name.c_str());
        return false;
    }
    // Assign the values to the response message
    toROSMsg(*(iem->get_combined_cloud()), response.cloud);  // the point cloud
    // Set the octree file name
    response.octree_file.data = iem->get_octree_file();
    // For each instance view point
    for (size_t i = 0; i < iem->size(); ++i)
    {
        squirrel_object_perception_msgs::ViewpointEntropy m;

        sensor_msgs::PointCloud2 empty;
        m.cloud = empty;
        if (i < iem->get_point_clouds().size())
        {
            if (iem->get_point_clouds()[i] != NULL)
                toROSMsg(*(iem->get_point_clouds()[i]), m.cloud);
            else
                ROS_WARN("EntropyMap::extract_entropy_map : cloud %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no cloud for index %lu", i);
        }

        // Set the class type
        m.class_type.data = iem->get_class_name();

        // Set the instance name
        m.instance_name.data = iem->get_instance_name();

        // Set the view name
        m.view_name.data = "null";
        if (i < iem->get_view_indices().size())
        {
            if (iem->get_view_indices()[i] != NULL)
                m.view_name.data = boost::lexical_cast<string>(*(iem->get_view_indices()[i]));
            else
                ROS_WARN("EntropyMap::extract_entropy_map : view %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no view for index %lu", i);
        }

        // Convert the transformation matrix
        Eigen::Matrix4d idm = Eigen::Matrix4d::Identity();
        std_msgs::Float64MultiArray transform;
        for (size_t r = 0; r < idm.rows(); ++r)
        {
            for (size_t c = 0; c < idm.cols(); ++c)
                transform.data.push_back(idm(r,c));
        }
        if (i < iem->get_cloud_transforms().size())
        {
            if (iem->get_cloud_transforms()[i] != NULL)
            {
                tf::matrixEigenToMsg(*(iem->get_cloud_transforms()[i]), transform);
                m.transform = transform;
            }
            else
                ROS_WARN("EntropyMap::extract_entropy_map : transform %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no transform for index %lu", i);
        }

        // Convert the sensor pose
        SensorPose s_pose;
        s_pose.origin = Eigen::Vector4f(0,0,0,0);
        s_pose.orientation = Eigen::Quaternionf(0,0,0,0);
        if (i < iem->get_camera_poses().size())
        {
            if (iem->get_camera_poses()[i] != NULL)
                s_pose = *(iem->get_camera_poses()[i]);
            else
                ROS_WARN("EntropyMap::extract_entropy_map : camera pose %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no camera pose for index %lu", i);
        }
        geometry_msgs::Pose g_pose;
        g_pose.position.x = s_pose.origin[0];
        g_pose.position.y = s_pose.origin[1];
        g_pose.position.z = s_pose.origin[2];
        g_pose.orientation.x = s_pose.orientation.x();
        g_pose.orientation.y = s_pose.orientation.y();
        g_pose.orientation.z = s_pose.orientation.z();
        g_pose.orientation.w = s_pose.orientation.w();
        m.camera_pose = g_pose;

        // Set the centroid
        Eigen::Vector4f centroid = Eigen::Vector4f(0,0,0,0);
        if (i < iem->get_cloud_centroids().size())
        {
            if (iem->get_cloud_centroids()[i] != NULL)
                centroid = *(iem->get_cloud_centroids()[i]);
            else
                ROS_WARN("EntropyMap::extract_entropy_map : centroid %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no centroid for index %lu", i);
        }
        m.centroid.push_back(centroid[0]);
        m.centroid.push_back(centroid[1]);
        m.centroid.push_back(centroid[2]);

        // Set the visible entropy
        m.surface_area_proportion = 100000;
        if (i < iem->get_surface_area_proportions().size())
        {
            if (iem->get_surface_area_proportions()[i] != NULL)
                m.surface_area_proportion = *(iem->get_surface_area_proportions()[i]);
            else
                ROS_WARN("EntropyMap::extract_entropy_map : surface area measure %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no surface area measure for index %lu", i);
        }

        // Set the class entropy
        m.class_entropy = 100000;
        if (i < iem->get_class_entropies().size())
        {
            if (iem->get_class_entropies()[i] != NULL)
                m.class_entropy = *(iem->get_class_entropies()[i]);
            else
                ROS_WARN("EntropyMap::extract_entropy_map : class entropy %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no class entropy for index %lu", i);
        }

        // Set the self probability
        m.recognition_probability = 0;
        if (i < iem->get_self_probabilities().size())
        {
            if (iem->get_self_probabilities()[i] != NULL)
                m.recognition_probability = *(iem->get_self_probabilities()[i]);
            else
                ROS_WARN("EntropyMap::extract_entropy_map : recognition probabilty %lu is NULL", i);
        }
        else
        {
            ROS_WARN("EntropyMap::extract_entropy_map : no recognition probability for index %lu", i);
        }       

        // Add to the list
        response.entropy_map.push_back(m);
    }

    return true;
}

bool EntropyMap::visualize_entropy_map(squirrel_object_perception_msgs::EntropyMapViz::Request &req,
                                       squirrel_object_perception_msgs::EntropyMapViz::Response &response)
{
    // Get the class name and instance name
    string class_type = req.class_type.data;
    string instance_name = req.instance_name.data;
    int view_index = req.view_index;
    string score_type = req.score_type.data;
    ROS_INFO("EntropyMap::visualize_entropy_map : requested");
    ROS_INFO("class_type : %s", class_type.c_str());
    ROS_INFO("instance_name : %s", instance_name.c_str());
    ROS_INFO("view_index : %i", view_index);
    ROS_INFO("score_type : %s", score_type.c_str());
    // Visualize the entrop map instance
    visualize(class_type, instance_name, view_index, score_type);
    return true;
}



/* *********************
   *** MAIN FUNCTION ***
   *********************
*/

int main (int argc, char ** argv)
{
    ROS_INFO("*** STARTING ENTROPY MAP ***");
    int exit_code = EXIT_SUCCESS;

    EntropyMap *emap (new EntropyMap());
    bool success = emap->initialize(argc, argv);
    if (!success)
        exit_code = EXIT_FAILURE;

    delete emap;
    ROS_INFO("*** FINISH ENTROPY MAP ***");
    ros::shutdown();
    return exit_code;
}
