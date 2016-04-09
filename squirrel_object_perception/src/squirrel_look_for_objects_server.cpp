#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <squirrel_object_perception_msgs/Classification.h>
#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/GetSaliency3DSymmetry.h>
#include <squirrel_object_perception_msgs/SegmentInit.h>
#include <squirrel_object_perception_msgs/SegmentOnce.h>
#include <squirrel_object_perception_msgs/SegmentsToObjects.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationInit.h>
#include <squirrel_object_perception_msgs/SegmentVisualizationOnce.h>
#include <squirrel_object_perception_msgs/Recognize.h>
#include <squirrel_planning_knowledge_msgs/UpdateObjectService.h>
#include <squirrel_planning_knowledge_msgs/AddObjectService.h>
#include <std_msgs/Int32MultiArray.h>
#include <sstream>
#include <algorithm>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <pcl/filters/passthrough.h>
#include<pcl_ros/transforms.h>



class Object
{

public:
    std::string id;
    std::string category;
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    sensor_msgs::PointCloud2 points;
    std_msgs::Int32MultiArray point_indices;
    squirrel_object_perception_msgs::BCylinder bounding_cylinder;
};

class LookForObjectsAction
{
protected:

    typedef pcl::PointXYZRGB PointT;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<squirrel_object_perception_msgs::LookForObjectsAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    squirrel_object_perception_msgs::LookForObjectsFeedback feedback_;
    squirrel_object_perception_msgs::LookForObjectsResult result_;
    // create needed variables
    sensor_msgs::PointCloud2ConstPtr scene;
    bool success;
    sensor_msgs::Image saliency_map;
    std::vector<Object> objects;
    std::vector<Object>::iterator objectIterator;
    // just for now
    std::vector<squirrel_object_perception_msgs::RecognizeResponse> recognized_object;
    std::vector<std_msgs::Int32MultiArray> cluster_indices;
    int id_cnt_;

    void set_publish_feedback(std::string phase, std::string status, int percent)
    {
        this->feedback_.current_phase = phase;
        this->feedback_.current_status = status;
        this->feedback_.percent_completed = percent;
        this->as_.publishFeedback(this->feedback_);
        return;
    }

    std::string get_unique_object_id()
    {
        std::stringstream ss;
        ss << this->id_cnt_;
        std::string str = ss.str();
        this->id_cnt_++;
        return (std::string("object") + str);
    }

    bool do_recognition(Object &object)
    {
        if (!ros::service::waitForService("/squirrel_recognizer/squirrel_recognize_objects", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize>("/squirrel_recognizer/squirrel_recognize_objects");

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*scene, *cloud);

        pcl::PointCloud<PointT>::Ptr segmented_object(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(object.points, *segmented_object);

        PointT min_p, max_p;
        pcl::getMinMax3D(*segmented_object, min_p, max_p);

        //TODO maybe add some buffer to the min/max points if segmentation method was not accurate
        pcl::PassThrough<PointT> pass;
        pass.setKeepOrganized(true);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_p.x, max_p.x);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_p.y, max_p.y);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_p.z, max_p.z);
        pass.setInputCloud(cloud);
        pass.filter(*cloud);


        squirrel_object_perception_msgs::Recognize srv;
        pcl::toROSMsg(*cloud, srv.request.cloud);
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_recognizer/squirrel_recognize_objects");
            this->recognized_object.push_back(srv.response);
            object.category = srv.response.ids.at(0).data; //this is only ok, when just one object gets recognized
            std::cout << "Category: " << object.category << std::endl;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool setup_visualization()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_visualization_init", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationInit>("/squirrel_segmentation_visualization_init");
        squirrel_object_perception_msgs::SegmentVisualizationInit srv;
        srv.request.cloud = *(this->scene);
        srv.request.saliency_map = this->saliency_map;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_init");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool run_visualization_once()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_visualization_once", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentVisualizationOnce>("/squirrel_segmentation_visualization_once");
        squirrel_object_perception_msgs::SegmentVisualizationOnce srv;
        srv.request.clusters_indices = this->cluster_indices;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_visualization_once");
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_saliency_map()
    {
        if (!ros::service::waitForService("/squirrel_attention_itti", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::GetSaliency3DSymmetry>("/squirrel_attention_itti");
        squirrel_object_perception_msgs::GetSaliency3DSymmetry srv;
        srv.request.cloud = *(this->scene);
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_attention_itti");
            this->saliency_map = srv.response.saliency_map;
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_attention_itti");
            return false;
        }
    }

    geometry_msgs::PoseStamped transform(double x, double y, double z, const std::string &from, const std::string &to)
    {
        geometry_msgs::PoseStamped before, after;

        before.pose.position.x = x;
        before.pose.position.y = y;
        before.pose.position.z = z;
        before.pose.orientation.x = 0;
        before.pose.orientation.y = 0;
        before.pose.orientation.z = 0;
        before.pose.orientation.w = 1;
        before.header.frame_id = from;
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose(to, before, after);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
        return after;
    }

    //transforms a whole point cloud and saves the result in the same object
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster, const std::string &from, const std::string &to) {
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            pcl_ros::transformPointCloud(to, *cloud_cluster, *cloud_cluster, tf_listener);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
    }

    //transforms a whole point cloud and saves the result in the same object
    void transformPointCloud(sensor_msgs::PointCloud2 &cloud_cluster, const std::string &from, const std::string &to) {
        try
        {
            tf_listener.waitForTransform(from, to, ros::Time::now(), ros::Duration(1.0));
            pcl_ros::transformPointCloud(to, cloud_cluster, cloud_cluster, tf_listener);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
    }


    bool add_object_to_db(Object object)
    {
        if (!ros::service::waitForService("/kcl_rosplan/add_object", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::AddObjectService>("/kcl_rosplan/add_object");
        squirrel_planning_knowledge_msgs::AddObjectService srv;
        srv.request.object.header = object.header;
        srv.request.object.header.frame_id = "/map";
        srv.request.object.id = object.id;
        srv.request.object.category = object.category;
        srv.request.object.pose = transform(object.pose.position.x, object.pose.position.y, object.pose.position.z, object.header.frame_id, "/map").pose;
        transformPointCloud(object.points, object.points.header.frame_id, "/map");
        srv.request.object.cloud = object.points;
        srv.request.object.bounding_cylinder = object.bounding_cylinder;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/kcl_rosplan/add_object");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/kcl_rosplan/add_object");
            return false;
        }
    }

    bool setup_segmentation()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_incremental_init", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
        squirrel_object_perception_msgs::SegmentInit srv;
        srv.request.cloud = *(this->scene);
        srv.request.saliency_map = this->saliency_map;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_init");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_init");
            return false;
        }
    }

    bool run_segmentation_once()
    {
        if (!ros::service::waitForService("/squirrel_segmentation_incremental_once", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
        squirrel_object_perception_msgs::SegmentOnce srv;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", "/squirrel_segmentation_incremental_once");
        }
        else
        {
            ROS_ERROR("Failed to call service %s", "/squirrel_segmentation_incremental_once");
            return false;
        }

        this->cluster_indices = srv.response.clusters_indices;


        for(int i=0; srv.response.poses.size(); i++) {
            ROS_INFO("appending object");
            Object obj;
            obj.category = "unknown";
            obj.id = get_unique_object_id();
            obj.header = srv.response.poses[i].header;
            obj.point_indices = srv.response.clusters_indices[i];
            obj.points = srv.response.points[i];
            obj.pose = srv.response.poses[i].pose;

            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::fromROSMsg(obj.points, *cloud);

            PointT min_p, max_p;
            pcl::getMinMax3D(*cloud, min_p, max_p);

            //it is in kinect-optical-frame! (x=z, y=x, z=y)
            double x_diam = double(max_p.x - min_p.x + 1);
            double y_diam = double(max_p.y - min_p.y + 1);
            double z_diam = double(max_p.z - min_p.z + 1);

            double diam = std::sqrt(std::pow(x_diam,2) + std::pow(z_diam,2));

            obj.bounding_cylinder.diameter = diam;
            obj.bounding_cylinder.height = y_diam;
            this->objects.push_back(obj);
            return true;
        }
    }

    bool update_object_in_db(Object object)
    {
        if (!ros::service::waitForService("/kcl_rosplan/update_object", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_planning_knowledge_msgs::UpdateObjectService>("/kcl_rosplan/update_object");
        squirrel_planning_knowledge_msgs::UpdateObjectService srv;
        srv.request.object.header = object.header;
        srv.request.object.header.frame_id = "/map";
        srv.request.object.id = object.id;
        srv.request.object.category = object.category;
        srv.request.object.pose = transform(object.pose.position.x, object.pose.position.y, object.pose.position.z, object.header.frame_id, "/map").pose;
        transformPointCloud(object.points, object.points.header.frame_id, "/map");
        srv.request.object.cloud = object.points;

        if (client.call(srv))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string most_confident_class(squirrel_object_perception_msgs::Classification classification)
    {
        std::vector<float>::iterator max;
        max = std::max_element(classification.confidence.begin(), classification.confidence.end());
        int max_index = std::distance(classification.confidence.begin(), max);
        return classification.class_type[max_index].data;
    }

    bool run_classifier()
    {
        if (!ros::service::waitForService("/squirrel_classify", ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Classify>("/squirrel_classify");
        squirrel_object_perception_msgs::Classify srv;
        // TODO: check data
        std::vector<std_msgs::Int32MultiArray> point_indices;
        point_indices.push_back(this->objects.back().point_indices);
        srv.request.cloud = *(this->scene);
        srv.request.clusters_indices = point_indices;
        if (client.call(srv))
        {
            if (srv.response.class_results.size() == 1)
            {
                this->objects.back().category = this->most_confident_class(srv.response.class_results[0]);
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

public:

    LookForObjectsAction(std::string name) :
        as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
        action_name_(name)
    {
        id_cnt_ = 1;
        as_.start();
        success = false;
    }

    ~LookForObjectsAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {

        ROS_INFO("%s: executeCB started", action_name_.c_str());

        sleep(2); // HACK: Michael Zillich

        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        // get data from depth camera
        scene = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", nh_, ros::Duration(5));
        if (scene)
        {
            ROS_DEBUG("%s: Received data", action_name_.c_str());
//            if (as_.Goal == squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE) {
//                //get lump size from DB and filter cloud for segmentation to cut off unnecessary parts

//            }
        }
        else
        {
            ROS_INFO("squirrel_object_perception: Did not receive any data from the camera");
            result_.result_status = "Unable to get data from the camera.";
            as_.setAborted(result_);
            success = false;
            return;
        }

        //TODO we do not use attention right now
        /*if (!get_saliency_map())
    {
        result_.result_status = "unable to get saliency map";
        as_.setAborted(result_);
        return;
    }*/

        if (!setup_segmentation())
        {
            result_.result_status = "unable to initialze segmentation";
            as_.setAborted(result_);
            return;
        }

        //not supported by popout_segmentation
        /*if (!setup_visualization())
    {
        result_.result_status = "unable to initialze visualization";
        as_.setAborted(result_);
        return;
    }*/


        // TODO: find a reasonable number of times to run here
        for(int i=0; i<1; i++)
        {
            run_segmentation_once();
            //run_visualization_once();
        }
        if (objects.size() < 1)
        {
            result_.result_status = "No objects classified";
            as_.setAborted(result_);
            return;
        }
        for(objectIterator = objects.begin(); objectIterator != objects.end(); objectIterator++)
        {
            do_recognition(*objectIterator);
            success = add_object_to_db(*objectIterator);
            if (!success)
                break;
        }

        if(success)
        {
            //result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
        else
        {
            result_.result_status = "Some error has occured.";
            as_.setAborted(result_);
        }

    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "look_for_objects");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    LookForObjectsAction lookforobjects(ros::this_node::getName());
    ros::spin();

    return 0;
}
