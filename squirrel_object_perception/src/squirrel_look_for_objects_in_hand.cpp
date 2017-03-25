#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <squirrel_object_perception_msgs/Recognize2d.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <sstream>
#include <vector>
#include <squirrel_view_controller_msgs/LookAtPosition.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>

#define DEFAULT_RECOGNIZER_TOPIC_ "/squirrel_recognizer/squirrel_recognize_objects"

class Object
{

public:
    squirrel_object_perception_msgs::SceneObject sceneObject;
};

class LookForObjectsInHandAction
{
protected:

    ros::NodeHandle nh_;

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<squirrel_object_perception_msgs::LookForObjectsAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    squirrel_object_perception_msgs::LookForObjectsFeedback feedback_;
    squirrel_object_perception_msgs::LookForObjectsResult result_;
    // create needed variables
    sensor_msgs::Image scene;
    bool success;
    tf::TransformListener tf_listener;
    ros::Publisher marker_pub;
    int id_cnt_;
    bool called_cam_service;
    squirrel_object_perception_msgs::SceneObject sceneObject;
    float max_conf;
    ros::Publisher mode_pub;
    ros::Publisher joint_pub;

    float dist_to_hand_thresh;
    int hand_joint;
    int DOF;

    // Recognition topic
    std::string recognizer_topic_;
    void set_publish_feedback(std::string phase, std::string status, int percent)
    {
        this->feedback_.current_phase = phase;
        this->feedback_.current_status = status;
        this->feedback_.percent_completed = percent;
        this->as_.publishFeedback(this->feedback_);
        return;
    }

    std::string get_unique_object_id() {
        std::stringstream ss;
        ss << id_cnt_;
        std::string str = ss.str();
        id_cnt_++;
        return str;
    }


    bool do_recognition()
    {
        if (!ros::service::waitForService(recognizer_topic_, ros::Duration(5.0))) {
            ROS_ERROR("Recognizer not available");
            return false;
        }
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize2d>(recognizer_topic_);
        std::cout << "Recognizer topic: " << recognizer_topic_ << std::endl;


        squirrel_object_perception_msgs::Recognize2d srv;
        srv.request.image = scene;
        int obj_ind = -1;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", recognizer_topic_.c_str());
            if (srv.response.ids.size() > 0)
            {
                for (int i = 0; i < srv.response.ids.size(); i++) {
                    if (srv.response.confidences.at(i) > 0.45) {
                        if (max_conf < srv.response.confidences.at(i)) {
                            if (check_pose(srv.response.transforms.at(i))) {
                                max_conf = srv.response.confidences.at(i);
                                obj_ind = i;
                            }
                        }
                    }
                }

                if (obj_ind != -1) {
                    sceneObject.category = srv.response.ids.at(obj_ind).data;

                    geometry_msgs::Transform obj_transform = srv.response.transforms.at(obj_ind);
                    sceneObject.pose = transformToPose(obj_transform);

                    geometry_msgs::PoseStamped helper_pose;
                    helper_pose.pose = sceneObject.pose;
                    helper_pose.header.frame_id = "/kinect_rgb_optical_frame";
                    try
                    {
                        tf_listener.waitForTransform("/map", "/kinect_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
                        tf_listener.transformPose("/map", helper_pose, helper_pose);
                        sceneObject.pose = helper_pose.pose;
                        sceneObject.header.frame_id = helper_pose.header.frame_id;
                    }
                    catch (tf::TransformException& ex)
                    {
                        ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
                    }
                }

                return true;
            } else {
                std::cout << "could not recognize an object!" << std::endl;
                return true;
            }
        }
        else
        {
            return false;
        }
    }

    geometry_msgs::Pose transformToPose(geometry_msgs::Transform transf) {
        geometry_msgs::Pose pose;

        Eigen::Quaternionf q_eigen(transf.rotation.w, transf.rotation.x, transf.rotation.y, transf.rotation.z);
        Eigen::Matrix3f R = q_eigen.toRotationMatrix();
        Eigen::Vector3f t(transf.translation.x, transf.translation.y, transf.translation.z);

        Eigen::Vector3f pt0 = R * Eigen::Vector3f(0,0,0) + t;
        //std::cout << "X: " << pt0[0] << " Y: " << pt0[1] << " Z: " << pt0[2] << std::endl;

        pose.position.x = pt0[0];
        pose.position.y = pt0[1];
        pose.position.z = pt0[2];
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        return pose;
    }

    bool check_pose(geometry_msgs::Transform transf) {
        ROS_INFO("Check pose");
        geometry_msgs::PoseStamped before, after;

        before.pose = transformToPose(transf);
        before.header.frame_id = "/kinect_rgb_optical_frame";

        try
        {
            tf_listener.waitForTransform("/hand_palm_link", "/kinect_rgb_optical_frame", ros::Time::now(), ros::Duration(1.0));
            tf_listener.transformPose("/hand_palm_link", before, after);

            double obj_dist = sqrt(after.pose.position.x * after.pose.position.x +
                                   after.pose.position.y * (after.pose.position.y) +
                                   (after.pose.position.z-0.1) * (after.pose.position.z-0.1));
            if (obj_dist > dist_to_hand_thresh) {
                return false;
            } else {
                return true;
            }
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s: %s", ros::this_node::getName().c_str(), ex.what());
        }
    }

    int most_confident_class(std::vector<float> r)
    {
        float  max_val=0.0;
        int ind = 0;
        for (size_t i = 0; i < r.size(); i++) {
            if (r.at(i) > max_val) {
                max_val = r.at(i);
                ind = i;
            }

        }
        return ind;
    }

    bool move_camera_to_hand() {
        //call view controller service
        ros::ServiceClient client = nh_.serviceClient<squirrel_view_controller_msgs::LookAtPosition>("/squirrel_view_controller/look_at_position");

        squirrel_view_controller_msgs::LookAtPosition srv;
        srv.request.target.header.frame_id= "/hand_palm_link";
        srv.request.target.pose.position.x = 0.0;
        srv.request.target.pose.position.y = 0.1;
        srv.request.target.pose.position.z = 0.0;
        srv.request.target.pose.orientation.x = 0.0;
        srv.request.target.pose.orientation.y = 0.0;
        srv.request.target.pose.orientation.z = 0.0;
        srv.request.target.pose.orientation.w = 1.0;

        if(client.call(srv)) {
            ROS_INFO("Moved camera to hand");
            return true;
        } else {
            ROS_INFO("Did NOT move camera to hand");
            return false;
        }
    }


    void getImage(const sensor_msgs::Image::ConstPtr& msg)
    {
        if (!called_cam_service) { //this is a little hack
            sensor_msgs::ImageConstPtr sceneConst = ros::topic::waitForMessage<sensor_msgs::Image>("/kinect/rgb/image_rect_color", nh_);
            std::cout << "Received camera image." << std::endl;
            scene = *sceneConst;
            called_cam_service = true;
        }
    }

    bool moveCameraToDefault() {
        //move back to default position
        ros::ServiceClient tilt_client = nh_.serviceClient<std_srvs::Empty>("/tilt_controller/resetPosition");

        std_srvs::Empty e;
        if (!tilt_client.call(e))
        {
            ROS_ERROR("Failed to move camera back to default tilt position");
            return false;
        }

        ros::ServiceClient pan_client = nh_.serviceClient<std_srvs::Empty>("/pan_controller/resetPosition");
        if(!pan_client.call(e))
        {
            ROS_ERROR("Failed to move camera back to default pan position");
            return false;
        }
        return true;
    }

    bool move_hand(float desired_angle) {
        sensor_msgs::JointStateConstPtr current_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/real/robotino/joint_control/get_state", nh_, ros::Duration(3));
        float step_size = 0.0f;
        if (current_state->position[hand_joint] > desired_angle) {
            step_size = -0.02;
        } else {
            step_size = 0.02;
        }

        std_msgs::Float64MultiArray joint_array;
        for (int i = 0; i < DOF; i++) {
            joint_array.data.push_back(std::numeric_limits<double>::quiet_NaN());
        }

        float dist = std::numeric_limits<float>::max();
        while (dist > 0.02) {
            sensor_msgs::JointStateConstPtr current_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/real/robotino/joint_control/get_state", nh_, ros::Duration(3));
            if (current_state == NULL) {
                return false;
            }
            if (dist == std::fabs(desired_angle - current_state->position[hand_joint])) {
                ROS_INFO("Hand reached its limit");
                return false;
            }
            dist = std::fabs(desired_angle - current_state->position[hand_joint]);
            joint_array.data[hand_joint] = current_state->position[hand_joint] + step_size;
            joint_pub.publish(joint_array);
        }

        ROS_INFO("Moved hand");

        return true;

    }



public:

    LookForObjectsInHandAction(ros::NodeHandle &nh, std::string name) :
        as_(nh_, name, boost::bind(&LookForObjectsInHandAction::executeCB, this, _1), false),
        action_name_(name)
    {
        nh_ = nh;
        as_.start();
        success = false;
        called_cam_service = false;
        recognizer_topic_ = DEFAULT_RECOGNIZER_TOPIC_;
        dist_to_hand_thresh = 0.20;
        hand_joint = 7;
        DOF = 8;

        mode_pub = nh_.advertise<std_msgs::Int32>("/real/robotino/settings/switch_mode",1);
        joint_pub = nh_.advertise<std_msgs::Float64MultiArray>("/real/robotino/joint_control/move", 1);

        marker_pub = nh_.advertise<visualization_msgs::Marker>("vis_marker_test", 1, true);

        if(nh_.getParam ( "recognizer_topic", recognizer_topic_ ))
            ROS_INFO("Listening to recognizer topic on %s", recognizer_topic_.c_str());
        else
            ROS_WARN("Recognizer topic not specified!");
    }

    ~LookForObjectsInHandAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {
        called_cam_service = false;
        success = true;
        result_.objects_added.clear();
        result_.objects_updated.clear();
        max_conf = std::numeric_limits<float>::min();
        sceneObject.id = goal->id;

        ROS_INFO("%s: executeCB from recognition in hand is started", action_name_.c_str());
        sleep(2); // HACK: Michael Zillich

        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        //the camera has to look at the hand
        success = move_camera_to_hand();
        if (!success) {
            result_.result_status = "unable to move camera to the hand";
            as_.setAborted(result_);
            return;
        }

        //move hand
        std_msgs::Int32 switch_mode;
        switch_mode.data = 10;
        mode_pub.publish(switch_mode);
        sensor_msgs::JointStateConstPtr joint_state_original = ros::topic::waitForMessage<sensor_msgs::JointState>("/real/robotino/joint_control/get_state", nh_, ros::Duration(30));
        float degree_rad = 60.0 * M_PI /180.0;
        for (float f = degree_rad; f >= -degree_rad; f -= degree_rad/2)
        {
            move_hand(joint_state_original->position[hand_joint] + f);
            ros::Subscriber sub_pc = nh_.subscribe ("/kinect/rgb/image_rect_color", 1, &LookForObjectsInHandAction::getImage, this);
            ros::Rate loop_rate (1);
            // poll until we did receive a point cloud
            sleep(3);
            while(!called_cam_service)
            {
                ros::spinOnce ();
                loop_rate.sleep ();
            }
            sub_pc.shutdown();
            called_cam_service = false;

            //recognize
            success = do_recognition();
            if (!success) {
                result_.result_status = "unable to recognize";
                as_.setAborted(result_);
                return;
            }
            if (max_conf == 1.0) {
                break;
            }
        }
        //move camera back to default position
        success = moveCameraToDefault();
        if (!success) {
            result_.result_status = "unable to move camera back to default position";
            as_.setAborted(result_);
            return;
        }

        if (max_conf == std::numeric_limits<float>::max()) {
            ROS_INFO("No object was recognized close to the hand");
            success = true;
        } else {
            std::cout << "Category: " << sceneObject.category << std::endl;
            result_.objects_added.push_back(sceneObject);
        }

        std::cout << "SUCCESS of action: " << success << std::endl;
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
    ros::init(argc, argv, "squirrel_look_for_objects_in_hand");
    ros::NodeHandle n("~");
    ROS_INFO("%s: started node", ros::this_node::getName().c_str());

    LookForObjectsInHandAction lookforobjects(n, ros::this_node::getName());
    ROS_INFO("%s: ready...", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
