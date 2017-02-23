#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <squirrel_object_perception_msgs/Recognize2d.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <sstream>
#include <vector>
#include <squirrel_view_controller_msgs/FixateOnPoseAction.h>
#include <actionlib/client/simple_action_client.h>

#define DEFAULT_RECOGNIZER_TOPIC_ "/squirrel_recognizer/squirrel_recognize_objects"

class Object
{

public:
    squirrel_object_perception_msgs::SceneObject sceneObject;
};

class LookForObjectsAction
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

    squirrel_object_perception_msgs::SceneObject sceneObject;

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


    bool do_recognition()
    {
        if (!ros::service::waitForService(recognizer_topic_, ros::Duration(5.0)))
            return false;
        ros::ServiceClient client = nh_.serviceClient<squirrel_object_perception_msgs::Recognize2d>(recognizer_topic_);
        std::cout << "Recognizer topic: " << recognizer_topic_ << std::endl;


        squirrel_object_perception_msgs::Recognize2d srv;
        srv.request.image = scene;
        if (client.call(srv))
        {
            ROS_INFO("Called service %s: ", recognizer_topic_.c_str());
            if (srv.response.ids.size() >0)
            {
                int ind = most_confident_class(srv.response.confidences);
                sceneObject.category = srv.response.ids.at(ind).data;
                std::cout << "Category: " << sceneObject.category << std::endl;
                result_.objects_updated.push_back(sceneObject);

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

    void move_camera_to_hand() {
        //call action server
        actionlib::SimpleActionClient<squirrel_view_controller_msgs::FixateOnPoseAction> ac("squirrel_view_controller", true);
        ROS_INFO("Waiting for view controller action server to start.");
        ac.waitForServer();
        ROS_INFO("View controller action server started, sending goal.");

        squirrel_view_controller_msgs::FixateOnPoseGoal goal;
        goal.pose.header.frame_id= "/hand_base_link";
        goal.pose.pose.position.x = 0.0;
        goal.pose.pose.position.y = 0.0;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation.w = 0.0;
        goal.pose.pose.orientation.w = 0.0;
        goal.pose.pose.orientation.w = 0.0;
        goal.pose.pose.orientation.w = 1.0;

        goal.enable =true;

        ac.sendGoal(goal);
    }

    void stop_camera_fixation() {
        //call action server
        actionlib::SimpleActionClient<squirrel_view_controller_msgs::FixateOnPoseAction> ac("squirrel_view_controller", true);
        ROS_INFO("Waiting for view controller action server to start.");
        ac.waitForServer();
        ROS_INFO("View controller action server started, sending goal.");
        squirrel_view_controller_msgs::FixateOnPoseGoal goal;
        goal.enable =false;

        ac.sendGoal(goal);
    }


public:

    LookForObjectsAction(ros::NodeHandle &nh, std::string name) :
        as_(nh_, name, boost::bind(&LookForObjectsAction::executeCB, this, _1), false),
        action_name_(name)
    {
        nh_ = nh;
        as_.start();
        success = false;
        recognizer_topic_ = DEFAULT_RECOGNIZER_TOPIC_;

        if(nh_.getParam ( "recognizer_topic", recognizer_topic_ ))
            ROS_INFO("Listening to recognizer topic on %s", recognizer_topic_.c_str());
        else
            ROS_WARN("Recognizer topic not specified!");
    }

    ~LookForObjectsAction(void)
    {
    }

    void executeCB(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {

        sceneObject.id = goal->id;

        sensor_msgs::ImageConstPtr sceneConst;
        ROS_INFO("%s: executeCB started", action_name_.c_str());

        sleep(2); // HACK: Michael Zillich


        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted(result_);
        }

        // get data from depth camera
        sceneConst = ros::topic::waitForMessage<sensor_msgs::Image>("/kinect/rgb/image_rect_color", nh_, ros::Duration(20));

        if (sceneConst != NULL)
        {
            scene = *sceneConst;
            sceneConst.reset();
            ROS_INFO("%s: Received data", action_name_.c_str());
        }

        //the camera has to look at the hand
        move_camera_to_hand();

        //recognize
        success = do_recognition();

        //stop fixation at hand
        stop_camera_fixation();

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

    LookForObjectsAction lookforobjects(n, ros::this_node::getName());
    ROS_INFO("%s: ready...", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
