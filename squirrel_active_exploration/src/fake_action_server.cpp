#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_object_perception_msgs/LookForObjectsAction.h>

using namespace std;

typedef actionlib::SimpleActionServer<squirrel_object_perception_msgs::LookForObjectsAction> Server;

class ActionServer
{
public:

    ActionServer(const string &name)
        : _ac_server(_n, name, boost::bind(&ActionServer::callback, this, _1), false),
          _action_name(name)
    {
        _ac_server.start();
        ROS_INFO("ActionServer::ActionServer : waiting for goals");
    }

    ~ActionServer(void)
    {
    }

    void callback(const squirrel_object_perception_msgs::LookForObjectsGoalConstPtr &goal)
    {
        ROS_INFO("ActionServer::callback : received goal with id %s", goal->id.c_str());

//        _feedback.current_phase = "fake";
//        _feedback.current_status = "action achieved";
//        _feedback.percent_completed = 0;

        _result.result_status = "action achieved";
        _ac_server.setSucceeded(_result, "action achieved");
    }

protected:
    ros::NodeHandle _n;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    Server _ac_server;
    string _action_name;
    // Create messages that are used to published feedback/result
    squirrel_object_perception_msgs::LookForObjectsFeedback _feedback;
    squirrel_object_perception_msgs::LookForObjectsResult _result;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "look_for_objects");

    ROS_INFO("look_for_objects::main : starting action server");
    ActionServer server("look_for_objects");
    ros::spin();

    return EXIT_SUCCESS;
}
