// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <squirrel_sensing/sensor_nodeConfig.h>

// ROS message includes
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <bride_tutorials/TriggerPublishAction.h>

// other includes
#include <sensor_node_common.cpp>


class sensor_node_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<squirrel_sensing::sensor_nodeConfig> server;
    dynamic_reconfigure::Server<squirrel_sensing::sensor_nodeConfig>::CallbackType f;

    ros::Publisher get_tactile_;
    ros::Publisher get_proximity_;
    ros::Publisher get_wrist_;
    actionlib::SimpleActionServer<bride_tutorials::TriggerPublishAction> as_trigger_proximity_;

    sensor_node_data component_data_;
    sensor_node_config component_config_;
    sensor_node_impl component_implementation_;

    sensor_node_ros() : np_("~")
    , as_trigger_proximity_(n_, "trigger_proximity", boost::bind(&sensor_node_impl::callback_trigger_proximity_, &component_implementation_, _1, &as_trigger_proximity_), false)
    {
        f = boost::bind(&sensor_node_ros::configure_callback, this, _1, _2);
        server.setCallback(f);
        as_trigger_proximity_.start();


        get_tactile_ = n_.advertise<std_msgs::Float64MultiArray>("get_tactile", 1);
        get_proximity_ = n_.advertise<std_msgs::Float64MultiArray>("get_proximity", 1);
        get_wrist_ = n_.advertise<std_msgs::Float64MultiArray>("get_wrist", 1);

    }

    void configure_callback(squirrel_sensing::sensor_nodeConfig &config, uint32_t level)
    {
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_get_tactile_active = true;
        component_data_.out_get_proximity_active = true;
        component_data_.out_get_wrist_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_get_tactile_active)
            get_tactile_.publish(component_data_.out_get_tactile);
        if (component_data_.out_get_proximity_active)
            get_proximity_.publish(component_data_.out_get_proximity);
        if (component_data_.out_get_wrist_active)
            get_wrist_.publish(component_data_.out_get_wrist);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "sensor_node");

    sensor_node_ros node;
    node.configure();

    ros::Rate loop_rate(1000.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
