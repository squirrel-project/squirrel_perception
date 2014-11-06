// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <squirrel_sensorTester/sensorTester_nodeConfig.h>

// ROS message includes
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

// other includes
#include <sensorTester_node_common.cpp>


class sensorTester_node_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<squirrel_sensorTester::sensorTester_nodeConfig> server;
    dynamic_reconfigure::Server<squirrel_sensorTester::sensorTester_nodeConfig>::CallbackType f;

    ros::Subscriber test_publish_;
    ros::ServiceServer testService_;

    sensorTester_node_data component_data_;
    sensorTester_node_config component_config_;
    sensorTester_node_impl component_implementation_;

    sensorTester_node_ros() : np_("~")
    {
        f = boost::bind(&sensorTester_node_ros::configure_callback, this, _1, _2);
        server.setCallback(f);

        std::string testService_remap;
        n_.param("testService_remap", testService_remap, (std::string)"testService");
        testService_ = n_.advertiseService<std_srvs::Empty::Request , std_srvs::Empty::Response>(testService_remap, boost::bind(&sensorTester_node_impl::callback_testService, &component_implementation_,_1,_2,component_config_));

        test_publish_ = n_.subscribe("test_publish", 1, &sensorTester_node_ros::topicCallback_test_publish, this);


    }
    void topicCallback_test_publish(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        component_data_.in_test_publish = *msg;
    }

    void configure_callback(squirrel_sensorTester::sensorTester_nodeConfig &config, uint32_t level)
    {
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "sensorTester_node");

    sensorTester_node_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
