#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/CreatePRM.h>
#include <rosplan_knowledge_msgs/AddWaypoint.h>
#include <rosplan_knowledge_msgs/Filter.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "tidy_room_execution");
	ros::NodeHandle nh;

	ros::ServiceClient roadmap_service = nh.serviceClient<rosplan_knowledge_msgs::CreatePRM>("/kcl_rosplan/roadmap_server/create_prm");
    ros::ServiceClient add_waypoint_service = nh.serviceClient<rosplan_knowledge_msgs::AddWaypoint>("/kcl_rosplan/roadmap_server/add_waypoint");
	
	// Get access to the knowledge base.
	ros::ServiceClient get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_instances");
	ros::ServiceClient get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_instances_attributes");
	ros::ServiceClient knowledge_update_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	ros::Publisher filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/mission_filter", 10, true);

	// Planner control.
	ros::ServiceClient run_planner_client = nh.serviceClient<std_srvs::Empty>("/kcl_rosplan/planning_server");

    // NEW
    // Add kenny
    rosplan_knowledge_msgs::KnowledgeUpdateService add_kenny;
    add_kenny.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
    rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
    kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
    kenny_knowledge.instance_type = "robot";
    kenny_knowledge.instance_name = "kenny";
    add_kenny.request.knowledge = kenny_knowledge;
    if (!knowledge_update_client.call(add_kenny)) {
            ROS_ERROR("KCL: (TidyRoom) Could not add kenny to the knowledge base.");
            exit(-1);
    }
    ROS_INFO("KCL: (TidyRoom) Added kenny to the knowledge base.");

	// clear the old filter
	rosplan_knowledge_msgs::Filter filterMessage;
	filterMessage.function = rosplan_knowledge_msgs::Filter::CLEAR;
	filter_publisher.publish(filterMessage);

	// push the new filter
	filterMessage.function = rosplan_knowledge_msgs::Filter::ADD;
	rosplan_knowledge_msgs::KnowledgeItem object_filter;
	object_filter.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	object_filter.instance_type = "object";
	filterMessage.knowledge_items.push_back(object_filter);
	rosplan_knowledge_msgs::KnowledgeItem waypoint_filter;
	waypoint_filter.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	waypoint_filter.instance_type = "waypoint";
	filterMessage.knowledge_items.push_back(waypoint_filter);
	filter_publisher.publish(filterMessage);
	rosplan_knowledge_msgs::KnowledgeItem tidy_location;
	tidy_location.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
	tidy_location.attribute_name = "tidy_location";
	filterMessage.knowledge_items.push_back(tidy_location);
	filter_publisher.publish(filterMessage);
	
	// Keep running forever.
	bool room_is_tidy = false;
    //while (!room_is_tidy) {
		ros::spinOnce();
		ROS_INFO("KCL: (TidyRoom) Start the loop!");
		
		// Start by generating some waypoints.
		rosplan_knowledge_msgs::CreatePRM create_prm;
        create_prm.request.nr_waypoints = 1;
		create_prm.request.min_distance = 0.5;
		create_prm.request.casting_distance = 1;
		create_prm.request.connecting_distance = 5;
		create_prm.request.occupancy_threshold = 20;
		create_prm.request.total_attempts = 1000;
		if (!roadmap_service.call(create_prm)) {
			ROS_ERROR("KCL: (TidyRoom) Failed to call the road map service.");
			return -1;
		}
		ROS_INFO("KCL: (TidyRoom) Road map service returned.");

        // ADD NEW WAYPOINT
        rosplan_knowledge_msgs::AddWaypoint add_waypoint1;
        add_waypoint1.request.id = "wp1";
        add_waypoint1.request.waypoint.header.frame_id = "/map";
        add_waypoint1.request.waypoint.pose.position.x = 0.0;
        add_waypoint1.request.waypoint.pose.position.y = -1.0;
        add_waypoint1.request.waypoint.pose.orientation.w = 1.0;
        add_waypoint1.request.occupancy_threshold = 70;
        add_waypoint1.request.connecting_distance = 5;
        if (!add_waypoint_service.call(add_waypoint1)) {
                ROS_ERROR("KCL: (TidyRoom) Failed to add the first waypoint.");
                return -1;
        }
		
		// Retreive the waypoints and add an observation goal to them.
		rosplan_knowledge_msgs::GetInstanceService get_instances;
		get_instances.request.type_name = "waypoint";
		
		if (!get_instance_client.call(get_instances)) {
			ROS_ERROR("KCL: (TidyRoom) Failed to get all the waypoint instances.");
			return -1;
		}
		ROS_INFO("KCL: (TidyRoom) Received all the waypoint instances.");
		
		// Generate a new goal for every waypoint we have received.
		for (std::vector<std::string>::const_iterator ci = get_instances.response.instances.begin(); ci != get_instances.response.instances.end(); ++ci)
		{
			const std::string& waypoint = *ci;
			
			// Setup the goal.
			rosplan_knowledge_msgs::KnowledgeItem waypoint_goal;
			waypoint_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
			waypoint_goal.attribute_name = "explored";
			diagnostic_msgs::KeyValue kv;
			kv.key = "wp";
			kv.value = *ci;
			waypoint_goal.values.push_back(kv);
			
			// Add the goal.
			rosplan_knowledge_msgs::KnowledgeUpdateService add_goal;
			add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
			add_goal.request.knowledge = waypoint_goal;
			
			if (!knowledge_update_client.call(add_goal)) {
				ROS_ERROR("KCL: (TidyRoom) Could not add the goal for waypoint %s to the knowledge base.", (*ci).c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (TidyRoom) Added the goal for waypoint %s to the knowledge base.", (*ci).c_str());
		}
		
		// Run the planner.
		std_srvs::Empty dummy;
		if (!run_planner_client.call(dummy)) {
			ROS_ERROR("KCL: (TidyRoom) Failed to run the planning system.");
			exit(-1);
		}
		ROS_INFO("KCL: (TidyRoom) Planning system returned.");
    //}
	return 0;
}
