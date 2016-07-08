/**
 * AttentionLegs.cpp
 *
 * Attends to the legs in the laser data.
 * 
 * 
 * @author Michael Zillich zillich@acin.tuwien.ac.at
 * @date Sept 2015
 */

#include <math.h>
#include <ios>
#include <robotino_msgs/LookAtImagePosition.h>
#include <squirrel_attention/AttentionLegs.h>

using namespace std;

AttentionLegs::AttentionLegs()
{
  legsSub_ = nh_.subscribe("/laser_person", 2, &AttentionLegs::legsCallback, this);
}

AttentionLegs::~AttentionLegs()
{
}

void AttentionLegs::run()
{
  ros::spin();
}

void AttentionLegs::legsCallback(const std_msgs::String& msg)
{
  stringstream poss(msg.data);
  float x, y;
  poss >> x >> y;
  ROS_INFO("look at person at %.3f %.3f", x, y);
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "attention_legs_node");
  AttentionLegs al;
  al.run();
  exit(0);
}
