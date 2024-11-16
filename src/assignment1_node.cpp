#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <cmath>

#define NEW_TURTLE "turtle_2"

ros::Publisher tpub;


int main(int argc, char **argv) {
	ros::init(argc, argv, "turtle_subscriber");  
	ros::NodeHandle handle;

	ros::ServiceClient sclient = handle.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn t2spawn;
	t2spawn.request.x = 2.0;
	t2spawn.request.y = 1.0;
	t2spawn.request.theta = 0.0;
	t2spawn.request.name = NEW_TURTLE;

	sclient.call(t2spawn);

	return 0;
}