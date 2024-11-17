#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"

#include <iostream>


bool check_input(int input, int size) {
	return input > 0 and input <= size;
}

int main(int argc, char **argv) {
	const ros::Duration turtle_moving_time(1, 0);
	const std::vector <std::string>turtles  = {"turtle1", "turtle2"};

	ros::init(argc, argv, "turtle_subscriber");  
	ros::NodeHandle handle;

	ros::ServiceClient sclient = handle.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn t2spawn;
	t2spawn.request.x = 2.0;
	t2spawn.request.y = 1.0;
	t2spawn.request.theta = 0.0;
	t2spawn.request.name = turtles[1];

	sclient.call(t2spawn);

	while (ros::ok) {
		int input = -1;

		while (!check_input(input, turtles.size())) {
			std::cout << "\nInsert what you want to do:" << std::endl;
			for (int i = 0; i < turtles.size(); ++i) {
			 	std::cout << i+1 << ". Move turtle " << turtles[i] << std::endl;
			}

			const int quit_option = turtles.size() + 1;
			std::cout << quit_option << ". QUIT" << std::endl;

			std::cout << ">> "; 
			std::cin >> input;

			if (input == quit_option) {
				exit(0);
			}

			if (!check_input(input, turtles.size())) {
				std::cout << "Invalid option number, try again..." << std::endl;
			}
		}
		const int turtle = input - 1;

		float x, y, theta; 
		std::cout << "Insert the x velocity: ";
		std::cin >> x;
		std::cout << "Insert the y velocity: ";
		std::cin >> y;
		std::cout << "Insert the angular velocity: ";
		std::cin >> theta;
		
		geometry_msgs::Twist t;
		t.linear.x = x;
		t.linear.y = y;
		t.angular.z = theta;

		ros::Publisher tpub = handle.advertise<geometry_msgs::Twist>("/" + turtles[turtle] + "/cmd_vel", 10);

		const ros::Time start_time = ros::Time::now();

		ros::Rate loop_rate(10);
		while (ros::ok() && ros::Time::now() - start_time < turtle_moving_time) {
			tpub.publish(t);
			ros::spinOnce();
			loop_rate.sleep();
		}

		tpub.publish(geometry_msgs::Twist{});
		ros::spinOnce();
	}

	return 0;
}