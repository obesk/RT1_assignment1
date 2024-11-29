#include "config.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"

#include <cstdlib>
#include <iostream>
#include <ctime>


bool check_input(int input, int size) {
	return input > 0 and input <= size;
}

int main(int argc, char **argv) {
	std::srand(std::time(NULL));

	if (N_TURTLES < 2) {
		ROS_ERROR("there should be at least 2 turtles");
		exit(1);
	}

	const ros::Duration turtle_moving_time(TURTLE_MOVING_TIME, 0);
	ros::init(argc, argv, "turtle_subscriber");  
	ros::NodeHandle handle;

	std::array <std::string, N_TURTLES>turtles;
	std::array <ros::Publisher, turtles.size()> publishers;

	ros::ServiceClient sclient = handle.serviceClient<turtlesim::Spawn>("/spawn");
	sclient.waitForExistence();

	for (int i = 0; i < N_TURTLES; ++i) {
		turtles[i] = "turtle" + std::to_string(i+1);

		if (i != 0) {
			turtlesim::Spawn tspawn;
			tspawn.request.x = std::rand() % static_cast<int>(BOARD_SIZE - ALLOWED_DISTANCE) + ALLOWED_DISTANCE;
			tspawn.request.y = std::rand() % static_cast<int>(BOARD_SIZE - ALLOWED_DISTANCE) + ALLOWED_DISTANCE;
			tspawn.request.theta = 0.0;
			tspawn.request.name = turtles[i];
			sclient.call(tspawn);
			ROS_INFO("Spawned turtle%d at coordinates x:%f, y%f", i+1, tspawn.request.x, tspawn.request.y);
		}
		publishers[i] = handle.advertise<geometry_msgs::Twist>("/" + turtles[i] + "/cmd_vel", 10);
	}

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
		
		geometry_msgs::Twist t{};
		t.linear.x = x;
		t.linear.y = y;
		t.angular.z = theta;

		const ros::Time start_time = ros::Time::now();

		ros::spinOnce();
		ros::Rate loop_rate(10);
		publishers[turtle].publish(t);
		while (ros::ok() && (ros::Time::now() - start_time) < turtle_moving_time) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		publishers[turtle].publish(geometry_msgs::Twist{});
		ros::spinOnce();
	}

	return 0;
}