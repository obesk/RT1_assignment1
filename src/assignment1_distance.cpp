#include "config.h"

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

#include <algorithm>
#include <cmath>
#include <functional>

struct TurtleState {
    double x;
    double y;
    double t;
    double linear;
    double angular;

    const double distance(const TurtleState t) const {
        return std::sqrt(std::pow(this->x - t.x, 2) + std::pow(this->y - t.y, 2));
    };

    const double minDistanceFromBorders() {
        return std::min({BOARD_SIZE - this->x, this->x, BOARD_SIZE - this->y, this->y});
    }
};


void updateTurtlePosition(const turtlesim::Pose::ConstPtr& msg, TurtleState *curr, TurtleState *old){
    *old = *curr;
    curr->x = msg->x;
    curr->y = msg->y;
    curr->t = msg->theta;
    curr->linear = msg->linear_velocity;
    curr->angular = msg->angular_velocity;
}

int main (int argc, char **argv){
    // Ros init

    std::array<TurtleState, N_TURTLES> positions{};
    std::array<TurtleState, N_TURTLES> old_positions{};

    ros::init(argc, argv, "turtlebotDistanceNode"); 
    ros::NodeHandle nh;

    std::array <ros::Publisher, N_TURTLES> publishers;
    std::array <ros::Subscriber, N_TURTLES> subscribers;

    for (int i = 0; i < N_TURTLES; ++i) {
        const std::string turtle = "turtle" + std::to_string(i+1);

        publishers[i] = nh.advertise<geometry_msgs::Twist>("/" + turtle + "/cmd_vel", 10);

        subscribers[i] = nh.subscribe<turtlesim::Pose>("/" + turtle + "/pose", 10,
                        std::bind(updateTurtlePosition, std::placeholders::_1, &positions[i], &old_positions[i]));
    }

    ros::Rate rate(20);

    const geometry_msgs::Twist stop_msg{};

    while(ros::ok()){
        for (int i = 0; i < positions.size(); ++i) {
            // if the turtle is not moving there is no reason to check collisions
            if (positions[i].linear == 0 && positions[i].angular == 0) {
                continue;
            }

            //Preventig border collision
            const double min_distance = positions[i].minDistanceFromBorders();
            if (min_distance <= ALLOWED_DISTANCE) {
                const double old_min_distance = old_positions[i].minDistanceFromBorders();
                if (old_min_distance > min_distance) {
                    ROS_WARN("turtle%d getting too close to the borders, stopping it ...", i + 1);
                    publishers[i].publish(stop_msg);
                }
            }

            // Preventing collision with other turtles
            for (int j = 0; j < positions.size(); ++j) {
                if (i == j) {
                    continue;
                }

                const double min_distance = positions[i].distance(positions[j]);
                if (min_distance < ALLOWED_DISTANCE) {
                    const double old_min_distance = old_positions[i].distance(old_positions[j]);
                    if(old_min_distance > min_distance) {
                        publishers[i].publish(stop_msg);
                        ROS_WARN("turtle%d getting too close to turtle%d stopping it ...", i+1, j+1);
                    }
                }
            }
        }

        ros::spinOnce();  // Handle callback functions
        rate.sleep();     // Sleep for the remainder of the loop cycle
    }
    return 0;
}
