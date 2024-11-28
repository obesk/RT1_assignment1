#include <cmath>
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher distance_pub;
geometry_msgs::Twist stop_msg;

struct TurtleState {
    double x;
    double y;
    double t;
    double linear;
    double angular;

    const double distance(const TurtleState t) const {
        return std::sqrt(std::pow(this->x - t.x, 2) + std::pow(this->y - t.y, 2));
    };

};

TurtleState pos1, pos2;
TurtleState oldpos1, oldpos2;

// Callback to update Turtle 1's position
void turtlePosition1(const turtlesim::Pose::ConstPtr& msg){
    oldpos1 = pos1;
    pos1.x = msg->x;
    pos1.y = msg->y;
    pos1.t = msg->theta;
    pos1.linear = msg->linear_velocity;
    pos1.angular = msg->angular_velocity;
}

// Callback to update Turtle 2's position
void turtlePosition2(const turtlesim::Pose::ConstPtr& msg){
    oldpos2 = pos2;
    pos2.x = msg->x;
    pos2.y = msg->y;
    pos2.t = msg->theta;
    pos2.linear = msg->linear_velocity;
    pos2.angular = msg->angular_velocity;
}

int main (int argc, char **argv){
    // Ros init
    ros::init(argc, argv, "turtlebotDistanceNode"); 
    ros::NodeHandle nh;

    // Set up the speed to stop the turtles
    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.angular.z = 0;

    // Publishers for the turtles
    pub1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    pub2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

    // Subscribers for the turtles' positions
    ros::Subscriber sub1 = nh.subscribe<turtlesim::Pose>("turtle1/pose", 10, turtlePosition1);
    ros::Subscriber sub2 = nh.subscribe<turtlesim::Pose>("turtle2/pose", 10, turtlePosition2);

    // Publisher for the distance topic
    distance_pub = nh.advertise<std_msgs::Float32>("turtle_distance", 1);

    ros::Rate rate(20);

    while(ros::ok()){
        const double distance = pos1.distance(pos2);
        if (distance < 1) {
            const double old_distance = oldpos1.distance(oldpos2);
            if(old_distance > distance) {
                pub1.publish(stop_msg);
                pub2.publish(stop_msg);
                std::cout << "turtles are getting closer, stopping them ..." << std::endl;
            }
        }

        ros::spinOnce();  // Handle callback functions
        rate.sleep();     // Sleep for the remainder of the loop cycle
    }
    return 0;
}
