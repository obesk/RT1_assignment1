# Research Track 1 assingment

This is the solution of the first assingment of the Research Track 1 course.

## Assignment

The assingment is to create two ROS nodes that interact with turtlesim:

- The first node should spawn a second turtle and let the user control one turtle at a time by setting it's velocity, the veloity is then maintained for 1s and after that the turtle is stopped
- The first node should prevent the turtles from colliding with each other and with the walls

## How to run

1. Start ROS with the command `roscore`
1. Be sure to clone this code in you ros workspace, i.e. `~/my_ros`
1. From your ROS workspace compile the code with `catkin_make`
1. Start the turtlesim node with: `rosrun turtlesim turtlesim_node`
1. Start the UI node with `rosrun assignment1 assignment1_UI` 
1. Start the supervisor node with: `rosrun assignment1 assignment1_supervisor`

## How to use

You can control the turtles by interacting with the UI node, the node will ask you what turtle you want to control and at what speed. The 
turtle will keep it's speed for 1s and then stop. 

Here we care about our turtles: so the supervisor node will make sure that no turtle 
gets harmed by colliding with a wall or with another turtle!

## BONUS: MORE TURTLES 🐢!

![more turtles](/img/more_turtles.png)

Are you tired of playing with ONLY two turtles? Luckily for you this code is 
parametrized so that you can play with as much turtles as you like.

To change the number of turtles it's possible to edit the file `include/config.h`
and change the value of the `N_TURTLES` macro. Note: only numbers greater than or to 2 are accepted

From the file `config.h` you can also change the minimum distance between turtles
(`ALLOWED_DISTANCE`) and the turtle moving time (`TURTLE_MOVING_TIME`).
