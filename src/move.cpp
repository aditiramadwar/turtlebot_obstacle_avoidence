/**
 * @file move.cpp
 * @author Aditi Ramadwar (adiram@umd.edu)
 * @brief A simple code for turtlebot to detect obstacles and avoid them
 * @version 0.1
 * @date 2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
/**
 * @brief Move class
 * 
 */
class Move {
 public:
    explicit Move(ros::NodeHandle& n);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

 private:
    ros::Subscriber laserscan;
    ros::Publisher pub;
    geometry_msgs::Twist velocity;
};

/**
 * @brief Callback method for the laser scan subscriber,
 *        Based on the distance measured between the turtlebot and an obstacle
 *        the turtlebot will be controlled accordingly by publishing velocity
 *        control on the /cmd_vel topic
 * 
 * @param msg Data collected by the laser and shared over the /scan topic
 */
void Move::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float distance = 1000;
  // Evaluated the distance of the closest obstacle ahead
  for (int i = 0; i < 60; i++) {
    if (msg->ranges[i] < distance)
      distance = msg->ranges[i];
  }
  ROS_INFO("Laser scan heard: %f", distance);
  if (distance < 0.4 && distance > 0.2) {
  // If obstacle is detected then make robot take a turn to avoid it
    ROS_INFO("Obstacle detected!");
    velocity.linear.x = 0.0;
    velocity.angular.z = 1.0;
  } else if (distance < 0.2) {
    // If robot is too close to the obstacle then turn backwards
    ROS_INFO("Obstacle is very close!");
    velocity.linear.x = -0.1;
    velocity.angular.z = 1.0;
  } else {
    // If obstacle is far enough then keep going ahead
    ROS_INFO("Safe");
    velocity.linear.x = 0.1;
    velocity.angular.z = 0.0;
  }
  // Publish the desired velocity on /cmd_vel topic
  pub.publish(velocity);
}

/**
 * @brief Construct a new Move:: Move object
 *        Listens to the /scan topic and published
 * 
 * @param n 
 */
Move::Move(ros::NodeHandle& n) {
  // Listen to /scan topic and perform velocity changes depending on the data
  laserscan = n.subscribe("/scan", 1000, &Move::callback, this);
  // Publish to the /cmd_vel topic
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
 /**
  * Specifies a frequency. It will keep track of how long it has been since the last
  * call to Rate::sleep(), and sleep for the correct amount of time.
  */
  ros::Rate loop_rate(2);
  while (n.ok()) {
    ros::spinOnce();
    // Sleep for the time remaining to let us hit our loop_rate publish rate
    loop_rate.sleep();
  }
}

/**
 * @brief The main function of the controller to listen to laser topic and send information
 *        over velocity topic to control the robot 
 * @param argc Argument count
 * @param argv Argument vector
 * @return int 0
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "move");
  ros::NodeHandle n;
  Move move(n);
  return 0;
}
