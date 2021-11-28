/**
 * Copyright 2021 Aditi Ramadwar
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list 
 * of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list 
 * of conditions and the following disclaimer in the documentation and/or other materials 
 * provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used 
 * to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 * 
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
    void avoidObstacles(double distance);

 private:
    ros::Subscriber laserscan;
    ros::Publisher pub;
    geometry_msgs::Twist velocity;
    double distance = 1000;
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
  // Evaluated the distance of the closest obstacle ahead
  distance = 1000;
  for (int i = 0; i < 75; i++) {
    if (msg->ranges[i] < distance)
      distance = msg->ranges[i];
  }
  avoidObstacles(distance);
}

/**
 * @brief A method to compute the distance measured by the turtlebot and avoid
 *        obstacles, if any, by turning the robot the other way.
 * 
 * @param distance 
 */
void Move::avoidObstacles(double distance) {
    if (distance < 0.4 && distance > 0.2) {
  // If obstacle is detected then make robot take a turn to avoid it
    ROS_INFO("Obstacle detected!");
    velocity.linear.x = 0.0;
    velocity.angular.z = 1.5;
  } else if (distance <= 0.2) {
    // If robot is too close to the obstacle then turn backwards
    ROS_INFO("Obstacle is very close!");
    velocity.linear.x = -0.1;
    velocity.angular.z = 1.5;
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
