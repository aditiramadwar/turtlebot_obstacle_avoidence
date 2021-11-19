#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Move {
 public:
    Move(ros::NodeHandle& n);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

 private:
    ros::Subscriber laserscan;
    ros::Publisher pub;
    geometry_msgs::Twist velocity;
};

void Move::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  float distance = 1000;
  for (int i = 0; i < 60; i++) {
    if (msg->ranges[i] < distance)
      distance = msg->ranges[i];
  }
  ROS_INFO("Laser scan heard: %f", distance);
  if (distance < 0.4) {
    ROS_INFO("Very close!");
    velocity.linear.x = 0.0;
    velocity.angular.z = 1.0;
  }
  else {
    ROS_INFO("Safe");
    velocity.linear.x = 0.1;
    velocity.angular.z = 0.0;
  }
  pub.publish(velocity);
}

Move::Move(ros::NodeHandle& n) {
  laserscan = n.subscribe("/scan", 1000, &Move::callback, this);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(2);
  while (n.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move");
  ros::NodeHandle n;
  Move move(n);
  return 0;
}
