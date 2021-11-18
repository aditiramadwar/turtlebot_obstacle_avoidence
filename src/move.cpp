#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

class Move {
 public:
    Move(ros::NodeHandle& n);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

 // private:
 //  ros::Subscriber laserscan;
};

void Move::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO("Laser scan heard: %f", msg->ranges[0]);
}

Move::Move(ros::NodeHandle& n) {
  //laserscan = n.subscribe("/scan", 1000, &Move::callback);
  ros::Subscriber laserscan = n.subscribe("/scan", 1000, &Move::callback, this);
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
