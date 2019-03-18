#ifndef DORY_BOATS_FROM_LIDAR_H
#define DORY_BOATS_FROM_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <social_navigation_layers/Boats.h>
#include <social_navigation_layers/Boat.h>

void filterBoats();
void publishBoats();
void matchBoats();
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

tf::TransformListener* listener_;
tf::TransformBroadcaster* broadcaster_;

// Topic and node names and message objects
ros::Publisher boats_pub_, lidar_pub_;
ros::Subscriber laser_sub_;

struct obstacle {
	int lidar_loc;
  geometry_msgs::Point min_point;
  geometry_msgs::Point closest_point;
  geometry_msgs::Point max_point;
};

double max_jump = 2.32;
double near_range_ = 8;
social_navigation_layers::Boats boats_list_;
std::vector<social_navigation_layers::Boat> detected_boats_;
std::vector<social_navigation_layers::Boat> prev_boats_;
std::vector<obstacle> obstacle_list;


#endif
