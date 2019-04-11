#ifndef DORY_BOATS_FROM_LIDAR_H
#define DORY_BOATS_FROM_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_obstacle_avoidance_layers/Boats.h>
#include <dynamic_obstacle_avoidance_layers/Boat.h>

void filterBoats();
void publishBoats();
const char * matchBoats(int lidar);
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void mapCallback(const nav_msgs::OccupancyGrid& map);

tf::TransformListener* listener_;
tf::TransformBroadcaster* broadcaster_;

// Topic and node names and message objects
ros::Publisher boats_pub_, lidar_pub_;
ros::Subscriber laser_sub_, map_sub_;

struct obstacle {
	int lidar_loc;
  geometry_msgs::Point min_point;
  geometry_msgs::Point closest_point;
  geometry_msgs::Point max_point;
};

double max_jump, min_size, min_obstacle_size;
int near_range;
dynamic_obstacle_avoidance_layers::Boats boats_list_;
nav_msgs::OccupancyGrid map_;
std::vector<dynamic_obstacle_avoidance_layers::Boat> detected_boats_;
std::vector<dynamic_obstacle_avoidance_layers::Boat> prev_boats_;
std::vector<obstacle> obstacle_list;


#endif
