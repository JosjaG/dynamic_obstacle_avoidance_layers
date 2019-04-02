#ifndef CUSTOM_LAYER_STATIC_H_
#define CUSTOM_LAYER_STATIC_H_
#include <ros/ros.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <social_navigation_layers/Boats.h>
#include <social_navigation_layers/Boat.h>
#include <social_navigation_layers/CustomLayerStaticConfig.h>

namespace social_navigation_layers
{
  class CustomLayerStatic : public SocialLayer
  {
    public:
      CustomLayerStatic() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      void configure(CustomLayerStaticConfig &config, uint32_t level);
      void timerCallback(const ros::TimerEvent&);
      int removeOldObstacles();
      void filterStatic();
      int search(social_navigation_layers::Boat& boat_in);
      double width_, height_;
      sensor_msgs::LaserScan laser_data;
      nav_msgs::Path current_path_;
      struct static_obstacle_ {
        ros::Time received;
        social_navigation_layers::Boat boat;
      };
      std::vector<static_obstacle_> static_obstacles_;
      ros::Duration static_keep_time_;
      bool timer;
      ros::Timer timer_;
      dynamic_reconfigure::Server<CustomLayerStaticConfig>* server_;
      dynamic_reconfigure::Server<CustomLayerStaticConfig>::CallbackType f_;
      tf::TransformListener listener_;
      std::list<social_navigation_layers::Boat> static_boats_;
  };
};
#endif