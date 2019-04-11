#ifndef SOCIAL_LAYER_H_
#define SOCIAL_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_obstacle_avoidance_layers/Boats.h>
#include <dynamic_obstacle_avoidance_layers/Boat.h>
#include <boost/thread.hpp>

namespace dynamic_obstacle_avoidance_layers
{
  class SocialLayer : public costmap_2d::Layer
  {
    public:
      SocialLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;
      virtual void updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

    protected:
      void boatsCallback(const dynamic_obstacle_avoidance_layers::Boats& boats);
      ros::Subscriber boats_sub_;
      dynamic_obstacle_avoidance_layers::Boats boats_list_;
      ros::Duration boats_keep_time_;
      boost::recursive_mutex lock_;
      tf::TransformListener tf_;
      bool first_time_;
      double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  };
};
#endif