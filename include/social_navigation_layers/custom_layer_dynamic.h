#ifndef CUSTOM_LAYER_DYNAMIC_H_
#define CUSTOM_LAYER_DYNAMIC_H_
#include <ros/ros.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <social_navigation_layers/Boats.h>
#include <social_navigation_layers/Boat.h>
#include <social_navigation_layers/CustomLayerDynamicConfig.h>

double boat_gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double boat_get_radius(double cutoff, double A, double var);

namespace social_navigation_layers
{
  class CustomLayerDynamic : public SocialLayer
  {
    public:
      CustomLayerDynamic() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      void configure(CustomLayerDynamicConfig &config, uint32_t level);
      void interpVelCallback(const dynamic_reconfigure::Config& vel); 
      void predictedBoatPath(const nav_msgs::Path& path);
      void goalReached(const actionlib_msgs::GoalStatusArray& status);
      void predictedBoat();
      void timerCallback(const ros::TimerEvent&);
      void newGoal(const geometry_msgs::PoseStamped& goal);
      double cutoff_, amplitude_, covar_, factor_, interp_velocity_;
      bool received_path_;
      ros::Timer timer_;
      nav_msgs::Path current_path_;
      dynamic_reconfigure::Server<CustomLayerDynamicConfig>* server_;
      dynamic_reconfigure::Server<CustomLayerDynamicConfig>::CallbackType f_;
      ros::Subscriber interp_vel_sub_, path_sub_, status_sub_, goal_sub_;
      tf::TransformListener listener_;
      std::list<social_navigation_layers::Boat> moved_boats_;
  };
};
#endif