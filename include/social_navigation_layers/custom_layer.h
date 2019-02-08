#ifndef CUSTOM_LAYER_H_
#define CUSTOM_LAYER_H_
#include <ros/ros.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/CustomLayerConfig.h>

double boat_gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double boat_get_radius(double cutoff, double A, double var);

namespace social_navigation_layers
{
  class CustomLayer : public SocialLayer
  {
    public:
      CustomLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      void configure(CustomLayerConfig &config, uint32_t level);
      void interpVelCallback(const dynamic_reconfigure::Config& vel); 
      void predictedBoat();
      double cutoff_, amplitude_, covar_, factor_, interp_velocity_;
      dynamic_reconfigure::Server<CustomLayerConfig>* server_;
      dynamic_reconfigure::Server<CustomLayerConfig>::CallbackType f_;
      ros::Subscriber interp_vel_sub_;
      tf::TransformListener listener_;
      std::list<people_msgs::Person> moved_boats_;
  };
};
#endif