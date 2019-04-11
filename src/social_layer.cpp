#include <dynamic_obstacle_avoidance_layers/social_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace dynamic_obstacle_avoidance_layers
{
    void SocialLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;
        current_ = true;
        first_time_ = true;
        boats_sub_ = nh.subscribe("/boats", 1, &SocialLayer::boatsCallback, this);
    }
    
    void SocialLayer::boatsCallback(const dynamic_obstacle_avoidance_layers::Boats& boats) {
        boost::recursive_mutex::scoped_lock lock(lock_);
        boats_list_ = boats;
    }


    void SocialLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y){
        boost::recursive_mutex::scoped_lock lock(lock_);

        updateBoundsFromBoats(min_x, min_y, max_x, max_y);
        if(first_time_){
            last_min_x_ = *min_x;
            last_min_y_ = *min_y;    
            last_max_x_ = *max_x;
            last_max_y_ = *max_y;    
            first_time_ = false;
        }else{
            double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
            *min_x = std::min(last_min_x_, *min_x);
            *min_y = std::min(last_min_y_, *min_y);
            *max_x = std::max(last_max_x_, *max_x);
            *max_y = std::max(last_max_y_, *max_y);
            last_min_x_ = a;
            last_min_y_ = b;
            last_max_x_ = c;
            last_max_y_ = d;
        }
    }
};