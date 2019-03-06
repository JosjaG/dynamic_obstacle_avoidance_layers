#include <social_navigation_layers/custom_layer_static.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <algorithm>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::CustomLayerStatic, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
  int CustomLayerStatic::search(std::string id){
    std::vector<static_obstacle_>::iterator result = std::find_if(
      static_obstacles_.begin(),
      static_obstacles_.end(),
      [id](const static_obstacle_& s) { return s.boat.id == id; }
    );
    if (result!=static_obstacles_.end()) {
      result->received = ros::Time::now();
      return 1;
    } else {
      return -1;
    }
  }

  void CustomLayerStatic::onInitialize() {
    SocialLayer::onInitialize();
    ros::NodeHandle nh("~/" + name_), g_nh;
    server_ = new dynamic_reconfigure::Server<CustomLayerStaticConfig>(nh);
    f_ = boost::bind(&CustomLayerStatic::configure, this, _1, _2);
    timer_ = nh.createTimer(ros::Duration(3.0), &CustomLayerStatic::timerCallback, this);
    server_->setCallback(f_);
  }

  void CustomLayerStatic::timerCallback(const ros::TimerEvent&) {
    boats_list_.boats.clear();
  }

  void CustomLayerStatic::filterStatic() {
    for (unsigned int i=0; i<boats_list_.boats.size(); i++) { 
      social_navigation_layers::Boat& boat = boats_list_.boats[i];
      double boat_vel = sqrt(pow(boat.velocity.x, 2) + pow(boat.velocity.y, 2));
      if (boat_vel==0.0) {
        if (CustomLayerStatic::search(boat.id)==-1) {
          struct static_obstacle_ obstacle;
          obstacle.boat = boat;
          obstacle.received = ros::Time::now();
          static_obstacles_.push_back(obstacle);
        }
      }
    }
    std::vector<static_obstacle_>::iterator o_it;   
    int it = 0;
    int count = 0;
    static_boats_.clear();
    for(o_it = static_obstacles_.begin(); o_it != static_obstacles_.end(); ++o_it) {
      struct static_obstacle_ obstacle = *o_it;
      social_navigation_layers::Boat tpt;
      geometry_msgs::PointStamped pt, opt;
      std::string global_frame = layered_costmap_->getGlobalFrameID();
      if (ros::Time::now().toSec()-obstacle.received.toSec()<static_keep_time_.toSec()) {  
        try{
          pt.point = obstacle.boat.position;
          pt.header.frame_id = boats_list_.header.frame_id;
          tf_.transformPoint(global_frame, pt, opt);
          tpt.position = opt.point;
          tpt.size = obstacle.boat.size;
          tf_.transformPoint(global_frame, pt, opt);
          
          static_boats_.push_back(tpt);
        }
        catch(tf::LookupException& ex) {
          ROS_ERROR("No Transform available Error: %s\n", ex.what());
          continue;
        }
      } else if (static_obstacles_.size() > 0) {
        std::swap(static_obstacles_[it], static_obstacles_.back());
        count++;
      }
      it++;
    }
    while (count>0) {
      static_obstacles_.pop_back();
      count--;
    }
  }

  void CustomLayerStatic::updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y) {
    std::list<social_navigation_layers::Boat>::iterator p_it;
    CustomLayerStatic::filterStatic();

    for(p_it = static_boats_.begin(); p_it != static_boats_.end(); ++p_it) {
      social_navigation_layers::Boat boat = *p_it;
      double point_x = boat.size.x/2.0;
      double point_y = boat.size.y/2.0;

      *min_x = std::min(*min_x, boat.position.x - point_x);
      *min_y = std::min(*min_y, boat.position.y - point_y);
      *max_x = std::max(*max_x, boat.position.x + point_x);
      *max_y = std::max(*max_y, boat.position.y + point_y);
    }
  }

  void CustomLayerStatic::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if(!enabled_) return;
    std::list<social_navigation_layers::Boat>::iterator p_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();

    for(p_it = static_boats_.begin(); p_it != static_boats_.end(); ++p_it) {
      social_navigation_layers::Boat boat = *p_it;

      double cx = boat.position.x, cy = boat.position.y;

      double ox, oy;
      oy = cy - boat.size.y/2.0;
      ox = cx - boat.size.x/2.0;

      int dx, dy;
      costmap->worldToMapNoBounds(ox, oy, dx, dy);

      int start_x = 0, start_y=0, end_x=boat.size.x/res, end_y = boat.size.y/res;
      
      if(dx < 0)
        start_x = -dx;
      else if(dx + boat.size.x > costmap->getSizeInCellsX())
        end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

      if((int)(start_x+dx) < min_i)
        start_x = min_i - dx;
      if((int)(end_x+dx) > max_i)
        end_x = max_i - dx;

      if(dy < 0)
        start_y = -dy;
      else if(dy + boat.size.y > costmap->getSizeInCellsY())
        end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

      if((int)(start_y+dy) < min_j)
        start_y = min_j - dy;
      if((int)(end_y+dy) > max_j)
        end_y = max_j - dy;

      double bx = ox + res / 2,
             by = oy + res / 2;

      for(int i=start_x;i<end_x;i++) {
        for(int j=start_y;j<end_y;j++) {
          unsigned char old_cost = costmap->getCost(i+dx, j+dy);
          if(old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx+i*res, y = by+j*res;
          double a;
          if ((fabs(x-cx)<(boat.size.x/2.0)) && (fabs(y-cy)<(boat.size.y/2.0))) {
            a = costmap_2d::LETHAL_OBSTACLE;
          }
          else {
            a = costmap_2d::FREE_SPACE;
          }

          unsigned char cvalue = (unsigned char) a;
          costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));
        }
      }
    }
  }

  void CustomLayerStatic::configure(CustomLayerStaticConfig &config, uint32_t level) {
    static_keep_time_ = ros::Duration(60*60*config.keep_time);
    enabled_ = config.enabled;
  }
};