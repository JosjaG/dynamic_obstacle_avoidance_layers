#include <social_navigation_layers/custom_layer_static.h>
#include <math.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <algorithm>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::CustomLayerStatic, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace social_navigation_layers
{
  int CustomLayerStatic::search(social_navigation_layers::Boat& boat_in){
    std::string id = boat_in.id;
    std::vector<static_obstacle_>::iterator result = std::find_if(
      static_obstacles_.begin(),
      static_obstacles_.end(),
      [id](const static_obstacle_& s) { return s.boat.id == id; }
    );
    if (result!=static_obstacles_.end()) {
      result->received = ros::Time::now();
      result->boat = boat_in;
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
    // ROS_INFO("Checking if any obstacle can be removed. \n");
    if (static_obstacles_.size() > 0)
      CustomLayerStatic::removeOldObstacles();
  }

  void CustomLayerStatic::removeOldObstacles() {
    double time_start = ros::Time::now().toSec();
    double tolerance = 0.5;
    boost::shared_ptr<sensor_msgs::LaserScan const> laser_msg;
    laser_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan",ros::Duration(0.5));
    // if(laser_msg != NULL){
    //   laser_data = *laser_msg; 
    // }
    ROS_INFO("Time after getting laser data = %f. \n", (ros::Time::now().toSec() - time_start));
    if (laser_msg->ranges.size() > 0) {
      tf::StampedTransform transform_d;
      std::vector<static_obstacle_> static_obstacles_keep;
      int buffer = laser_msg->ranges.size()/100;
      std::vector<static_obstacle_>::iterator o_it;
      for(o_it = static_obstacles_.begin(); o_it != static_obstacles_.end(); ++o_it) {
        struct static_obstacle_ obstacle = *o_it;
        try {
          listener_.waitForTransform("map", obstacle.boat.id, ros::Time(0), ros::Duration(1.0));
          listener_.lookupTransform("map", obstacle.boat.id, ros::Time(0), transform_d);
        } catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
        }
        double transform_length = sqrt(pow(transform_d.getOrigin().x(), 2.0) + pow(transform_d.getOrigin().y(), 2.0));
        int lidar_index = obstacle.boat.lidar_index;
        buffer = buffer + laser_msg->range_max/laser_msg->ranges[lidar_index];
        for (int i=(lidar_index - buffer); i < (lidar_index + buffer); ++i) {
          if (fabs(laser_msg->ranges[i] - (transform_length - std::min(obstacle.boat.size.x, obstacle.boat.size.y))) < tolerance) {
            static_obstacles_keep.push_back(obstacle);
            break;
          }
        }
      ROS_INFO("Time after lidar loop = %f. \n", (ros::Time::now().toSec() - time_start));
      }
      static_obstacles_.clear();
      std::vector<static_obstacle_>::iterator s_it;
      for(s_it = static_obstacles_keep.begin(); s_it != static_obstacles_keep.end(); ++s_it) {
        struct static_obstacle_ obstacle = *s_it;
        static_obstacles_.push_back(obstacle);
      }
    }
    ROS_INFO("Time  CustomLayerStatic::removeOldObstacles() end = %f. \n", (ros::Time::now().toSec() - time_start));
  }

  void CustomLayerStatic::filterStatic() {
    // double time_start = ros::Time::now().toSec();
    for (unsigned int i=0; i<boats_list_.boats.size(); i++) { 
      social_navigation_layers::Boat& boat = boats_list_.boats[i];
      double boat_vel = sqrt(pow(boat.velocity.x, 2) + pow(boat.velocity.y, 2));
      if (boat_vel<0.1) {
        if (CustomLayerStatic::search(boat)==-1) {
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
    ROS_INFO("Size within filter = %lu. \n", static_obstacles_.size());
    for(o_it = static_obstacles_.begin(); o_it != static_obstacles_.end(); ++o_it) {
      struct static_obstacle_ obstacle = *o_it;
      social_navigation_layers::Boat tpt;
      geometry_msgs::PoseStamped pt, opt;
      std::string global_frame = layered_costmap_->getGlobalFrameID();
      if (ros::Time::now().toSec()-obstacle.received.toSec()<static_keep_time_.toSec()) {  
        try{
          pt.pose = obstacle.boat.pose;
          pt.header.frame_id = boats_list_.header.frame_id;
          tf_.transformPose(global_frame, pt, opt);
          tpt.pose = opt.pose;
          tpt.size = obstacle.boat.size;
          tf_.transformPose(global_frame, pt, opt);
          
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
    // ROS_INFO("Time  CustomLayerStatic::filterStatic() = %f. \n", (ros::Time::now().toSec() - time_start));
  }

  void CustomLayerStatic::updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y) {
    std::list<social_navigation_layers::Boat>::iterator p_it;
    CustomLayerStatic::filterStatic();

    for(p_it = static_boats_.begin(); p_it != static_boats_.end(); ++p_it) {
      social_navigation_layers::Boat boat = *p_it;

      *min_x = std::min(*min_x, boat.pose.position.x - 3 * boat.size.x);
      *min_y = std::min(*min_y, boat.pose.position.y - 3 * boat.size.y);
      *max_x = std::max(*max_x, boat.pose.position.x + 3 * boat.size.x);
      *max_y = std::max(*max_y, boat.pose.position.y + 3 * boat.size.y);
    }
  }

  void CustomLayerStatic::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
    double time_start = ros::Time::now().toSec();
    boost::recursive_mutex::scoped_lock lock(lock_);
    if(!enabled_) return;
    std::list<social_navigation_layers::Boat>::iterator p_it;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = costmap->getResolution();
    // ROS_INFO("Size within updater = %lu. \n", static_boats_.size());

    for(p_it = static_boats_.begin(); p_it != static_boats_.end(); ++p_it) {
      social_navigation_layers::Boat boat = *p_it;

      double cx = boat.pose.position.x;
      double cy = boat.pose.position.y;
      // ROS_INFO("X position boat = %f. \n", cx);

      double roll, pitch, yaw;
      geometry_msgs::Quaternion q = boat.pose.orientation;
      tf::Quaternion tfq;
      tf::quaternionMsgToTF(q, tfq);
      tf::Matrix3x3(tfq).getEulerYPR(yaw,pitch,roll);
      double boat_size = sqrt(pow(boat.size.x, 2) + pow(boat.size.y, 2));

      double ox, oy;
      if(sin(yaw)>0.0)
          oy = cy - boat_size;
      else
          oy = cy + (boat_size) * sin(yaw) - boat_size;

      if(cos(yaw)>=0.0)
          ox = cx - boat_size;
      else
          ox = cx + (boat_size) * cos(yaw) - boat_size;

      int dx, dy;
      costmap->worldToMapNoBounds(ox, oy, dx, dy);

      int start_x = 0, start_y=0, end_x=(3 * boat_size) / res, end_y = (3 * boat_size) / res;

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

      double bx = ox + res / 2, by = oy + res / 2;

      double long_side = sqrt(pow(boat.size.x/2, 2) + pow(boat.size.y/2, 2));
      double angle_orientation = atan2(boat.size.y/2, boat.size.x/2);
      double angle_calc[2];
      angle_calc[0] = yaw - angle_orientation; //0
      angle_calc[1] = M_PI/2 - yaw - angle_orientation; //1 
      double dist_y_0 = long_side * std::sin(angle_calc[0]);
      double dist_x_0 = long_side * std::cos(angle_calc[0]);
      double dist_y_1 = long_side * std::cos(angle_calc[1]);
      double dist_x_1 = long_side * std::sin(angle_calc[1]);

      // Assuming the rectangle is represented by three points A,B,C, with AB and BC perpendicular, you only need to check
      // the projections of the query point M on AB and BC:

      // 0 <= dot(AB,AM) <= dot(AB,AB) &&
      // 0 <= dot(BC,BM) <= dot(BC,BC)

      // AB is vector AB, with coordinates (Bx-Ax,By-Ay), and dot(AB,AM) is the dot product of vectors AB and AM: ABx*AMx+ABy*AMy.
      
      double point0[2], point1[2], point2[2];
      point0[0] = cx + dist_x_0;
      point0[1] = cy + dist_y_0;
      point1[0] = cx + dist_x_1;
      point1[1] = cy + dist_y_1;
      point2[0] = cx - dist_x_0;
      point2[1] = cy - dist_y_0;
      double AB[2], BC[2];
      AB[0] = point1[0] - point0[0];
      AB[1] = point1[1] - point0[1];
      BC[0] = point2[0] - point1[0];
      BC[1] = point2[1] - point1[1];
      double dot_AB, dot_BC;
      dot_AB = AB[0]*AB[0] + AB[1]*AB[1];
      dot_BC = BC[0]*BC[0] + BC[1]*BC[1];
      for(int i=start_x;i<end_x;i++) {
        for(int j=start_y;j<end_y;j++) {
          unsigned char old_cost = costmap->getCost(i+dx, j+dy);
          if(old_cost == costmap_2d::NO_INFORMATION)
            continue;

          double x = bx+i*res, y = by+j*res;
          double a;

          double AP[2], BP[2];
          AP[0] = x - point0[0];
          AP[1] = y - point0[1];
          BP[0] = x - point1[0];
          BP[1] = y - point1[1];
          double dot_ABAP, dot_BCBP;
          dot_ABAP = AB[0]*AP[0] + AB[1]*AP[1];
          dot_BCBP = BC[0]*BP[0] + BC[1]*BP[1];

          if ((0.0 < (dot_ABAP)) && ((dot_ABAP) < (dot_AB)) && (0.0 < (dot_BCBP)) && ((dot_BCBP) < (dot_BC))) {
            // ROS_INFO("dot_ABAP = %f, dot_BCBP = %f. \n", dot_ABAP, dot_BCBP);
            a = costmap_2d::LETHAL_OBSTACLE;
          } else {
            a = costmap_2d::FREE_SPACE;
          }

          unsigned char cvalue = (unsigned char) a;
          costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));
        }
      }
    }
    ROS_INFO("Time  CustomLayerStatic::updateCosts() = %f. \n", (ros::Time::now().toSec() - time_start));
  }

  void CustomLayerStatic::configure(CustomLayerStaticConfig &config, uint32_t level) {
    static_keep_time_ = ros::Duration(60*60*config.keep_time);
    enabled_ = config.enabled;
  }
};