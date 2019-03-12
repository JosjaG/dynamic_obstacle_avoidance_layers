#include <social_navigation_layers/custom_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::CustomLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double boat_gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx),
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

double boat_get_radius(double cutoff, double A, double var){
    return sqrt(-2*var * log(cutoff/A) );
}

namespace social_navigation_layers
{
    void CustomLayer::onInitialize()
    {
        SocialLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        received_path_ = false;
        server_ = new dynamic_reconfigure::Server<CustomLayerConfig>(nh);
        f_ = boost::bind(&CustomLayer::configure, this, _1, _2);
        interp_vel_sub_ = g_nh.subscribe("interpolator/parameter_updates", 1, &CustomLayer::interpVelCallback, this);
        path_sub_ = g_nh.subscribe("path", 1, &CustomLayer::predictedBoatPath, this);
        status_sub_ = g_nh.subscribe("move_base/status", 1, &CustomLayer::goalReached, this);
        server_->setCallback(f_);
    }

    void CustomLayer::interpVelCallback(const dynamic_reconfigure::Config& vel) {
        interp_velocity_ = vel.doubles[0].value;
    }

    void CustomLayer::predictedBoatPath(const nav_msgs::Path& path) {
      received_path_ = true;
      current_path_ = path;
    }

    void CustomLayer::goalReached(const actionlib_msgs::GoalStatusArray& status) {
      for (unsigned int i=0; i<status.status_list.size(); i++){
        if (status.status_list[i].status==3) // PENDING 0, ACTIVE 1,PREEMPTED 2,SUCCEEDED 3, ABORTED 4,REJECTED 5,PREEMPTING 6,RECALLING 7,RECALLED 8,LOST 9
          received_path_ = false;
      }
    }

    struct temp_boat {
      double dist;
      double time;
      double stay_time;
    };

    void CustomLayer::predictedBoat() {   // should, for each boat, remap it to the expected location
      // ROS_INFO("received_path_ is %d. \n", received_path_);
      double dist_boat_x, dist_boat_y, dist_boat, time_boat;
      tf::StampedTransform transform_d;
      std::string global_frame = layered_costmap_->getGlobalFrameID();
      moved_boats_.clear();
      if (received_path_) {
        ROS_INFO("received_path_ found true. \n");
        // for each point in the path, calculate the time dory needs to get there, then determine if the boat is in a specified radius of that point
        // if yes, place the boat in the cost map, otherwise move on to the next point. 

        // you only want each boat to be in the costmap maximally once
        // keep the one that is the biggest troublemaker (closest to the path)
        // make a temp array with the closest distance of each boat to the path, -1 if the boat is not yet in the costmap

        struct temp_boat temp_boats[boats_list_.boats.size()];
        geometry_msgs::Point predicted_boat;
        double max_d, temp_dist_boat;
        for (unsigned int i=0; i<current_path_.poses.size(); i+=(current_path_.poses.size()/10)) {
            try {
                listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
                listener_.lookupTransform("map", "base_link", ros::Time(0), transform_d);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            dist_boat_x = current_path_.poses[i].pose.position.x - transform_d.getOrigin().x();
            dist_boat_y = current_path_.poses[i].pose.position.y - transform_d.getOrigin().y();
            dist_boat = sqrt(pow(dist_boat_x, 2) + pow(dist_boat_y, 2));
            time_boat = dist_boat/interp_velocity_;
            for (unsigned int j=0; j<boats_list_.boats.size(); j++) {
              if (i==0) {
                temp_boats[j].dist = -1.0;
              }
              social_navigation_layers::Boat& boat = boats_list_.boats[j];
              max_d = 0.75*std::max(boat.size.x, boat.size.y);
              predicted_boat.x = boat.position.x + time_boat*boat.velocity.x;
              predicted_boat.y = boat.position.y + time_boat*boat.velocity.y;
              temp_dist_boat = sqrt(pow(predicted_boat.x-current_path_.poses[i].pose.position.x, 2) + pow(predicted_boat.y-current_path_.poses[i].pose.position.y, 2));
              ROS_INFO("Distance from boat %d to path is %f, length of path is %d. \n", j, temp_dist_boat, current_path_.poses.size());
              if (temp_dist_boat<max_d) {
                temp_boats[j].time = time_boat;
                ROS_INFO("Boat is close, distance to path is %f. \n", temp_dist_boat);
                if (temp_boats[j].dist>0.0)
                  temp_boats[j].dist = std::min(temp_dist_boat, temp_boats[j].dist);
                else
                  temp_boats[j].dist = temp_dist_boat;
              }
            }
        }
        for (unsigned int i=0; i<boats_list_.boats.size(); i++) { 
          social_navigation_layers::Boat& boat = boats_list_.boats[i];
          social_navigation_layers::Boat tpt;
          geometry_msgs::PointStamped pt, opt;
            if (temp_boats[i].dist>0.0){          
              try{
                pt.point.x = boat.position.x + temp_boats[i].time*boat.velocity.x;
                pt.point.y = boat.position.y + temp_boats[i].time*boat.velocity.y;
                pt.point.z = boat.position.z;
                pt.header.frame_id = boats_list_.header.frame_id;
                tf_.transformPoint(global_frame, pt, opt);
                tpt.position = opt.point;

                tpt.velocity = boat.velocity;
                tpt.size = boat.size;
                tf_.transformPoint(global_frame, pt, opt);
                
                moved_boats_.push_back(tpt);
              }
              catch(tf::LookupException& ex) {
                ROS_ERROR("No Transform available Error: %s\n", ex.what());
                continue;
              }
              ROS_INFO("Distance to path within if is %f. \n", temp_boats[i].dist);  
            }
        }
      // Based on path topic, pridict where boats will be whilst Dory follows the path
      // based on segements of the path? Always split path in n (10?) segements, and predict 
      // Only look at boats with a velocity in the direction of your path?
      }
      else {
        for(unsigned int i=0; i<boats_list_.boats.size(); i++){
            social_navigation_layers::Boat& boat = boats_list_.boats[i];
            // position boat wrt dory --> distance between boat and dory
            try {
                listener_.waitForTransform("base_link", boat.id, ros::Time(0), ros::Duration(1.0) );
                listener_.lookupTransform("base_link", boat.id, ros::Time(0), transform_d);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            ROS_INFO("Distance of transform in x is %f. \n", transform_d.getOrigin().x());
            dist_boat_x = transform_d.getOrigin().x();
            dist_boat_y = transform_d.getOrigin().y();
            dist_boat = sqrt(pow(dist_boat_x, 2) + pow(dist_boat_y, 2));
            // time to boat --> distance between boat and dory / interp_velocity_
            time_boat = dist_boat/interp_velocity_;
            // predicted distance boat will travel in given time --> time to boat * velocity of boat
            // new location of boat wrt map (so move boat predicted distance in velocity direction)
            // result is new list of boats with adjusted locations
            social_navigation_layers::Boat tpt;
            geometry_msgs::PointStamped pt, opt;
            try{
              pt.point.x = boat.position.x + time_boat*boat.velocity.x;
              pt.point.y = boat.position.y + time_boat*boat.velocity.y;
              pt.point.z = boat.position.z;
              pt.header.frame_id = boats_list_.header.frame_id;
              tf_.transformPoint(global_frame, pt, opt);
              tpt.position = opt.point;

              tpt.velocity = boat.velocity;
              tpt.size = boat.size;
              tf_.transformPoint(global_frame, pt, opt);
              
              moved_boats_.push_back(tpt);
            }
            catch(tf::LookupException& ex) {
              ROS_ERROR("No Transform available Error: %s\n", ex.what());
              continue;
            }
        }
      }
      ROS_INFO("Number of boats ini the moved_boats list: %i. \n", moved_boats_.size());
    }

    void CustomLayer::updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::list<social_navigation_layers::Boat>::iterator p_it;
        CustomLayer::predictedBoat();
        // ROS_INFO("Received time to boat is %f. \n", CustomLayer::predictedBoat(1.0,2.0));

        for(p_it = moved_boats_.begin(); p_it != moved_boats_.end(); ++p_it){
            social_navigation_layers::Boat boat = *p_it;

            double mag = sqrt(pow(boat.velocity.x,2) + pow(boat.velocity.y, 2));
            double factor = 1.0 + mag * factor_;
            double point = boat_get_radius(cutoff_, amplitude_, covar_ * factor );

            *min_x = std::min(*min_x, boat.position.x - point);
            *min_y = std::min(*min_y, boat.position.y - point);
            *max_x = std::max(*max_x, boat.position.x + point);
            *max_y = std::max(*max_y, boat.position.y + point);

        }
    }

    void CustomLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(!enabled_) return;

        if( boats_list_.boats.size() == 0 )
          return;
        if( cutoff_ >= amplitude_)
          return;

        std::list<social_navigation_layers::Boat>::iterator p_it;
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();

        for(p_it = moved_boats_.begin(); p_it != moved_boats_.end(); ++p_it){
            social_navigation_layers::Boat boat = *p_it;
            double angle = atan2(boat.velocity.y, boat.velocity.x);
            double mag = sqrt(pow(boat.velocity.x,2) + pow(boat.velocity.y, 2));
            double factor = 1.0 + mag * factor_;
            double boat_size = sqrt(pow(boat.size.x, 2) + pow(boat.size.y, 2));
            double speed_factor = boat_size + sqrt(pow(boat.velocity.x, 2) + pow(boat.velocity.y, 2));
            double base = boat_get_radius(cutoff_, amplitude_, covar_) + boat_size;
            double point = boat_get_radius(cutoff_, amplitude_, covar_ * factor) + boat_size;

            // unsigned int width = std::max(1, int( (base + point) / res )),
            //               height = std::max(1, int( (base + point) / res ));

            unsigned int width = int( (base + point) / res ),
                          height = int( (base + point) / res );

            double cx = boat.position.x, cy = boat.position.y;

            double ox, oy;
            if(sin(angle)>0)
                oy = cy - base;
            else
                oy = cy + (point-base) * sin(angle) - base;

            if(cos(angle)>=0)
                ox = cx - base;
            else
                ox = cx + (point-base) * cos(angle) - base;

            int dx, dy;
            costmap->worldToMapNoBounds(ox, oy, dx, dy);

            int start_x = 0, start_y=0, end_x=width, end_y = height;
            if(dx < 0)
                start_x = -dx;
            else if(dx + width > costmap->getSizeInCellsX())
                end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

            if((int)(start_x+dx) < min_i)
                start_x = min_i - dx;
            if((int)(end_x+dx) > max_i)
                end_x = max_i - dx;

            if(dy < 0)
                start_y = -dy;
            else if(dy + height > costmap->getSizeInCellsY())
                end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

            if((int)(start_y+dy) < min_j)
                start_y = min_j - dy;
            if((int)(end_y+dy) > max_j)
                end_y = max_j - dy;

            double bx = ox + res / 2,
                   by = oy + res / 2;
            for(int i=start_x;i<end_x;i++){
                for(int j=start_y;j<end_y;j++){
                  unsigned char old_cost = costmap->getCost(i+dx, j+dy);
                  if(old_cost == costmap_2d::NO_INFORMATION)
                    continue;

                  double x = bx+i*res, y = by+j*res;
                  double ma = atan2(y-cy,x-cx);
                  double diff = angles::shortest_angular_distance(angle, ma);
                  double a;
                  if ((fabs(x-cx)<(boat.size.x/2.0)) && (fabs(y-cy)<(boat.size.y/2.0)))
                    a = costmap_2d::LETHAL_OBSTACLE;
                  else if(fabs(diff)<M_PI/2)
                    a = boat_gaussian(x,y,cx,cy,amplitude_,covar_*factor*speed_factor,covar_*speed_factor,angle);
                  else
                    a = boat_gaussian(x,y,cx,cy,amplitude_,covar_*speed_factor,covar_*speed_factor,0);

                  if(a < cutoff_)
                    continue;
                  unsigned char cvalue = (unsigned char) a;
                  costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));

              }
            }
        }
    }

    void CustomLayer::configure(CustomLayerConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        boats_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
    }
};