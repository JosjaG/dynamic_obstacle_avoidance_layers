#include <social_navigation_layers/custom_layer_dynamic.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/Matrix3x3.h>
// #include <tf/transformations.h> //2_geometry_msgs/tf2_geometry_msgs.h>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::CustomLayerDynamic, costmap_2d::Layer)

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
    void CustomLayerDynamic::onInitialize()
    {
        SocialLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        received_path_ = false;
        server_ = new dynamic_reconfigure::Server<CustomLayerDynamicConfig>(nh);
        f_ = boost::bind(&CustomLayerDynamic::configure, this, _1, _2);
        interp_vel_sub_ = g_nh.subscribe("interpolator/parameter_updates", 1, &CustomLayerDynamic::interpVelCallback, this);
        path_sub_ = g_nh.subscribe("path", 1, &CustomLayerDynamic::predictedBoatPath, this);
        timer_ = g_nh.createTimer(ros::Duration(5.2), &CustomLayerDynamic::timerCallback, this);
        status_sub_ = g_nh.subscribe("move_base/status", 1, &CustomLayerDynamic::goalReached, this);
        goal_sub_ = g_nh.subscribe("move_base/current_goal", 1, &CustomLayerDynamic::newGoal, this);
        server_->setCallback(f_); 
    }

    void CustomLayerDynamic::interpVelCallback(const dynamic_reconfigure::Config& vel) {
        interp_velocity_ = vel.doubles[0].value;
    }

    void CustomLayerDynamic::predictedBoatPath(const nav_msgs::Path& path) {
      received_path_ = true;
      current_path_ = path;
    }

    void CustomLayerDynamic::goalReached(const actionlib_msgs::GoalStatusArray& status) {
      for (unsigned int i=0; i<status.status_list.size(); i++){
        if (status.status_list[i].status==3) // PENDING 0, ACTIVE 1,PREEMPTED 2,SUCCEEDED 3, ABORTED 4,REJECTED 5,PREEMPTING 6,RECALLING 7,RECALLED 8,LOST 9
          received_path_ = false;
      }
    }

    void CustomLayerDynamic::timerCallback(const ros::TimerEvent&) {
      boost::recursive_mutex::scoped_lock lock(lock_);
      moved_boats_.clear();
    }

    void CustomLayerDynamic::newGoal(const geometry_msgs::PoseStamped& goal) {
      boost::recursive_mutex::scoped_lock lock(lock_);
      moved_boats_.clear();
    }

    struct temp_boat {
      double dist;
      double time;
    };

    void CustomLayerDynamic::predictedBoat() {   // should, for each boat, remap it to the expected location
      // double time_start = ros::Time::now().toSec();
      // ROS_INFO("received_path_ is %d. \n", received_path_);
      double dist_boat_x, dist_boat_y, dist_boat, time_boat;
      tf::StampedTransform transform_d;

      std::string global_frame = layered_costmap_->getGlobalFrameID();
      if (received_path_) {
        // for each point in the path, calculate the time dory needs to get there, then determine if the boat is in a specified radius of that point
        // if yes, place the boat in the cost map, otherwise move on to the next point. 

        // you only want each boat to be in the costmap maximally once
        // keep the one that is the biggest troublemaker (closest to the path)
        // make a temp array with the closest distance of each boat to the path, -1 if the boat is not yet in the costmap

        struct temp_boat temp_boats[boats_list_.boats.size()];
        geometry_msgs::Pose predicted_boat;
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
              double boat_vel = sqrt(pow(boat.velocity.x, 2) + pow(boat.velocity.y, 2));
              if (boat_vel>0.1) {
                max_d = 0.75*std::max(boat.size.x, boat.size.y);
                predicted_boat.position.x = boat.pose.position.x + time_boat*boat.velocity.x;
                predicted_boat.position.y = boat.pose.position.y + time_boat*boat.velocity.y;
                temp_dist_boat = sqrt(pow(predicted_boat.position.x-current_path_.poses[i].pose.position.x, 2) + pow(predicted_boat.position.y-current_path_.poses[i].pose.position.y, 2));
                // ROS_INFO("Distance from boat %d to path is %f, length of path is %d. \n", j, temp_dist_boat, current_path_.poses.size());
                if (temp_dist_boat<max_d) {
                  temp_boats[j].time = time_boat;
                  // ROS_INFO("Boat is close, distance to path is %f. \n", temp_dist_boat);
                  if (temp_boats[j].dist>0.0)
                    temp_boats[j].dist = std::min(temp_dist_boat, temp_boats[j].dist);
                  else
                    temp_boats[j].dist = temp_dist_boat;
                }
              }
            }
        }
        for (unsigned int i=0; i<boats_list_.boats.size(); i++) { 
          social_navigation_layers::Boat& boat = boats_list_.boats[i];
          double boat_vel = sqrt(pow(boat.velocity.x, 2) + pow(boat.velocity.y, 2));
          if (boat_vel>0.1) {
            social_navigation_layers::Boat tpt;
            geometry_msgs::PoseStamped pt, opt;
              if (temp_boats[i].dist>0.0){          
                try{
                  pt.pose.position.x = boat.pose.position.x + temp_boats[i].time*boat.velocity.x;
                  pt.pose.position.y = boat.pose.position.y + temp_boats[i].time*boat.velocity.y;
                  pt.pose.position.z = boat.pose.position.z;
                  pt.pose.orientation = boat.pose.orientation;
                  pt.header.frame_id = boats_list_.header.frame_id;
                  tf_.transformPose(global_frame, pt, opt);
                  tpt.pose = opt.pose;

                  tpt.velocity = boat.velocity;
                  tpt.size = boat.size;
                  tf_.transformPose(global_frame, pt, opt);
                  
                  moved_boats_.push_back(tpt);
                }
                catch(tf::LookupException& ex) {
                  ROS_ERROR("No Transform available Error: %s\n", ex.what());
                  continue;
                }
                // ROS_INFO("Distance to path within if is %f. \n", temp_boats[i].dist);  
              }
          }
        }
      // Based on path topic, pridict where boats will be whilst Dory follows the path
      // based on segements of the path? Always split path in n (10?) segements, and predict 
      // Only look at boats with a velocity in the direction of your path?
      }
      else {
        for(unsigned int i=0; i<boats_list_.boats.size(); i++){
          social_navigation_layers::Boat& boat = boats_list_.boats[i];
          double boat_vel = sqrt(pow(boat.velocity.x, 2) + pow(boat.velocity.y, 2));
          if (boat_vel>0.1) {
          // position boat wrt dory --> distance between boat and dory
            try {
                listener_.waitForTransform("base_link", boat.id, ros::Time(0), ros::Duration(1.0) );
                listener_.lookupTransform("base_link", boat.id, ros::Time(0), transform_d);
            } catch (tf::TransformException ex) {
                ROS_ERROR("%s",ex.what());
            }
            // ROS_INFO("Distance of transform in x is %f. \n", transform_d.getOrigin().x());
            dist_boat_x = transform_d.getOrigin().x();
            dist_boat_y = transform_d.getOrigin().y();
            dist_boat = sqrt(pow(dist_boat_x, 2) + pow(dist_boat_y, 2));
            // time to boat --> distance between boat and dory / interp_velocity_
            time_boat = dist_boat/interp_velocity_;
            // predicted distance boat will travel in given time --> time to boat * velocity of boat
            // new location of boat wrt map (so move boat predicted distance in velocity direction)
            // result is new list of boats with adjusted locations
            social_navigation_layers::Boat tpt;
            // geometry_msgs::PoseStamped pt, opt;
            try{
              tpt.pose.position.x = boat.pose.position.x + time_boat*boat.velocity.x;
              tpt.pose.position.y = boat.pose.position.y + time_boat*boat.velocity.y;
              tpt.pose.position.z = boat.pose.position.z;
              tpt.pose.orientation = boat.pose.orientation;
              // tpt.header.frame_id = boats_list_.header.frame_id;
              // tf_.transformPose(global_frame, pt, opt);
              // tpt.pose = opt.pose;

              tpt.velocity = boat.velocity;
              tpt.size = boat.size;
              // tf_.transformPose(global_frame, pt, opt);
              
              moved_boats_.push_back(tpt);
            }
            catch(tf::LookupException& ex) {
              ROS_ERROR("No Transform available Error: %s\n", ex.what());
              continue;
            }
          }
        }
      }
      // ROS_INFO("Number of boats in the moved_boats list: %i. \n", moved_boats_.size());
      // ROS_INFO("Time  CustomLayerDynamic::predictedBoat() = %f. \n", (ros::Time::now().toSec() - time_start));
    }

    void CustomLayerDynamic::updateBoundsFromBoats(double* min_x, double* min_y, double* max_x, double* max_y)
    {
      std::list<social_navigation_layers::Boat>::iterator p_it;
      CustomLayerDynamic::predictedBoat();
      // ROS_INFO("Received time to boat is %f. \n", CustomLayer::predictedBoat(1.0,2.0));

      for(p_it = moved_boats_.begin(); p_it != moved_boats_.end(); ++p_it){
        social_navigation_layers::Boat boat = *p_it;

        // double mag = sqrt(pow(boat.velocity.x,2) + pow(boat.velocity.y, 2));
        // double factor = 1.0 + mag * factor_;
        // double point = 0.5*std::max(boat.size.x, boat.size.y)*boat_get_radius(cutoff_, amplitude_, covar_ * factor );
        double boat_size = sqrt(pow(boat.size.x, 2) + pow(boat.size.y, 2));

        *min_x = std::min(*min_x, boat.pose.position.x - 0.75 * boat_size);
        *min_y = std::min(*min_y, boat.pose.position.y - 0.75 * boat_size);
        *max_x = std::max(*max_x, boat.pose.position.x + 0.75 * boat_size);
        *max_y = std::max(*max_y, boat.pose.position.y + 0.75 * boat_size);
      }
    }

    void CustomLayerDynamic::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
      double time_start = ros::Time::now().toSec();
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
        double boat_size = sqrt(pow(boat.size.x, 2) + pow(boat.size.y, 2));
        double base = boat_get_radius(cutoff_, amplitude_, covar_) + boat_size;
        double point = boat_get_radius(cutoff_, amplitude_, covar_ * (sqrt(pow(boat.velocity.x,2) + pow(boat.velocity.y, 2)) * factor_)) + boat_size;

        unsigned int width = int( (base + point) / res ),
                      height = int( (base + point) / res );

        double cx = boat.pose.position.x, cy = boat.pose.position.y;

        double ox, oy;
        if(sin(angle)>0)
            oy = cy - base;
        else
            oy = cy + (point-base) * sin(angle) - base;

        if(cos(angle)>=0)
            ox = cx - base;
        else
            ox = cx + (point-base) * cos(angle) - base;

        // ROS_INFO("ox = %f, oy = %f. \n", ox, oy);
        ROS_INFO("width = %u, height = %u. \n", width, height);
        int dx, dy;
        costmap->worldToMapNoBounds(ox, oy, dx, dy);
        // ROS_INFO("dx = %u, dy = %u. \n", dx, dy);

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
        // ROS_INFO("size x = %u, size y = %u. \n", ((int)costmap->getSizeInCellsX()), ((int) costmap->getSizeInCellsY()));

        if((int)(start_y+dy) < min_j)
            start_y = min_j - dy;
        if((int)(end_y+dy) > max_j)
            end_y = max_j - dy;
        // ROS_INFO("min j = %u, max j = %u. \n", min_j, max_j);
      ROS_INFO("ID = %s, Start x = %d, end_x = %d, start_y = %d, end_y = %d. \n", boat.id.c_str(), start_x, end_x, start_y, end_y); 

        double bx = ox + res / 2,
               by = oy + res / 2;


        double long_side = sqrt(pow(boat.size.x/2, 2) + pow(boat.size.y/2, 2));
        double angle_orientation = atan2(boat.size.y/2, boat.size.x/2);
        double angle_calc[2];
        double roll, pitch, yaw;
        geometry_msgs::Quaternion q = boat.pose.orientation;
        tf::Quaternion tfq;
        tf::quaternionMsgToTF(q, tfq);
        tf::Matrix3x3(tfq).getEulerYPR(yaw,pitch,roll);
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
        // ROS_INFO("dot_AB = %f, dot_BC = %f. \n", dot_AB, dot_BC);
        // ROS_INFO("end_x = %d, end_y = %d. \n", end_x, end_y);
        for(int i=start_x;i<end_x;i++){
          for(int j=start_y;j<end_y;j++){
            unsigned char old_cost = costmap->getCost(i+dx, j+dy);
            if(old_cost == costmap_2d::NO_INFORMATION)
              continue;

            double x = bx+i*res, y = by+j*res;
            double ma = atan2(y-cy,x-cx);
            double diff = angles::shortest_angular_distance(angle, ma);
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
            } else if(fabs(diff)<M_PI/2)
              a = boat_gaussian(x,y,cx,cy,amplitude_,covar_*factor_*boat_size,covar_*factor_*boat_size,angle);
            else //gaussian distribution not in velocity direction
              a = boat_gaussian(x,y,cx,cy,amplitude_,covar_*boat.size.x,covar_*boat.size.y,yaw);

            if(a < cutoff_)
              continue;
            unsigned char cvalue = (unsigned char) a;
            costmap->setCost(i+dx, j+dy, cvalue); // std::max(cvalue, old_cost));
          }
        }
      }
      ROS_INFO("Time  CustomLayerDynamic::updateCosts = %f. \n", (ros::Time::now().toSec() - time_start));
    }

    void CustomLayerDynamic::configure(CustomLayerDynamicConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        boats_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
    }
};