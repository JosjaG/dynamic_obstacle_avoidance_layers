#include <social_navigation_layers/boats_from_lidar.h>

const char * matchBoats(social_navigation_layers::Boat boat) {
    if (prev_boats_.size() != 0) {
        std::vector<social_navigation_layers::Boat>::iterator o_it;
        for(o_it = prev_boats_.begin(); o_it != prev_boats_.end(); ++o_it) {
            social_navigation_layers::Boat prev_boat = *o_it;
            if (abs(boat.lidar_index - prev_boat.lidar_index) < near_range) {
                const char *str = prev_boat.id.c_str();
                return str;
            } else if (fabs(boat.pose.position.x - prev_boat.pose.position.x) < (0.5*max_jump) && fabs(boat.pose.position.y - prev_boat.pose.position.y) < (0.5*max_jump)){
                const char *str = prev_boat.id.c_str();
                return str;
            }
        }
    }
    return "new";
}

void filterBoats() {
    //ROS_INFO("In filter boats. \n");
    detected_boats_.clear();
    //if ((obstacle_list[0].min_point.x == 0.0 ) && (obstacle_list[0].min_point.y == 0.0)) {  // first and last obstacle should be merged
    //    obstacle_list[0].min_point = obstacle_list.back().min_point;
    //    if ((pow(obstacle_list[0].min_point.x, 2)+pow(obstacle_list[0].min_point.y, 2)) > (pow(obstacle_list.back().min_point.x, 2)+pow(obstacle_list.back().min_point.y, 2)))
    //        obstacle_list[0].closest_point = obstacle_list.back().closest_point;
    //    obstacle_list.pop_back();
    //}
    //ROS_INFO("After merge. \n");
    std::vector<obstacle>::iterator o_it;
    for(o_it = obstacle_list.begin(); o_it != obstacle_list.end(); ++o_it) {
        tf::Transform transform;
        bool is_line = false;
        double angle;
        struct obstacle obstacle = *o_it;
        // Check if at least two of the three coordinates are marked as obstacles by the static map layer
        // In that case, pre-mapped obstacles are not also marked as new obstacles in the boats topic
        int a = (int) ((obstacle.min_point.x - map_->info.origin.position.x)/map_->info.resolution + 0.5) + (int) ((obstacle.min_point.y - map_->info.origin.position.y)/map_->info.resolution + 0.5) * map_->info.width;
        int b = (int) ((obstacle.closest_point.x - map_->info.origin.position.x)/map_->info.resolution + 0.5) + (int) ((obstacle.closest_point.y - map_->info.origin.position.y)/map_->info.resolution + 0.5) * map_->info.width;
        int c = (int) ((obstacle.max_point.x - map_->info.origin.position.x)/map_->info.resolution + 0.5) + (int) ((obstacle.max_point.y - map_->info.origin.position.y)/map_->info.resolution + 0.5) * map_->info.width;
	// ROS_INFO("Processing map data. \n");
        // ROS_INFO("a = %d, b = %d, c = %d, size map data = %d. \n", a, b, c, map_->data.size());
        bool not_static_obstacle = true; //should be initialized as false
        bool not_unknown_space = true; //should be initialized as false
        if (a > 0 && b > 0 && c > 0) {
            not_static_obstacle = (map_->data[a] != 100 || map_->data[b] != 100 || map_->data[c] != 100);
            not_unknown_space = (map_->data[a] != -1 || map_->data[b] != -1 || map_->data[c] != -1);
        } 
        ROS_INFO("Inspecting map data. \n");
        if (not_static_obstacle && not_unknown_space) { // || map_.data[a] == -1 || map_.data[b] == -1 || map_.data[c] == -1) {
          // break;
        // } else {
            social_navigation_layers::Boat boat;

            if (obstacle.closest_point.x == 0.0 && obstacle.closest_point.y == 0.0)
                obstacle.closest_point = obstacle.min_point;

            boat.pose.position.x = (obstacle.max_point.x + obstacle.min_point.x)/2;
            boat.pose.position.y = (obstacle.max_point.y + obstacle.min_point.y)/2;

            boat.lidar_index = obstacle.lidar_loc;
            if (strcmp(matchBoats(boat), "new") == 0)
                boat.id = "boat_" + std::to_string(rand() % 100000);
            else
                boat.id = matchBoats(boat);
            // ROS_INFO("Boat id before sending %s. \n", boat.id.c_str());

            // check if the three points form a rectangle or a line:
            double AB[2];
            AB[0] = obstacle.max_point.x - obstacle.min_point.x;
            AB[1] = obstacle.max_point.y - obstacle.min_point.y;
            double BC[2];
            BC[0] = obstacle.closest_point.x - obstacle.min_point.x;
            BC[1] = obstacle.closest_point.y - obstacle.min_point.y;
            double area = AB[0]*BC[1] - AB[1]*BC[0];
            ROS_INFO("area is %f. \n", area);
            double length = sqrt(pow((obstacle.min_point.x - obstacle.max_point.x), 2) + pow((obstacle.min_point.y - obstacle.max_point.y), 2));
            if (fabs(area) < (2.0*length))
                is_line = true;

            if (is_line) {
                ROS_INFO("Boat identified as line");
                angle = atan2(AB[1], AB[0]);
                if ((angle > -(M_PI/4.0) && angle < (M_PI/4.0)) || (angle > (3.0*M_PI/4.0) && angle < (-3.0*M_PI/4.0))) {
                    boat.size.x = length;
                    boat.size.y = (fabs(area)*2.0/length);
                } else {
                    boat.size.y = length;
                    boat.size.x = (fabs(area)*2.0/length);
                    angle+=(M_PI/2.0);
                }
            } else {
                angle = atan2((obstacle.min_point.y - obstacle.closest_point.y), (obstacle.min_point.x - obstacle.closest_point.x));
                boat.size.x = sqrt(pow((obstacle.min_point.x - obstacle.closest_point.x), 2) + pow((obstacle.min_point.y - obstacle.closest_point.y), 2));
                boat.size.y = sqrt(pow((obstacle.closest_point.x - obstacle.max_point.x), 2) + pow((obstacle.closest_point.y - obstacle.max_point.y), 2));
            }        
            boat.size.x = std::max(boat.size.x, min_size);
            boat.size.y = std::max(boat.size.y, min_size);
            transform.setOrigin(tf::Vector3(boat.pose.position.x, boat.pose.position.y, 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, (angle - (M_PI/2.0)));
            transform.setRotation(q);
            broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", boat.id));
            boat.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
            detected_boats_.push_back(boat);
        }
    }
    boats_list_.boats = detected_boats_;
    prev_boats_ = detected_boats_;
    boats_list_.header.frame_id = "map";
    boats_list_.header.stamp = ros::Time::now();
    boats_pub_.publish(boats_list_);
    ROS_INFO("Boat list length is %d. \n", detected_boats_.size());
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    bool in_object = false;
    struct obstacle temp_obstacle;
    obstacle_list.clear();
    tf::StampedTransform transform_d;
    try {
        listener_->waitForTransform("map", "lidar", ros::Time(0), ros::Duration(1.0));
        listener_->lookupTransform("map", "lidar", ros::Time(0), transform_d);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    double roll, pitch, yaw;
    tf::Matrix3x3(transform_d.getRotation()).getEulerYPR(yaw,pitch,roll);
    int num_scans = (fabs(scan->angle_min) + fabs(scan->angle_max))/scan->angle_increment;
    sensor_msgs::LaserScan lidar_filtered = *scan;
    lidar_filtered.ranges.clear();
    int min_point_lidar;
    if (scan->ranges[0] < scan->range_max) {
        in_object = true;
        temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw + (num_scans/2)*scan->angle_increment) * scan->ranges[0];
        temp_obstacle.min_point.y = transform_d.getOrigin().y() + sin(yaw + (num_scans/2)*scan->angle_increment) * scan->ranges[0];
        //double x_wrt_lidar = std::sin(scan->angle_min) * scan->ranges[0];
        //double y_wrt_lidar = std::cos(scan->angle_min) * scan->ranges[0];
        //temp_obstacle.min_point.x = transform_d.getOrigin().x() + x_wrt_lidar * std::sin(M_PI/2.0 - yaw);
        //temp_obstacle.min_point.y = transform_d.getOrigin().y() + y_wrt_lidar * std::cos(yaw);
        temp_obstacle.lidar_loc = 0;
        min_point_lidar = 0;
        lidar_filtered.ranges.push_back(scan->ranges[0]); 
    }
    ROS_INFO("Num scans  = %d, ranges size = %d. \n", num_scans, scan->ranges.size());
    ROS_INFO("Transform x = %f, transform y = %f, yaw = %f. \n", transform_d.getOrigin().x(), transform_d.getOrigin().y(), yaw);
    double closest_entity = scan->range_max;
    //int min_point_lidar; 
    for (int i=1; i < scan->ranges.size(); ++i) {
        if (in_object && (scan->ranges[i] < (scan->range_max - 1.0))) {
            if (scan->ranges[i] < closest_entity) {
                if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                    //temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.y = transform_d.getOrigin().y() + sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    //double x_wrt_lidar = std::sin(scan->angle_min + i*scan->angle_increment) * scan->ranges[i];
                    //double y_wrt_lidar = std::cos(scan->angle_min + i*scan->angle_increment) * scan->ranges[i];
                    //temp_obstacle.closest_point.x = transform_d.getOrigin().x() + x_wrt_lidar * std::sin(M_PI/2.0 - yaw);
                    //temp_obstacle.closest_point.y = transform_d.getOrigin().y() + y_wrt_lidar * std::cos(yaw);
                } else {
                    temp_obstacle.closest_point.x = transform_d.getOrigin().x() + cos(-yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    //temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.y = transform_d.getOrigin().y() - sin(-yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
                }
                closest_entity = scan->ranges[i];
                lidar_filtered.ranges.push_back(75);
            }
            if (fabs(scan->ranges[i+1] - scan->ranges[i]) > max_jump) {
                if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                    ROS_INFO("Should be setting max point. \n");
                    //temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.y = transform_d.getOrigin().y() + sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    //double x_wrt_lidar = std::sin(scan->angle_min + i*scan->angle_increment) * scan->ranges[i];
                    //double y_wrt_lidar = std::cos(scan->angle_min + i*scan->angle_increment) * scan->ranges[i];
                    //temp_obstacle.max_point.x = transform_d.getOrigin().x() + x_wrt_lidar * std::sin(M_PI/2.0 - yaw);
                    //temp_obstacle.max_point.y = transform_d.getOrigin().y() + y_wrt_lidar * std::cos(yaw);
                } else {
                    temp_obstacle.max_point.x = transform_d.getOrigin().x() + cos(-yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    //temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.y = transform_d.getOrigin().y() - sin(-yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
                }
                temp_obstacle.lidar_loc = (temp_obstacle.lidar_loc + i) / 2;
                bool obstacle_big_enough = ((scan->ranges[i]*((i - min_point_lidar)*scan->angle_increment)) > min_obstacle_size);
                if (((-scan->range_max < temp_obstacle.min_point.x && -scan->range_max < temp_obstacle.closest_point.x && -scan->range_max < temp_obstacle.max_point.x) &&
                 (temp_obstacle.min_point.x < (scan->range_max - 1.0) && temp_obstacle.closest_point.x < (scan->range_max - 1.0) && temp_obstacle.max_point.x < (scan->range_max - 1.0))) && obstacle_big_enough)
                //if ((temp_obstacle.min_point.x < scan->range_max && temp_obstacle.closest_point.x < scan->range_max && temp_obstacle.max_point.x < scan->range_max) && obstacle_big_enough)
                    obstacle_list.push_back(temp_obstacle);
                in_object = false;
                //ROS_INFO("Obstacle size is %f. \n", (scan->ranges[i]*((i - min_point_lidar)*scan->angle_increment)));
                //ROS_INFO("Obstacle big enough is %d. \n", obstacle_big_enough);
                //ROS_INFO("min_obstacle_size is %f. \n", min_obstacle_size);
                lidar_filtered.ranges.push_back(scan->ranges[i]);
                ROS_INFO("MIN POINT : x = %f, y = %f. \n", temp_obstacle.min_point.x, temp_obstacle.min_point.y);
                ROS_INFO("CLOSEST POINT : x = %f, y = %f. \n", temp_obstacle.closest_point.x, temp_obstacle.closest_point.y);
                ROS_INFO("MAX POINT : x = %f, y = %f. \n", temp_obstacle.max_point.x, temp_obstacle.max_point.y);
                temp_obstacle = {};
                closest_entity = scan->range_max;
            } else 
                lidar_filtered.ranges.push_back(75);
        } else if ((fabs(scan->ranges[i-1] - scan->ranges[i]) > max_jump) && (fabs(scan->ranges[i] - scan->ranges[i+1]) < max_jump)) {
            lidar_filtered.ranges.push_back(scan->ranges[i]);
            ROS_INFO("(num_scans/2 - i)*scan->angle_increment) = %f, (i - num_scans/2)*scan->angle_increment = %f. \n", (num_scans/2 - i)*scan->angle_increment, (i - num_scans/2)*scan->angle_increment);
            ROS_INFO("sin(1) = %f, sin(2) =  %f. \n", sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i], sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i]);
            if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                //temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.y = transform_d.getOrigin().y() + sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                //double x_wrt_lidar = std::sin(scan->angle_min + i*scan->angle_increment) * scan->ranges[i];
                //double y_wrt_lidar = std::cos(scan->angle_min + i*scan->angle_increment) * scan->ranges[i];
                //temp_obstacle.min_point.x = transform_d.getOrigin().x() + x_wrt_lidar * std::sin(M_PI/2.0 - yaw);
                //temp_obstacle.min_point.y = transform_d.getOrigin().y() + y_wrt_lidar * std::cos(yaw);
            } else {
                temp_obstacle.min_point.x = transform_d.getOrigin().x() + cos(-yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                //temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.y = transform_d.getOrigin().y() - sin(-yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
            }
            temp_obstacle.lidar_loc = i;
            min_point_lidar = i;
            in_object = true;
        } else
            lidar_filtered.ranges.push_back(75);
    }
    lidar_pub_.publish(lidar_filtered);
    ROS_INFO("Obstacle length is %d. \n", obstacle_list.size());
    if (obstacle_list.size() != 0)
        filterBoats();
}

//void mapCallback(const nav_msgs::OccupancyGrid& map) {
//    map_ = map;
//}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "lidar_to_boats");
  ros::NodeHandle node;
  if(ros::param::get("/dory/boats_from_lidar/min_size", min_size))
    ros::param::get("/dory/boats_from_lidar/min_size", min_size);
  else
    min_size = 0.3;
  if(ros::param::get("/dory/boats_from_lidar/max_jump", max_jump))
    ros::param::get("/dory/boats_from_lidar/max_jump", max_jump);
  else
    max_jump = 0.5;
  if(ros::param::get("/dory/boats_from_lidar/near_range", near_range))
    ros::param::get("/dory/boats_from_lidar/near_range", near_range);
  else
    near_range = 7;
  if(ros::param::get("/dory/boats_from_lidar/min_obstacle_size", min_obstacle_size))
    ros::param::get("/dory/boats_from_lidar/min_obstacle_size", min_obstacle_size);
  else
    min_obstacle_size = 0.5;
  // instantiate publishers & subscribers
  boats_pub_ = node.advertise<social_navigation_layers::Boats>("boats_detected", 1);
  laser_sub_ = node.subscribe("dory/scan", 1, lidarCallback);
  // map_sub_ = node.subscribe("dory/map", 1, mapCallback);
  lidar_pub_ = node.advertise<sensor_msgs::LaserScan>("/lidar_boat_point", 1);

  listener_ = new (tf::TransformListener);
  broadcaster_ = new (tf::TransformBroadcaster);

  //boost::shared_ptr<nav_msgs::OccupancyGrid const> map_;
  map_ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("dory/map",ros::Duration(0.5));
  double loop_rate = 2.0;
  ros::Rate rate(loop_rate);

  while (ros::ok()) {
    // Spin
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
