#include <social_navigation_layers/boats_from_lidar.h>

const char * matchBoats(int lidar) {
    if (prev_boats_.size() != 0) {
        std::vector<social_navigation_layers::Boat>::iterator o_it;
        for(o_it = prev_boats_.begin(); o_it != prev_boats_.end(); ++o_it) {
            social_navigation_layers::Boat boat = *o_it;
            if (abs(lidar - boat.lidar_index) < near_range_) {
                const char *str = boat.id.c_str();
                return str;
            }
        }
    }
    return "new";
}

void filterBoats() {
    detected_boats_.clear();
    if ((obstacle_list[0].min_point.x == 0.0 ) && (obstacle_list[0].min_point.y == 0.0)) {  // first and last obstacle should be merged
        obstacle_list[0].min_point = obstacle_list.back().min_point;
        if ((pow(obstacle_list[0].min_point.x, 2)+pow(obstacle_list[0].min_point.y, 2)) > (pow(obstacle_list.back().min_point.x, 2)+pow(obstacle_list.back().min_point.y, 2)))
            obstacle_list[0].closest_point = obstacle_list.back().closest_point;
        obstacle_list.pop_back();
    }
    std::vector<obstacle>::iterator o_it;
    for(o_it = obstacle_list.begin(); o_it != obstacle_list.end(); ++o_it) {
        tf::Transform transform;
        bool is_line = false;
        double angle;
        struct obstacle obstacle = *o_it;
        // Check if at least two of the three coordinates are marked as obstacles by the static map layer
        // In that case, pre-mapped obstacles are not also marked as new obstacles in the boats topic
        int a = (int) ((obstacle.min_point.x - map_.info.origin.position.x)/map_.info.resolution + 0.5) + (int) ((obstacle.min_point.y - map_.info.origin.position.y)/map_.info.resolution + 0.5) * map_.info.width;
        int b = (int) ((obstacle.closest_point.x - map_.info.origin.position.x)/map_.info.resolution + 0.5) + (int) ((obstacle.closest_point.y - map_.info.origin.position.y)/map_.info.resolution + 0.5) * map_.info.width;
        int c = (int) ((obstacle.max_point.x - map_.info.origin.position.x)/map_.info.resolution + 0.5) + (int) ((obstacle.max_point.y - map_.info.origin.position.y)/map_.info.resolution + 0.5) * map_.info.width;
        if (map_.data[a] != 100 || map_.data[b] != 100 || map_.data[c] != 100) {
            social_navigation_layers::Boat boat;
            if (strcmp(matchBoats(obstacle.lidar_loc), "new") == 0)
                boat.id = "boat_" + std::to_string(rand() % 100000);
            else
                boat.id = matchBoats(obstacle.lidar_loc);
            boat.lidar_index = obstacle.lidar_loc;
            if (obstacle.closest_point.x == 0.0 && obstacle.closest_point.y == 0.0)
                obstacle.closest_point = obstacle.min_point;

            boat.pose.position.x = (obstacle.max_point.x + obstacle.min_point.x)/2;
            boat.pose.position.y = (obstacle.max_point.y + obstacle.min_point.y)/2;

            // check if the three points form a rectangle or a line:
            double AB[2];
            AB[0] = obstacle.max_point.x - obstacle.min_point.x;
            AB[1] = obstacle.max_point.y - obstacle.min_point.y;
            double BC[2];
            BC[0] = obstacle.closest_point.x - obstacle.min_point.x;
            BC[1] = obstacle.closest_point.y - obstacle.min_point.y;
            double area = AB[0]*BC[1] - AB[1]*BC[0];
            if (fabs(area) < 2.0) {
                is_line = true;
                ROS_INFO("id = %s. \n", boat.id.c_str());
            }

            if (is_line) {
                angle = atan2(AB[1], AB[0]);
                double length = sqrt(pow((obstacle.min_point.x - obstacle.max_point.x), 2) + pow((obstacle.min_point.y - obstacle.max_point.y), 2));
                if ((angle > -(M_PI/4.0) && angle < (M_PI/4.0)) || (angle > (3.0*M_PI/4.0) && angle < (-3.0*M_PI/4.0))) {
                    boat.size.x = length;
                } else {
                    boat.size.y = length;
                    angle+=(M_PI/2.0);
                }
            } else {
                angle = atan2((obstacle.min_point.y - obstacle.closest_point.y), (obstacle.min_point.x - obstacle.closest_point.x));
                boat.size.x = sqrt(pow((obstacle.min_point.x - obstacle.closest_point.x), 2) + pow((obstacle.min_point.y - obstacle.closest_point.y), 2));
                boat.size.y = sqrt(pow((obstacle.closest_point.x - obstacle.max_point.x), 2) + pow((obstacle.closest_point.y - obstacle.max_point.y), 2));
            }        
            boat.size.x = std::max(boat.size.x, 0.5);
            boat.size.y = std::max(boat.size.y, 0.5);
            transform.setOrigin(tf::Vector3(boat.pose.position.x, boat.pose.position.y, 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, (angle - (M_PI/2)));
            transform.setRotation(q);
            broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", boat.id));
            boat.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
            detected_boats_.push_back(boat);
        }
    }
    boats_list_.boats = detected_boats_;
    prev_boats_ = detected_boats_;
    boats_list_.header.frame_id = "map";
    boats_pub_.publish(boats_list_);
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
    int num_scans = (scan->angle_min + scan->angle_max)/scan->angle_increment;
    if (scan->ranges[0] < scan->range_max) {
        in_object = true;
    }
    double closest_entity = scan->range_max;
    for (int i=1; i < scan->ranges.size(); ++i) {
        if (in_object) {
            if (scan->ranges[i] < closest_entity) {
                if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                    temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.y = transform_d.getOrigin().y() - sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                } else {
                    temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.y = transform_d.getOrigin().y() - sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
                }
                closest_entity = scan->ranges[i];
            }
            if (fabs(scan->ranges[i+1] - scan->ranges[i]) > max_jump) {
                if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                    temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.y = transform_d.getOrigin().y() - sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                } else {
                    temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.y = transform_d.getOrigin().y() - sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
                }
                temp_obstacle.lidar_loc = (temp_obstacle.lidar_loc + i) / 2.0;
                if ((-scan->range_max < temp_obstacle.min_point.x && -scan->range_max < temp_obstacle.closest_point.x && -scan->range_max < temp_obstacle.max_point.x) &&
                 (temp_obstacle.min_point.x < scan->range_max && temp_obstacle.closest_point.x < scan->range_max && temp_obstacle.max_point.x < scan->range_max))
                    obstacle_list.push_back(temp_obstacle);
                // ROS_INFO("CLOS: scan_x = %f, scan_y = %f. \n", temp_obstacle.closest_point.x, temp_obstacle.closest_point.y);
                in_object = false;
                temp_obstacle = {};
                closest_entity = scan->range_max;
            }
        } else if (fabs(scan->ranges[i-1] - scan->ranges[i]) > max_jump) {
            if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.y = transform_d.getOrigin().y() - sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
            } else {
                temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.y = transform_d.getOrigin().y() - sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
            }
            ROS_INFO("MIN: scan_x = %f, scan_y = %f. \n", temp_obstacle.min_point.x, temp_obstacle.min_point.y);
            temp_obstacle.lidar_loc = i;
            in_object = true;
        }
    }
    if (obstacle_list.size() != 0)
        filterBoats();
}

void mapCallback(const nav_msgs::OccupancyGrid& map) {
    map_ = map;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "lidar_to_boats");
  ros::NodeHandle node;

  // instantiate publishers & subscribers
  boats_pub_ = node.advertise<social_navigation_layers::Boats>("/boats_detected", 1);
  map_sub_ = node.subscribe("clara/map", 1, mapCallback);
  laser_sub_ = node.subscribe("/clara/scan", 1, lidarCallback);

  listener_ = new (tf::TransformListener);
  broadcaster_ = new (tf::TransformBroadcaster);

  double loop_rate = 1.0;
  ros::Rate rate(loop_rate);

  while (ros::ok()) {
    // Spin
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
