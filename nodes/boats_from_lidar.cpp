#include <social_navigation_layers/boats_from_lidar.h>


// void updateVelocities() {
//     for ...

// }

void filterBoats() {
    tf::Transform transform;
    double angle;
    detected_boats_.clear();
    if ((obstacle_list[0].min_point.x < 0.1) && (obstacle_list[0].min_point.y < 0.1)) {  // first and last obstacle should be merged
        obstacle_list[0].min_point = obstacle_list.back().min_point;
        if ((pow(obstacle_list[0].min_point.x, 2)+pow(obstacle_list[0].min_point.y, 2)) > (pow(obstacle_list.back().min_point.x, 2)+pow(obstacle_list.back().min_point.y, 2)))
            obstacle_list[0].closest_point = obstacle_list.back().closest_point;
        obstacle_list.pop_back();
    }
    std::vector<obstacle>::iterator o_it;
    int count = 0;
    for(o_it = obstacle_list.begin(); o_it != obstacle_list.end(); ++o_it) {
        struct obstacle obstacle = *o_it;
        social_navigation_layers::Boat boat;
        boat.id = "boat_" + std::to_string(count);
        boat.pose.position.x = (obstacle.max_point.x + obstacle.min_point.x)/2;
        boat.pose.position.y = (obstacle.max_point.y + obstacle.min_point.y)/2;
        boat.size.x = std::max(sqrt(pow((obstacle.min_point.x - obstacle.closest_point.x), 2) + pow((obstacle.min_point.y - obstacle.closest_point.y), 2)), 0.5);
        boat.size.y = std::max(sqrt(pow((obstacle.closest_point.x - obstacle.max_point.x), 2) + pow((obstacle.closest_point.y - obstacle.max_point.y), 2)), 0.5);
        angle = atan2(obstacle.closest_point.y-obstacle.min_point.y, obstacle.closest_point.x-obstacle.min_point.x);
        while (angle > M_PI/2 || angle < 0) {
            if (angle < 0) {
                angle+=M_PI/2;
                double tmp_size = boat.size.x;
                boat.size.x = boat.size.y;
                boat.size.y = tmp_size;
            }
            else {
                angle-=M_PI/2;
                double tmp_size = boat.size.x;
                boat.size.x = boat.size.y;
                boat.size.y = tmp_size;
            }
        }
        // boat.pose.orientation.w = 1.0;
        // detected_boats_.push_back(boat);
        transform.setOrigin(tf::Vector3(boat.pose.position.x, boat.pose.position.y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, angle);
        transform.setRotation(q);
        broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", boat.id));
        boat.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        // boat.pose.orientation.w = q;
        detected_boats_.push_back(boat);
        count++;
    }
    boats_list_.boats = detected_boats_;
    boats_list_.header.frame_id = "map";
    boats_pub_.publish(boats_list_);
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    bool in_object = false;
    // double prev = 1000;
    struct obstacle temp_obstacle;
    obstacle_list.clear();
    try {
        listener_->waitForTransform("map", "lidar", ros::Time(0), ros::Duration(1.0));
        listener_->lookupTransform("map", "lidar", ros::Time(0), transform_d);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    double roll, pitch, yaw;
    tf::Matrix3x3(transform_d.getRotation()).getEulerYPR(yaw,pitch,roll);
    int num_scans = (scan->angle_min + scan->angle_max)/scan->angle_increment;
    sensor_msgs::LaserScan lidar_filtered = *scan;
    lidar_filtered.ranges.clear();
    if (scan->ranges[0] < scan->range_max) {
        in_object = true;
    }
    // https://github.com/wg-perception/people/blob/ccf714b37f3482a938df6fb424910279a45cb978/leg_detector/src/leg_detector.cpp#L683
    for (int i=1; i < scan->ranges.size(); ++i) {
        if (in_object) {
            lidar_filtered.ranges.push_back(scan->ranges[i]);
            if (scan->ranges[i] < scan->ranges[i-1]) {
                // lidar_filtered.ranges.push_back(scan->ranges[i]);
                if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                    temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.y = transform_d.getOrigin().y() - sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                } else {
                    temp_obstacle.closest_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.closest_point.y = transform_d.getOrigin().y() - sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
                }
            }
            if (fabs(scan->ranges[i+1] - scan->ranges[i]) > max_jump) {
                if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                    temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.y = transform_d.getOrigin().y() - sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                } else {
                    temp_obstacle.max_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                    temp_obstacle.max_point.y = transform_d.getOrigin().y() - sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
                }
                ROS_INFO("CLOSEST: scan_x = %f, scan_y = %f. \n", temp_obstacle.closest_point.x, temp_obstacle.closest_point.y);
                ROS_INFO("MAX: scan_x = %f, scan_y = %f. \n", temp_obstacle.max_point.x, temp_obstacle.max_point.y);
                if ((scan->range_min < temp_obstacle.min_point.x && scan->range_min < temp_obstacle.closest_point.x && scan->range_min < temp_obstacle.max_point.x) &&
                 (temp_obstacle.min_point.x < scan->range_max && temp_obstacle.closest_point.x < scan->range_max && temp_obstacle.max_point.x < scan->range_max))
                    obstacle_list.push_back(temp_obstacle);
                else
                    temp_obstacle = {};
                in_object = false;
            }
        } else if (fabs(scan->ranges[i-1] - scan->ranges[i]) > max_jump) {
            lidar_filtered.ranges.push_back(scan->ranges[i]);
            if (i<(num_scans/2)) { // Slightly nasty assumption that angle_min = angle_max
                temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.y = transform_d.getOrigin().y() - sin(yaw - (num_scans/2 - i)*scan->angle_increment) * scan->ranges[i];
            } else {
                temp_obstacle.min_point.x = transform_d.getOrigin().x() - cos(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];
                temp_obstacle.min_point.y = transform_d.getOrigin().y() - sin(yaw + (i - num_scans/2)*scan->angle_increment) * scan->ranges[i];                
            }
            ROS_INFO("MIN: scan_x = %f, scan_y = %f. \n", temp_obstacle.min_point.x, temp_obstacle.min_point.y);
            // ROS_INFO("lidar_x = %f, lidar_y = %f, scan_x = %f, scan_y = %f. \n", transform_d.getOrigin().x(), transform_d.getOrigin().y(), sin(yaw - (num_scans/2 - i)*scan->angle_increment), cos(yaw - (num_scans/2 - i)*scan->angle_increment));

            in_object = true;
        } else {
            lidar_filtered.ranges.push_back(75);
        }
        // prev = scan->ranges[i];
    }
    ROS_INFO("First obstacle min_x = %f, min_y = %f, number of obstacles: %lu. \n", obstacle_list[0].min_point.x, obstacle_list[0].min_point.y, obstacle_list.size());
    lidar_pub_.publish(lidar_filtered);
    filterBoats();
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "lidar_to_boats");
  ros::NodeHandle node;

  // instantiate publishers & subscribers
  boats_pub_ = node.advertise<social_navigation_layers::Boats>("/boats", 1);
  laser_sub_ = node.subscribe("/scan", 1, lidarCallback);
  lidar_pub_ = node.advertise<sensor_msgs::LaserScan>("/lidar_boat_point", 1);

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