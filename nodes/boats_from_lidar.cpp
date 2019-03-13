#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <social_navigation_layers/Boats.h>
#include <social_navigation_layers/Boat.h>
// #include <social_navigation_layers/boats_from_lidar.h>

struct obstacle {
    int min_point;
    int closest_point;
    int max_point;
};

double max_jump = 2.32;
std::vector<obstacle> obstacle_list;
social_navigation_layers::Boats boats_list_;
std::list<social_navigation_layers::Boat> transformed_boats_; 
void updateVelocities() {
    for ...

}

void filterBoats() {
    updateVelocities();
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    bool in_object = false;
    double prev = scan->ranges[0];
    struct obstacle temp_obstacle;
  // https://github.com/wg-perception/people/blob/ccf714b37f3482a938df6fb424910279a45cb978/leg_detector/src/leg_detector.cpp#L683
    for (int i=1; i < scan->ranges.size(); ++i) {
        if (scan->ranges[i] - prev > max_jump) {
            temp_obstacle.min_point = i;
            in_object = true;
        }
        if (in_object) {
            if (scan->ranges[i] > prev)
                temp_obstacle.closest_point = i;
            if (scan->ranges[i+1] - scan->ranges[i] > max_jump) {
                temp_obstacle.max_point = i;
                in_object = false;
                obstacle_list.push_back(temp_obstacle);
            }
        }
    prev = scan->ranges[i];
    }
    ROS_INFO("Number of obstacles: %lu. \n", obstacle_list.size());
    filterBoats();
}

// void publishBoats() {
//     pub_.publish(boats_list_);
// }


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_to_boats");
    int rate = 1;
    ros::NodeHandle nh_;

    ros::Subscriber sub_ = nh_.subscribe("/scan", 1, lidarCallback);
    ros::Publisher pub_ = nh_.advertise<social_navigation_layers::Boats>("/boats", 1);
    ros::Rate r(rate);
    while (ros::ok()) {
        ros::spin();
        // filterBoats();
        // updateVelocities();
        pub_.publish(boats_list_);
        r.sleep();
    }
    return 0;
}