#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <ctime>

struct Waypoint {
    double x;
    double y;
    double z;
    std::string name;
};

double current_x = 0.0;
double current_y = 0.0;
double current_z = 0.0;
bool odom_received = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    current_z = msg->pose.pose.position.z;
    odom_received = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh("~");

    std::string goal_topic;
    nh.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
    ros::Subscriber odom_sub = nh.subscribe("/drone_0_visual_slam/odom", 1, odomCallback);

    ros::Rate rate(10);

    std::vector<Waypoint> points = {
        {0.0, 0.0, 1.0, "A"},
        {4.0, 1.0, 1.0, "B"},
        {8.0, 0.0, 1.0, "C"},
        {4.0, -2.0, 1.0, "D"},
        {0.0, 0.0, 1.0, "A"}
    };

    ROS_INFO("Mission node started. Waiting for odometry...");
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Odometry received. Starting mission execution.");

    std::string csv_filename = "/tmp/flight_mission_log.csv";
    std::ofstream csv_file(csv_filename);
    csv_file << "time_from_start_sec,x,y,z,target_id,distance_to_target\n";

    ros::Time start_time = ros::Time::now();
    ros::Time segment_start_time = start_time;

    int current_wp_index = 0;
    bool mission_completed = false;
    int hold_counter = 0;
    const int required_hold_checks = 10; 

    while (ros::ok() && !mission_completed) {
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();
        double t_from_start = (current_time - start_time).toSec();

        Waypoint target = points[current_wp_index];

        double dx = target.x - current_x;
        double dy = target.y - current_y;
        double dz = target.z - current_z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        csv_file << t_from_start << "," 
                 << current_x << "," 
                 << current_y << "," 
                 << current_z << "," 
                 << current_wp_index << "," 
                 << distance << "\n";

        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = current_time;
        goal_msg.header.frame_id = "world";
        goal_msg.pose.position.x = target.x;
        goal_msg.pose.position.y = target.y;
        goal_msg.pose.position.z = target.z;
        goal_msg.pose.orientation.w = 1.0;
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.0;

        goal_pub.publish(goal_msg);

        if (distance < 0.5) {
            hold_counter++;
            if (hold_counter >= required_hold_checks) {
                ros::Time segment_end_time = ros::Time::now();
                double seg_duration = (segment_end_time - segment_start_time).toSec();
                
                ROS_INFO("Reached waypoint %s (Index: %d) in %.2f seconds.", 
                         target.name.c_str(), current_wp_index, seg_duration);

                current_wp_index++;
                hold_counter = 0;
                segment_start_time = ros::Time::now();

                if (current_wp_index >= points.size()) {
                    mission_completed = true;
                    ROS_INFO("Mission successfully completed! All waypoints visited.");
                }
            }
        } else {
            if (hold_counter > 0) {
                hold_counter = 0;
            }
        }

        rate.sleep();
    }

    csv_file.close();
    ROS_INFO("Flight log saved to %s", csv_filename.c_str());

    return 0;
}
