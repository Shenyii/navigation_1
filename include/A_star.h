#ifndef A_STAR_H
#define A_STAR_H

#include <iostream>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/path.hpp>

//using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

class Node2 {
public:
    double x_;
    double y_;
    double theta_;
    double distance_;
    double heuristics_value_;
    Node2* father_node_;

    Node2(double x, double y, double distance, double value, Node2* father_node) {
        x_ = x;
        y_ = y;
        theta_ = 0;
        distance_ = distance;
        heuristics_value_ = value;
        father_node_ = father_node;
    };
};

bool nodeLess(Node2* node1, Node2* node2) {
    return node1->heuristics_value_ < node2->heuristics_value_;
}

class AStar : public rclcpp::Node {
public:
    AStar();
    ~AStar();

private:
    nav_msgs::msg::Path path_;
    double start_x_;
    double start_y_;
    double goal_x_;
    double goal_y_;
    double extend_dist_;
    vector<Node2*> close_list_;
    vector<Node2*> open_list_;
    int find_path_flag_;
    double obstacle_threshold_;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool searchThePath();
    bool nodeObstacleCheck(double x, double y);
    void extendCloseList();
    int beInList(Node2* node, vector<Node2*> list);
    int beInList(double x, double y, vector<Node2*> list);
    int findPathCheck();
    void getThePath();
    void add2Openlist(Node2* node);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_ori_map_;
    nav_msgs::msg::OccupancyGrid ori_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_start_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose2_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_points;

    void subOriMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void subStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr start_pose);
    void subGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);
    void displayCloseList();
};

#endif