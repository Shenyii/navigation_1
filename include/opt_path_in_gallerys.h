#ifndef OPT_PATH_IN_GALLERYS_H
#define OPT_PATH_IN_GALLERYS_H

#include <iostream>
#include <vector>
#include <list>
#include <cmath>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/path.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "osqp_solver.h"

using namespace std;
using namespace Eigen;
using std::placeholders::_1;

class Box {
public:
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;

    Box() {
        x_min_ = 0;
        x_max_ = 0;
        y_min_ = 0;
        y_max_ = 0;
    };
    Box(double x0, double x1, double y0, double y1) {
        x_min_ = x0;
        x_max_ = x1;
        y_min_ = y0;
        y_max_ = y1;
    };
};

class GenerateGallery : public rclcpp::Node {
public:
    GenerateGallery();
    ~GenerateGallery();

private:
    double obstacle_threshold_;
    double map_resolution_;
    vector<Box> gallerys_;
    Osqp_Solver osqp_solver;

    void worldToMap(double wx, double wy, int& mx, int& my);
    bool nodeObstacleCheck(double x, double y);
    bool generateGallery();
    bool extendBox(double x0, double y0, double x1, double y1);
    bool pathNodeInBox(double x, double y, Box box);
    bool pointInGallery(double x, double y, Box box, double bias);
    bool twoGallerysIntersect(Box box0, Box box1);
    void generateQpProblam();

    nav_msgs::msg::Path ori_path_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    nav_msgs::msg::OccupancyGrid ori_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_gallerys_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_gallerys2_;

    void subPath(const nav_msgs::msg::Path::SharedPtr path);
    void subMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void pubGallerys();
    void displayGallery();
};

#endif