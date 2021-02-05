#include "opt_path_in_gallerys.h"

GenerateGallery::GenerateGallery() : Node("opt_path_in_gallerys") {
    sub_path_ = this ->create_subscription<nav_msgs::msg::Path>("own_path", 2, std::bind(&GenerateGallery::subPath, this, _1));
    sub_map_ = this ->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 2, std::bind(&GenerateGallery::subMap, this, _1));
    pub_gallerys_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("agvs", 2);
    pub_gallerys2_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("gallerys", 2);
    pub_points_ = this->create_publisher<sensor_msgs::msg::PointCloud>("points", 2);
}

GenerateGallery::~GenerateGallery() {}

void GenerateGallery::worldToMap(double wx, double wy, int& mx, int& my) {
    mx = (wx - ori_map_.info.origin.position.x) / ori_map_.info.resolution;
    my = (wy - ori_map_.info.origin.position.y) / ori_map_.info.resolution;
}

void GenerateGallery::subPath(const nav_msgs::msg::Path::SharedPtr path) {
    ori_path_ = *path;

    auto t1 = chrono::steady_clock::now();
    generateGallery();
    cout << "cost time of opt path is: " << chrono::duration<double, micro>(chrono::steady_clock::now() - t1).count() << "us" << endl;
}

void GenerateGallery::subMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    ori_map_ = *map;
    map_resolution_ = ori_map_.info.resolution;
}

bool GenerateGallery::nodeObstacleCheck(double x, double y) {
    int mx;
    int my;
    worldToMap(x, y, mx, my);
    if(x <= ori_map_.info.origin.position.x || x >= ori_map_.info.origin.position.x + ori_map_.info.width * ori_map_.info.resolution ||
       y <= ori_map_.info.origin.position.y || y >= ori_map_.info.origin.position.y + ori_map_.info.height * ori_map_.info.resolution) {
        return true;
    }
    if(ori_map_.data[mx + my * ori_map_.info.width] < obstacle_threshold_) {
        return false;
    }
    else {
        return true;
    }
}

bool GenerateGallery::generateGallery() {
    if(ori_map_.data.size() < 7) {
        cout << "no map for opt." << endl;
    }
    if(ori_path_.poses.size() < 1) {
        cout << "no path for opt." << endl;
    }

    gallerys_.clear();
    vector<geometry_msgs::msg::Point> path;
    path.resize(ori_path_.poses.size());
    for(int i = 0; i < ori_path_.poses.size(); i++) {
        path[i].x = ori_path_.poses[i].pose.position.x;
        path[i].y = ori_path_.poses[i].pose.position.y;
        path[i].z = ori_path_.poses[i].pose.position.z;
    }
    Box box(path[0].x, path[0].x, path[0].y, path[0].y);
    bool ext_x_min = 1;
    bool ext_x_max = 1;
    bool ext_y_min = 1;
    bool ext_y_max = 1;
    int n = 0;
    while(path.size() != 0) {
        if(path[0].x >= box.x_min_ - 0.5* map_resolution_ && 
           path[0].x <= box.x_max_ + 0.5* map_resolution_ && 
           path[0].y >= box.y_min_ - 0.5* map_resolution_ && 
           path[0].y <= box.y_max_ + 0.5* map_resolution_) {
            path.erase(path.begin(), path.begin() + 1);
            continue;
        }
        if(ext_x_min == 0 && ext_x_max == 0 && ext_y_min == 0 && ext_y_max == 0) {
            gallerys_.push_back(box);
            box.x_min_ = path[0].x;
            box.x_max_ = path[0].x;
            box.y_min_ = path[0].y;
            box.y_max_ = path[0].y;
            ext_x_min = 1;
            ext_x_max = 1;
            ext_y_min = 1;
            ext_y_max = 1;
        }
        if(path[0].x < box.x_min_) {
            //cout << "left" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
        }
        else if(path[0].y < box.y_min_) {
            //cout << "down" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
        }
        else if(path[0].x > box.x_max_) {
            //cout << "right" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
            //cout << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
        }
        else if(path[0].y > box.y_max_) {
            //cout << "up" << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
            if(ext_y_max == 1) {
                if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                    box.y_max_ += map_resolution_;
                }
                else {
                    ext_y_max = 0;
                }
            }
            if(ext_x_min == 1) {
                if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                    box.x_min_ -= map_resolution_;
                }
                else {
                    ext_x_min = 0;
                }
            }
            if(ext_y_min == 1) {
                if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                    box.y_min_ -= map_resolution_;
                }
                else {
                    ext_y_min = 0;
                }
            }
            if(ext_x_max == 1) {
                if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                    box.x_max_ += map_resolution_;
                }
                else {
                    ext_x_max = 0;
                }
            }
        }
        
        // cout << path.size() << ", " << path[0].x << ", " << path[0].y << endl;
        // cout << box.x_min_ << ", " << box.x_max_ << ", " << box.y_min_ << ", " << box.y_max_ << endl;
        // cout << ext_x_min << ext_x_max << ext_y_min << ext_y_max << endl;
    }
    while(ext_x_min == 1 || ext_x_max == 1 || ext_y_min == 1 || ext_y_max == 1) {
        if(ext_x_min == 1) {
            if(extendBox(box.x_min_ - map_resolution_, box.y_min_, box.x_min_ - map_resolution_, box.y_max_) == true) {
                box.x_min_ -= map_resolution_;
            }
            else {
                ext_x_min = 0;
            }
        }
        if(ext_y_min == 1) {
            if(extendBox(box.x_min_, box.y_min_ - map_resolution_, box.x_max_, box.y_min_ - map_resolution_) == true) {
                box.y_min_ -= map_resolution_;
            }
            else {
                ext_y_min = 0;
            }
        }
        if(ext_x_max == 1) {
            if(extendBox(box.x_max_ + map_resolution_, box.y_min_, box.x_max_ + map_resolution_, box.y_max_) == true) {
                box.x_max_ += map_resolution_;
            }
            else {
                ext_x_max = 0;
            }
        }
        if(ext_y_max == 1) {
            if(extendBox(box.x_min_, box.y_max_ + map_resolution_, box.x_max_, box.y_max_ + map_resolution_) == true) {
                box.y_max_ += map_resolution_;
            }
            else {
                ext_y_max = 0;
            }
        }
    }
    gallerys_.push_back(box);
    displayGallery();
    for(int i = 0; i < gallerys_.size() - 1; i++) {
        if(twoGallerysIntersect(gallerys_[i], gallerys_[i + 1]) == 0) {
            cout << "generate error gallerys." << endl;

            return false;
        }
    }

    //auto t1 = chrono::steady_clock::now();
    generateQpProblam();
    //cout << "cost time of solve problam by OSQP is: " << chrono::duration<double, micro>(chrono::steady_clock::now() - t1).count() << "us" << endl;

    pubGallerys();

    return true;
}

bool GenerateGallery::extendBox(double x0, double y0, double x1, double y1) {
    bool ans = true;
    double d = hypot(x0 - x1, y0 - y1);
    if(d == 0) {
        if(nodeObstacleCheck(x0, y0) == 1) {
            return false;
        }
        else {
            return true;
        }
    }
    for(int i = 0; i <= d / map_resolution_; i++) {
        double x = x0 + double(i) * (x1 - x0) * map_resolution_ / d;
        double y = y0 + double(i) * (y1 - y0) * map_resolution_ / d;
        if(nodeObstacleCheck(x, y) == 1) {
            ans = false;
            break;
        }
    }

    return ans;
}

bool GenerateGallery::pathNodeInBox(double x, double y, Box box) {
    bool ans = false;
    if(x >= box.x_min_ && x <= box.x_max_ && y >= box.y_min_ && y <= box.y_max_) {}

    return ans;
}

bool GenerateGallery::pointInGallery(double x, double y, Box box, double bias) {
    bool ans = true;
    if(x >= box.x_min_ - bias && x <= box.x_max_ + bias && y >= box.y_min_ - bias && y <= box.y_max_ + bias) {
        return true;
    }
    else {
        return false;
    }

    return ans;
}

bool GenerateGallery::twoGallerysIntersect(Box box0, Box box1) {
    bool ans = true;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    x_min = box0.x_min_ > box1.x_min_ ? box0.x_min_ : box1.x_min_;
    x_max = box0.x_max_ < box1.x_max_ ? box0.x_max_ : box1.x_max_;
    y_min = box0.y_min_ > box1.y_min_ ? box0.y_min_ : box1.y_min_;
    y_max = box0.y_max_ < box1.y_max_ ? box0.y_max_ : box1.y_max_;
    if((x_max - x_min >= 0) && (y_max - y_min >= 0)) {
        return true;
    }
    else {
        return false;
    }

    return ans;
}

void GenerateGallery::generateQpProblam() {
    double x0 = ori_path_.poses[0].pose.position.x;
    double y0 = ori_path_.poses[0].pose.position.y;
    double x1 = (ori_path_.poses.end() - 1)->pose.position.x;
    double y1 = (ori_path_.poses.end() - 1)->pose.position.y;

    vector<Box> intersection_areas;    //两个走廊相交的区域
    for(int i = 0; i < int(gallerys_.size()) - 1; i++) {
        Box box;
        box.x_min_ = gallerys_[i].x_min_ > gallerys_[i + 1].x_min_ ? gallerys_[i].x_min_ : gallerys_[i + 1].x_min_;
        box.x_max_ = gallerys_[i].x_max_ < gallerys_[i + 1].x_max_ ? gallerys_[i].x_max_ : gallerys_[i + 1].x_max_;
        box.y_min_ = gallerys_[i].y_min_ > gallerys_[i + 1].y_min_ ? gallerys_[i].y_min_ : gallerys_[i + 1].y_min_;
        box.y_max_ = gallerys_[i].y_max_ < gallerys_[i + 1].y_max_ ? gallerys_[i].y_max_ : gallerys_[i + 1].y_max_;
        intersection_areas.push_back(box);

        //cout << "test: " << box.x_min_ << ", " << box.x_max_ << ", " << box.y_min_ << ", " << box.y_max_ << endl;
    }

    Matrix<double, Dynamic, Dynamic> H;
    Matrix<double, Dynamic, Dynamic> g;
    Matrix<double, Dynamic, Dynamic> A;
    Matrix<double, Dynamic, Dynamic> lb;
    Matrix<double, Dynamic, Dynamic> ub;
    vector<Matrix<double, Dynamic, Dynamic> > contraint_A;
    vector<double> contraint_lb;
    vector<double> contraint_ub;
    Matrix<double, 10, 10> M1;
    Matrix<double, 10, 10> M2;
    M1 << 0., 0., 0., 0.,  0., 0., 0., 0., 0.,  0.,
          0., 0., 0., 0.,  0., 0., 0., 0., 0.,  0.,
          0., 0., 2., 0.,  0., 0., 0., 0., 0.,  0.,
          0., 0., 0., 6.,  0., 0., 0., 0., 0.,  0.,
          0., 0., 0., 0., 12., 0., 0., 0., 0.,  0.,
          0., 0., 0., 0.,  0., 0., 0., 0., 0.,  0.,
          0., 0., 0., 0.,  0., 0., 0., 0., 0.,  0.,
          0., 0., 0., 0.,  0., 0., 0., 2., 0.,  0.,
          0., 0., 0., 0.,  0., 0., 0., 0., 6.,  0.,
          0., 0., 0., 0.,  0., 0., 0., 0., 0., 12.;
    H.setZero(10 * gallerys_.size(), 10 * gallerys_.size());
    g.setZero(10 * gallerys_.size(), 1);
    vector<double> t;    //各段曲线的参数t
    for(uint i = 0; i < gallerys_.size(); i++) {
        double t0;    //距离除以速度等于时间，默认速度1m/s
        if(gallerys_.size() > 1) {
            if(i == 0) {
                t0 = hypot(0.5 * intersection_areas[i].y_min_ + 0.5 * intersection_areas[i].y_max_ - y0, 0.5 * intersection_areas[i].x_min_ + 0.5 * intersection_areas[i].x_max_ - x0);
            }
            else if(i == gallerys_.size() - 1) {
                t0 = hypot(0.5 * intersection_areas[i - 1].y_min_ + 0.5 * intersection_areas[i - 1].y_max_ - y1, 0.5 * intersection_areas[i - 1].x_min_ + 0.5 * intersection_areas[i - 1].x_max_ - x1);
            }
            else {
                t0 = hypot(0.5 * (intersection_areas[i].y_min_ + intersection_areas[i].y_max_ - intersection_areas[i - 1].y_min_ - intersection_areas[i - 1].y_max_), 
                           0.5 * (intersection_areas[i].x_min_ + intersection_areas[i].x_max_ - intersection_areas[i - 1].x_min_ - intersection_areas[i - 1].x_max_));
            }
        }
        else {
            t0 = hypot(y1 - y0, x1 - x0);
        }
        t.push_back(t0);
    }
    double max_t = *max_element(t.begin(), t.end());
    for(uint i = 0; i < gallerys_.size(); i++) {    //放缩所有参数t，使最大值为2
        t[i] *= 2 / max_t;
    }
    for(uint i = 0; i < gallerys_.size(); i++) {
        M2 << 0., 0.,             0.,             0.,              0., 0., 0.,             0.,             0.,              0.,
              0., 0.,             0.,             0.,              0., 0., 0.,             0.,             0.,              0.,
              0., 0.,           t[i], pow(t[i], 2)/2,  pow(t[i], 3)/3, 0., 0.,             0.,             0.,              0.,
              0., 0., pow(t[i], 2)/2, pow(t[i], 3)/3,  pow(t[i], 4)/4, 0., 0.,             0.,             0.,              0.,
              0., 0., pow(t[i], 3)/3, pow(t[i], 4)/4,  pow(t[i], 5)/5, 0., 0.,             0.,             0.,              0.,
              0., 0.,             0.,             0.,              0., 0., 0.,             0.,             0.,              0.,
              0., 0.,             0.,             0.,              0., 0., 0.,             0.,             0.,              0.,
              0., 0.,             0.,             0.,              0., 0., 0.,           t[i], pow(t[i], 2)/2,  pow(t[i], 3)/3,
              0., 0.,             0.,             0.,              0., 0., 0., pow(t[i], 2)/2, pow(t[i], 3)/3,  pow(t[i], 4)/4,
              0., 0.,             0.,             0.,              0., 0., 0., pow(t[i], 3)/3, pow(t[i], 4)/4,  pow(t[i], 5)/5;
        H.block(10 * i, 10 * i, 10, 10) = M1 * M2 * M1;

        Matrix<double, Dynamic, Dynamic> contraint_A0;
        if(i == 0) {
            contraint_A0.setZero(1, 10 * gallerys_.size());    //位置起点约束
            contraint_A0.block(0, 10 * i, 1, 10) << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(x0);
            contraint_ub.push_back(x0);
            contraint_A0.setZero(1, 10 * gallerys_.size());    //位置起点约束
            contraint_A0.block(0, 10 * i, 1, 10) << 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(y0);
            contraint_ub.push_back(y0);
        }
        else {
            contraint_A0.setZero(1, 10 * gallerys_.size());    //起点位置约束
            contraint_A0.block(0, 10 * i - 10, 1, 20) << 1., t[i-1], t[i-1]*t[i-1], t[i-1]*t[i-1]*t[i-1], t[i-1]*t[i-1]*t[i-1]*t[i-1], 0., 0., 0., 0., 0., -1., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(0);
            contraint_ub.push_back(0);
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i - 10, 1, 20) << 0., 0., 0., 0., 0., 1., t[i-1], t[i-1]*t[i-1], t[i-1]*t[i-1]*t[i-1], t[i-1]*t[i-1]*t[i-1]*t[i-1], 0., 0., 0., 0., 0., -1., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(0);
            contraint_ub.push_back(0);

            contraint_A0.setZero(1, 10 * gallerys_.size());    //起点速度约束
            contraint_A0.block(0, 10 * i - 10, 1, 20) << 0., 1., 2*t[i-1], 3*t[i-1]*t[i-1], 4*t[i-1]*t[i-1]*t[i-1], 0., 0., 0., 0., 0., 0., -1., 0., 0., 0., 0., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(0);
            contraint_ub.push_back(0);
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i - 10, 1, 20) << 0., 0., 0., 0., 0., 0., 1., 2*t[i-1], 3*t[i-1]*t[i-1], 4*t[i-1]*t[i-1]*t[i-1], 0., 0., 0., 0., 0., 0., -1., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(0);
            contraint_ub.push_back(0);
        }
        for(double t0 = 0.; t0 < t[i]; t0 += 0.2) {    //中间点位置约束
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i, 1, 10) << 1., t0, t0*t0, t0*t0*t0, t0*t0*t0*t0, 0., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(gallerys_[i].x_min_);
            contraint_ub.push_back(gallerys_[i].x_max_);
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i, 1, 10) << 0., 0., 0., 0., 0., 1., t0, t0*t0, t0*t0*t0, t0*t0*t0*t0;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(gallerys_[i].y_min_);
            contraint_ub.push_back(gallerys_[i].y_max_);
        }
        if(i == gallerys_.size() - 1) {    //终点位置约束
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i, 1, 10) << 1., t[i], t[i]*t[i], t[i]*t[i]*t[i], t[i]*t[i]*t[i]*t[i], 0., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(x1);
            contraint_ub.push_back(x1);
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i, 1, 10) << 0., 0., 0., 0., 0., 1., t[i], t[i]*t[i], t[i]*t[i]*t[i], t[i]*t[i]*t[i]*t[i];
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(y1);
            contraint_ub.push_back(y1);
        }
        else {    //终点位置约束
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i, 1, 10) << 1., t[i], t[i]*t[i], t[i]*t[i]*t[i], t[i]*t[i]*t[i]*t[i], 0., 0., 0., 0., 0.;
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(intersection_areas[i].x_min_);
            contraint_ub.push_back(intersection_areas[i].x_max_);
            contraint_A0.setZero(1, 10 * gallerys_.size());
            contraint_A0.block(0, 10 * i, 1, 10) << 0., 0., 0., 0., 0., 1., t[i], t[i]*t[i], t[i]*t[i]*t[i], t[i]*t[i]*t[i]*t[i];
            contraint_A.push_back(contraint_A0);
            contraint_lb.push_back(intersection_areas[i].y_min_);
            contraint_ub.push_back(intersection_areas[i].y_max_);
        }
    }
    A.resize(contraint_A.size(), 10 * gallerys_.size());
    lb.resize(contraint_A.size(), 1);
    ub.resize(contraint_A.size(), 1);
    for(uint i = 0; i < contraint_A.size(); i++) {
        A.block(i, 0, 1, 10 * gallerys_.size()) = contraint_A[i];
        lb(i, 0) = contraint_lb[i];
        ub(i, 0) = contraint_ub[i];
    }

    Matrix<double, Dynamic, 1> solve;
    bool solve_state = osqp_solver.solveQpProblam(H, g, A, lb, ub, solve);
    //cout << solve << endl;

    if(solve_state) {
        sensor_msgs::msg::PointCloud points;
        points.header.frame_id = "map";
        for(uint i = 0; i < gallerys_.size(); i++) {
            for(double t0 = 0; t0 < t[i]; t0 += 0.05) {
                geometry_msgs::msg::Point32 point;
                point.x = solve(10 * i + 0, 0) + solve(10 * i + 1, 0) * t0 + solve(10 * i + 2, 0) * t0*t0 + solve(10 * i + 3, 0) * t0*t0*t0 + solve(10 * i + 4, 0) * t0*t0*t0*t0;
                point.y = solve(10 * i + 5, 0) + solve(10 * i + 6, 0) * t0 + solve(10 * i + 7, 0) * t0*t0 + solve(10 * i + 8, 0) * t0*t0*t0 + solve(10 * i + 9, 0) * t0*t0*t0*t0;
                point.z = 0.05;
                points.points.push_back(point);
            }

            // cout << "position: " << solve(10 * i + 0, 0) << ", " << solve(10 * i + 5, 0) << ", "
            //                     << solve(10 * i + 0, 0) + solve(10 * i + 1, 0) * t[i] + solve(10 * i + 2, 0) * t[i]*t[i] + solve(10 * i + 3, 0) * t[i]*t[i]*t[i] + solve(10 * i + 4, 0) * t[i]*t[i]*t[i]*t[i] << ", "
            //                     << solve(10 * i + 5, 0) + solve(10 * i + 6, 0) * t[i] + solve(10 * i + 7, 0) * t[i]*t[i] + solve(10 * i + 8, 0) * t[i]*t[i]*t[i] + solve(10 * i + 9, 0) * t[i]*t[i]*t[i]*t[i] << ", " 
            //     << "velocity: " << solve(10 * i + 1, 0) << ", " << solve(10 * i + 6, 0) << ", "
            //                     << solve(10 * i + 1, 0) + 2 * solve(10 * i + 2, 0) * t[i] + 3 * solve(10 * i + 3, 0) * t[i]*t[i] + 4 * solve(10 * i + 4, 0) * t[i]*t[i]*t[i] << ", "
            //                     << solve(10 * i + 6, 0) + 2 * solve(10 * i + 7, 0) * t[i] + 3 * solve(10 * i + 8, 0) * t[i]*t[i] + 4 * solve(10 * i + 9, 0) * t[i]*t[i]*t[i] << endl;
        }
        pub_points_->publish(points);
    }
}

void GenerateGallery::pubGallerys() {
    std_msgs::msg::Float32MultiArray gallerys;
    gallerys.layout.dim.resize(gallerys_.size());
    for(int i = 0; i < gallerys_.size(); i++) {
        gallerys.data.push_back(gallerys_[i].x_min_);
        gallerys.data.push_back(gallerys_[i].x_max_);
        gallerys.data.push_back(gallerys_[i].y_min_);
        gallerys.data.push_back(gallerys_[i].y_max_);
    }
    pub_gallerys2_->publish(gallerys);
}

void GenerateGallery::displayGallery() {
    static int last_gallerys_num = 0;
    cout << "display the gallerys. " << gallerys_.size() << endl;
    visualization_msgs::msg::MarkerArray markers;
    for(int i = 0; i < gallerys_.size(); i++) {
        visualization_msgs::msg::Marker module;
        module.header.frame_id = "map";
        module.ns = "";
        module.color.r = 0.5f;
        module.color.g = 0.8f;
        module.color.b = 0.2f;
        module.color.a = 1.0;
        module.frame_locked = true;
        module.type = visualization_msgs::msg::Marker::CUBE;
        module.action = visualization_msgs::msg::Marker::ADD;
        module.id = i;
        module.pose.position.x = 0.5 * (gallerys_[i].x_min_ + gallerys_[i].x_max_);
        module.pose.position.y = 0.5 * (gallerys_[i].y_min_ + gallerys_[i].y_max_);
        module.pose.position.z = -0.1 -0.05 * i;
        module.scale.x = gallerys_[i].x_max_ - gallerys_[i].x_min_;
        module.scale.y = gallerys_[i].y_max_ - gallerys_[i].y_min_;
        module.scale.z = 0.05;
        markers.markers.push_back(module);

        cout << gallerys_[i].x_min_ << ", " << gallerys_[i].x_max_ << ", "
             << gallerys_[i].y_min_ << ", " << gallerys_[i].y_max_ << endl;
    }
    for(int i = 0; i < last_gallerys_num - int(gallerys_.size()); i++) {
        visualization_msgs::msg::Marker module;
        module.header.frame_id = "map";
        module.ns = "";
        module.color.r = 0.5f;
        module.color.g = 0.8f;
        module.color.b = 0.2f;
        module.color.a = 0;
        module.frame_locked = true;
        module.type = visualization_msgs::msg::Marker::CUBE;
        module.action = visualization_msgs::msg::Marker::ADD;
        module.id = i + gallerys_.size();
        module.pose.position.x = 0;
        module.pose.position.y = 0;
        module.pose.position.z = 0;
        module.scale.x = 0.1;
        module.scale.y = 0.1;
        module.scale.z = 0.1;
        markers.markers.push_back(module);
    }

    pub_gallerys_->publish(markers);
    last_gallerys_num = gallerys_.size();
}

int main(int argc, char** argv) {
    cout << "begin opt path in gallerys." << endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenerateGallery>());
    rclcpp::shutdown();
    return 0;
}