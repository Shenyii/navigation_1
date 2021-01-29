#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;
using namespace std;

class GenerateMap  : public rclcpp::Node{
public:
    GenerateMap() : Node("generate_map") {
        initMap();
        pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 2);
        timer1_ = this->create_wall_timer(2000ms, std::bind(&GenerateMap::pubMap, this));
    }

private:
    nav_msgs::msg::OccupancyGrid map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::TimerBase::SharedPtr timer1_;

    void initMap() {
        map_.header.frame_id = "map";
        map_.info.resolution = 0.05;
        map_.info.width = 400;
        map_.info.height = 400;
        map_.info.origin.position.x = -10;
        map_.info.origin.position.y = -10;
        map_.info.origin.position.z = 0;
        map_.info.origin.orientation.x = 0;
        map_.info.origin.orientation.y = 0;
        map_.info.origin.orientation.z = 0;
        map_.info.origin.orientation.w = 1;
        map_.data.resize(map_.info.width * map_.info.height);
        for(int i = 0; i < int(map_.data.size()); i++) {
            map_.data[i] = -1;
        }

        uint mx0;
        uint my0;
        mx0 = (-5 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.1 / map_.info.resolution; mx <= mx0 + 0.1 / map_.info.resolution; mx++) {
            for(uint my = my0 - 5.0 / map_.info.resolution; my <= my0 + 5.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (5 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 5.0 / map_.info.resolution; mx <= mx0 + 5.0 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.1 / map_.info.resolution; my <= my0 + 0.1 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (5 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.1 / map_.info.resolution; mx <= mx0 + 0.1 / map_.info.resolution; mx++) {
            for(uint my = my0 - 5.0 / map_.info.resolution; my <= my0 + 5.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (2.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-5.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 5.0 / map_.info.resolution; mx <= mx0 + 5.0 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.1 / map_.info.resolution; my <= my0 + 0.1 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (-3 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-1 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.1 / map_.info.resolution; mx <= mx0 + 0.1 / map_.info.resolution; mx++) {
            for(uint my = my0 - 4.0 / map_.info.resolution; my <= my0 + 4.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.5 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (3.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 3.5 / map_.info.resolution; mx <= mx0 + 3.5 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.1 / map_.info.resolution; my <= my0 + 0.1 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (3.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (0.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.1 / map_.info.resolution; mx <= mx0 + 0.1 / map_.info.resolution; mx++) {
            for(uint my = my0 - 3.0 / map_.info.resolution; my <= my0 + 3.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.5 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-3.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 2.5 / map_.info.resolution; mx <= mx0 + 2.5 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.15 / map_.info.resolution; my <= my0 + 0.15 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (-2.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-0.25 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 2.75 / map_.info.resolution; my <= my0 + 2.75 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (2.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 2.0 / map_.info.resolution; mx <= mx0 + 2.0 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.15 / map_.info.resolution; my <= my0 + 0.15 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (2.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-0.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 2.0 / map_.info.resolution; my <= my0 + 2.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.5 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-2.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 1.5 / map_.info.resolution; mx <= mx0 + 1.5 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.15 / map_.info.resolution; my <= my0 + 0.15 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (-1 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-0.5 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 1.5 / map_.info.resolution; my <= my0 + 1.5 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (1.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 1.0 / map_.info.resolution; mx <= mx0 + 1.0 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.15 / map_.info.resolution; my <= my0 + 0.15 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (1.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (0.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 1.0 / map_.info.resolution; my <= my0 + 1.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.5 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-1.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.5 / map_.info.resolution; mx <= mx0 + 0.5 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.15 / map_.info.resolution; my <= my0 + 0.15 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-0.5 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.5 / map_.info.resolution; my <= my0 + 0.5 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (0.25 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (0.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.25 / map_.info.resolution; mx <= mx0 + 0.25 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.15 / map_.info.resolution; my <= my0 + 0.15 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (-2.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-5.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 1.0 / map_.info.resolution; my <= my0 + 1.0 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (-1.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-4.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 0.15 / map_.info.resolution; mx <= mx0 + 0.15 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.5 / map_.info.resolution; my <= my0 + 0.5 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
        mx0 = (1.0 - map_.info.origin.position.x) / map_.info.resolution;
        my0 = (-4.0 - map_.info.origin.position.y) / map_.info.resolution;
        for(uint mx = mx0 - 1.0 / map_.info.resolution; mx <= mx0 + 1.0 / map_.info.resolution; mx++) {
            for(uint my = my0 - 0.5 / map_.info.resolution; my <= my0 + 0.5 / map_.info.resolution; my++) {
                if(my * map_.info.width + mx < map_.data.size()) {
                    map_.data[my * map_.info.width + mx] = 100;
                }
            }
        }
    }

    void pubMap() {
        cout << "pub the map." << endl;
        pub_map_->publish(map_);
    }
};

int main(int argc, char* argv[]) {
    cout << "hello world!" << endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenerateMap>());
    rclcpp::shutdown();
    return 0;
}
