#ifndef RM_SENTRY_2024_PATROL_4_
#define RM_SENTRY_2024_PATROL_4_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"
#include "toml.hpp"
#include "decision_utils/judging_point.hpp"
#include <random>   

namespace BehaviorTree{
    class Patrol4Node:public BT::SyncActionNode{
        private:
        public:
        Patrol4Node(const std::string&name, const BT::NodeConfig& config);
        geometry_msgs::msg::Point now_navigation_point, navigation_point;
        double radium;
        double weight_id_enemy[9];
        double limit_distance[9];
        unsigned int UNSIGNED_MAX;
        std::random_device my_device; // 创建一个std::random_device对象
        unsigned int seed;
        std::mt19937 engine;
        double random_gen_limit;
        rclcpp::Time ti;
        double mini_X,mini_Y,maxi_X,maxi_Y;

        std::vector<std::pair<double,double> > restrict_area;

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<int>("area_id"),
                BT::InputPort<int>("random_gen_limit"),
                BT::BidirectionalPort<geometry_msgs::msg::Point>("now_navigation_point")//导航点
            };
        }
        double get_random01();
        BT::NodeStatus tick() override;
            
    };
}

#endif