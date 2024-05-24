#ifndef RM_SENTRY_2024_PATROL_3_
#define RM_SENTRY_2024_PATROL_3_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <toml.hpp>

namespace BehaviorTree{
    class Patrol3Node:public BT::SyncActionNode{
        private:
        public:
            rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr patrol_radar_info;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subcribe_my_pos;
            rclcpp::Node::SharedPtr node_patrol1;
            Patrol3Node(const std::string&name, const BT::NodeConfig& config);
            int num,clock,situation,new_situation;
            int num_limit[5],clock_limit[5];
            std::vector< std::vector < std::pair < double,double > > > map_position;
            std::vector< std::vector < Eigen::Vector3d > > map_position_eigen ;
            double distance_limit;
            double weight_enemy[9];  //0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8 0.8
            double time_limit;
            std::vector<robot_msgs::msg::RobotInfo> robot_pos_array;
            std::pair<double,double> start_pos;
            rclcpp::Time ti_now;
            geometry_msgs::msg::Point my_pos;
            void message_callback_patrol_radar_info(const robot_msgs::msg::AutoaimInfo &msg);
            void message_callback_my_pos(const nav_msgs::msg::Odometry &msg);
            static BT::PortsList providedPorts(){
                return {
                    BT::BidirectionalPort<int>("area_choose"),
                    BT::InputPort<double>("time_begin"),
                    BT::InputPort<double>("distance_limit"),
                    BT::OutputPort<geometry_msgs::msg::Point>("navigation_point"),  ////目标点的坐标
                    BT::InputPort<double>("time_limit")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif