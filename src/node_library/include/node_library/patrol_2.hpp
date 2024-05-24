#ifndef RM_SENTRY_2024_PATROL_2_
#define RM_SENTRY_2024_PATROL_2_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "decision_utils/judging_point.hpp"
#include "std_msgs/msg/int32.hpp"
#include "toml.hpp"

#include "Eigen/Core"
#include "Eigen/Dense"

//geometry_msgs::Point：前两维用来存坐标，后一维用来存接受时间

namespace BehaviorTree{
    class Patrol2Node:public BT::SyncActionNode{
        private:
        public:
        Patrol2Node(const std::string&name, const BT::NodeConfig& config);
        int now_enemy_id;
        geometry_msgs::msg::Point now_navigation_point, navigation_point;
        rclcpp::Node::SharedPtr node;
        double distance;
        double distance_self_now_navigation_point,limit;
        double weight_id_enemy[9];
        bool state;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_id;
        rclcpp::Subscription<robot_msgs::msg::AutoaimInfo>::SharedPtr subscription_robot_position;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry;

        std::map<int,Eigen::Vector3d> ally_position_map;
        std::map<int,Eigen::Vector3d> enemy_position_map;

        Eigen::Vector3d self_position;
        
        int self_id;

        rclcpp::Time begin_time;

        //占用区：一个机器人在占用区域中呆很长时间
        
        std::vector<std::pair<double,double> >  occupation_centre;
        std::vector<double>                     occupation_radium;

        void message_callback_my_id(const std_msgs::msg::Int32 &msg);
        void message_callback_robot_position(const robot_msgs::msg::AutoaimInfo &msg);
        void message_callback_self_position(const nav_msgs::msg::Odometry &msg);
        void refresh_map();

        double limit_distance[9];
        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<double>("distance_limit_min"),
                BT::InputPort<double>("distance_limit_max"),
                BT::BidirectionalPort<geometry_msgs::msg::Point>("now_navigation_point"),  //导航点
                BT::InputPort<geometry_msgs::msg::Point>("navigation_point"),//巡逻路径给定点
            };
        }
        BT::NodeStatus tick() override;
            
    };
}

#endif