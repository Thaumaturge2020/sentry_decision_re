#ifndef RM_SENTRY_2024_PARAM_NODE_1_
#define RM_SENTRY_2024_PARAM_NODE_1_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point.h"
#include "robot_msgs/msg/autoaim_info.hpp"
#include "robot_msgs/msg/robot_info.hpp"
#include <toml.hpp>

namespace BehaviorTree{
    class ParamNode1:public BT::SyncActionNode{
        private:
        public:
            ParamNode1(const std::string&name, const BT::NodeConfig& config);
            int decision_type,
                my_outpost_blood_lowerbound,
                my_outpost_blood_upperbound,
                enemy_outpost_blood_lowerbound,
                enemy_outpost_blood_upperbound,
                area_choose;

            double  chase_distance_able,
                    chase_distance_limit;
            
            std::string priority_input,
                        priority_type;

            static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("decision_type"),
                    BT::OutputPort<int>("my_outpost_blood_lowerbound"),
                    BT::OutputPort<int>("my_outpost_blood_upperbound"),
                    BT::OutputPort<int>("enemy_outpost_blood_lowerbound"),
                    BT::OutputPort<int>("enemy_outpost_blood_upperbound"),
                    BT::OutputPort<int>("area_choose"),
                    BT::OutputPort<double>("chase_distance_able"),
                    BT::OutputPort<double>("chase_distance_limit"),
                    BT::OutputPort<std::string>("priority_input"),
                    BT::OutputPort<std::string>("gimbal_choose_enemy_priority_input")
                };
            }
            BT::NodeStatus tick() override;
            
    };
}

#endif