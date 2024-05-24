#ifndef RM_SENTRY_2024_PATROL_AREA_DECISION_
#define RM_SENTRY_2024_PATROL_AREA_DECISION_

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include "robot_msgs/msg/build_state.hpp"
namespace BehaviorTree{
    class patrol_area_decision:public BT::SyncActionNode{
        public:
        patrol_area_decision(const std::string&name, const BT::NodeConfig& config);
        rclcpp::Subscription<std_msgs::msg::Int32>:: SharedPtr decsionx;
        rclcpp::Node::SharedPtr patrol_decision;

        void message_callback_decsionx(const std_msgs::msg::Int32 &msg);

        static BT::PortsList providedPorts(){
                return {
                    BT::InputPort<int>("decisionx"),
                    BT::OutputPort<int>("decisionx")
                };
            }
        int decision_num;
        rclcpp::Time start_time,now_time;

        void decision1();
        void decision2();
        void decision3();
        void decision4();
        void decision5();
        void decision6();

        BT::NodeStatus tick() override;
    };
}

#endif