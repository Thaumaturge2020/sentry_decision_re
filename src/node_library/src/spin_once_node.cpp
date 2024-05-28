#include "node_library/spin_once_node.hpp"

namespace BehaviorTree{
    SpinOnceNode::SpinOnceNode(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name,config){
            std::stringstream ss;
            ss << "spin_once_node";
            node = std::make_shared<rclcpp::Node>(ss.str().c_str());
            subscription_1 = node->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,[this](const nav_msgs::msg::Odometry &msg){
                self_position = msg.pose.pose.position;
            });
            publisher_1 = node->create_publisher<robot_msgs::msg::WalkCmd>("/decision2transplan",10);
        }
    BT::NodeStatus BehaviorTree::SpinOnceNode::tick(){
        rclcpp::spin_some(node);
        robot_msgs::msg::WalkCmd msg;
        int aerial_type;
        getInput<int>("aerial_type",aerial_type);
        if(aerial_type == 7){
            spin_ti = rclcpp::Clock().now();
        }
        setOutput<int>("reset_aerial_type",-1);
        if((rclcpp::Clock().now()-spin_ti).seconds() < 1){
            return BT::NodeStatus::SUCCESS;
        }
        msg.pos = self_position;
        msg.radium = 180;
        msg.velocity = 0;
        publisher_1->publish(msg);
        return BT::NodeStatus::FAILURE;
    }


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::SpinOnceNode>("SpinOnceNode");
// }