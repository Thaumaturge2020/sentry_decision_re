#include "node_library/stay_ori_node.hpp"

namespace BehaviorTree{
    StayOriNode::StayOriNode(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name,config){
            start_time = rclcpp::Clock().now();
            setOutput<double>("time_during",0.0);

            std::stringstream ss;
            ss << "stay_ori_node";
            node_stay_ori = rclcpp::Node::make_shared(ss.str().c_str());
            robot_odometry_subscription = node_stay_ori->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&StayOriNode::message_callback_odometry,this,std::placeholders::_1));
            const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
            init_pos = toml::find<std::pair<double,double> >(toml_file,"self_position");
            pose_stamp.x = init_pos.first;
            pose_stamp.y = init_pos.second;
            pose_stamp.z = 0;
        }

void StayOriNode::message_callback_odometry(const nav_msgs::msg::Odometry &msg){
    pose_stamp = msg.pose.pose.position;
    pose_stamp.x += init_pos.first;
    pose_stamp.y += init_pos.second;
    return;
}

BT::NodeStatus BehaviorTree::StayOriNode::tick(){
    rclcpp::spin_some(node_stay_ori);
    setOutput<geometry_msgs::msg::Point>("now_navigation_point",pose_stamp);
    return BT::NodeStatus::SUCCESS;
}


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::StayOriNode>("StayOriNode");
// }