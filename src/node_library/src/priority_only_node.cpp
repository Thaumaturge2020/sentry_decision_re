#include "node_library/priority_only_node.hpp"

namespace BehaviorTree{
    PriorityOnlyNode::PriorityOnlyNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    std::stringstream ss;
                    ss << "PriorityOnlyNode";
                    node = std::make_shared<rclcpp::Node>(ss.str().c_str());
                    publisher_1 = node->create_publisher<robot_msgs::msg::CamCommand>("/decision2transmit",10);
                }
    BT::NodeStatus PriorityOnlyNode::tick()
    {
        robot_msgs::msg::CamCommand msg;
        std::string priority_type,
                    priority_level;
        int autoaim_mode;
        getInput<std::string>("priority_type",priority_type);
        getInput<std::string>("priority_level",priority_level);
        getInput<int>("autoaim_mode",autoaim_mode);
        std::vector<int> priority_type_arr = decision_utils::string_parser::GetIntArray(priority_type);
        std::vector<int> priority_level_arr = decision_utils::string_parser::GetIntArray(priority_level);
        std::vector<unsigned char> priority_type_uarr,priority_level_uarr;
        for(auto i:priority_type_arr) priority_type_uarr.push_back((unsigned char)i);
        for(auto i:priority_level_arr) priority_level_uarr.push_back((unsigned char)i);
        msg.priority_type_arr = priority_type_uarr;
        msg.priority_level_arr = priority_level_uarr;
        msg.autoaim_mode = autoaim_mode;
        msg.yaw = 0;
        publisher_1->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::PriorityOnlyNode>("NavUNode");
// }