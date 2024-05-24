#include "node_library/check_val_node.hpp"

namespace BehaviorTree{
    CheckValNode::CheckValNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    val_desire = 0;
                }
    BT::NodeStatus CheckValNode::tick()
    {
        int my_val = 0;
        getInput<int>("val_desire",val_desire);
        getInput<int>("val_port",my_val);
        // RCLCPP_INFO(rclcpp::get_logger("CheckValNode"),"%d %d",val_desire,my_val);
        if(my_val == val_desire)
        return BT::NodeStatus::SUCCESS;
        else
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::CheckValNode>("NavUNode");
// }