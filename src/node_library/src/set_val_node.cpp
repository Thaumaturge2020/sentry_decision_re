#include "node_library/set_val_node.hpp"

namespace BehaviorTree{
    SetValNode::SetValNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                }
    BT::NodeStatus SetValNode::tick()
    {
        int my_val = 0;
        getInput<int>("val_set",my_val);
        setOutput<int>("val_port",my_val);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::SetValNode>("NavUNode");
// }