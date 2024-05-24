#include "node_library/set_val_once_node.hpp"

namespace BehaviorTree{
    SetValOnceNode::SetValOnceNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    base_flag = 1;
                }
    BT::NodeStatus SetValOnceNode::tick()
    {
        int my_val = 0;
        if(base_flag == 1){
            getInput<int>("val_set",my_val);
            setOutput<int>("val_port",my_val);
            base_flag = 0;
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::SetValOnceNode>("NavUNode");
// }