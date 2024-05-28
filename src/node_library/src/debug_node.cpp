#include "node_library/debug_node.hpp"

namespace BehaviorTree{

    DebugNode::DebugNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){;
                    id = 0;
                    ti = rclcpp::Clock().now();
                }

    BT::NodeStatus DebugNode::tick()
    {
        getInput<int>("debug_output_id",id);
        
        if((rclcpp::Clock().now() - ti).seconds() > 1){
            ti = rclcpp::Clock().now();
            RCLCPP_INFO(rclcpp::get_logger("debug_output"),"debug output %d",id);
        }

        int type = 0;
        getInput<int>("true_or_false",type);
        if(type == 0) 
        return BT::NodeStatus::SUCCESS;
        else
        return BT::NodeStatus::FAILURE;
    }


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::DebugNode>("BuildAttackNode");
// }