#include "node_library/resurrection_type_node.hpp"

namespace BehaviorTree{
    ResurrectionTypeNode::ResurrectionTypeNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    const auto toml_file = toml::parse(ROOT "config/config.toml");
                    patrol_type = toml::find<int>(toml_file,"resurrection_type");
                }
    BT::NodeStatus ResurrectionTypeNode::tick()
    {
        setOutput<int>("val_port",patrol_type);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorExTree::ResurrctionTypeNode>("NavUNode");
// }