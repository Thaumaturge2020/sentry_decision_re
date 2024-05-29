#include "behavior_tree_2/patrol_area_node.hpp"

namespace BehaviorTree{
    ResurrctionTypeNode::ResurrctionTypeNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    const auto toml_file = toml::parse(ROOT "config/config.toml");
                    patrol_type = toml::find<int>(toml_file,"resurrection_type");
                }
    BT::NodeStatus ResurrctionTypeNode::tick()
    {
        setOutput<int>("val_port",patrol_type);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::ResurrctionTypeNode>("NavUNode");
// }