#include "node_library/compute_area_choose.hpp"

namespace BehaviorTree{

    ComputeAreaChoose::ComputeAreaChoose(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){;}
                
    BT::NodeStatus ComputeAreaChoose::tick()
    {
        int now_area_choose;
        if(!getInput<int>("default_area_choose",now_area_choose))
        {
            RCLCPP_INFO(rclcpp::get_logger("ComputeAreaChoose"),"No Response");
            return BT::NodeStatus::SUCCESS;
        }
        setOutput<int>("area_choose",now_area_choose);
        return BT::NodeStatus::SUCCESS;
    }


}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::ComputeAreaChoose>("BuildAttackNode");
// }