#include "node_library/guard_outpost.hpp"

namespace BehaviorTree{

    GuardOutpost::GuardOutpost(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    int post_area=0,post_target_id=0; //此处postarea为我方前哨站巡逻区域的信号 
                    setOutput<int>("id",post_target_id);
                   // setOutput<int>("area_choose",post_area);
                }
    
    BT::NodeStatus GuardOutpost::tick(){return BT::NodeStatus::SUCCESS;}


}
