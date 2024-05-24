#ifndef RM_SENTRY_2024_GUARD_OUTPOST_
#define RM_SENTRY_2024_GUARD_OUTPOST_

#include "behaviortree_cpp/bt_factory.h"

namespace BehaviorTree{
    class GuardOutpost:public BT::SyncActionNode{
        private:
        public:
            GuardOutpost(const std::string&name, const BT::NodeConfig& config);
            static BT::PortsList providedPorts(){
                return {
                  //  BT::OutputPort<int>("area_choose"),
                    BT::OutputPort<int>("id")

                };
            }
            BT::NodeStatus tick() override;
    };
}

#endif