#include "node_library/patrol_area_decision.hpp"

namespace BehaviorTree{
    patrol_area_decision::patrol_area_decision(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                
                    std::stringstream ss;
                    ss << "patrol_area_decision";
                    patrol_decision = rclcpp::Node::make_shared(ss.str().c_str());

                    decsionx = patrol_decision->create_subscription<std_msgs::msg::Int32>("decision_number", 10, std::bind(&patrol_area_decision::message_callback_decsionx, this, std::placeholders::_1));
                    start_time=rclcpp::Clock().now();
                }
    void patrol_area_decision::message_callback_decsionx(const std_msgs::msg::Int32 &msg){
        int a=msg.data;
        setOutput<int>("decisionx",a);
    }

    void patrol_area_decision::decision1(){

    }

    void patrol_area_decision::decision2(){

    }

    void patrol_area_decision::decision3(){

    }

    void patrol_area_decision::decision4(){

    }

    void patrol_area_decision::decision5(){

    }

    void patrol_area_decision::decision6(){

    }

    BT::NodeStatus patrol_area_decision::tick(){
        if(!getInput<int>("decisionx",decision_num))return BT::NodeStatus::FAILURE;
        switch (decision_num)
        {
        case 1:
            decision1();
            break;
        case 2:
            decision2();
            break;
        case 3:
            decision3();
            break;
        case 4:
            decision4();
            break;
        case 5:
            decision5();
            break;
        case 6:
            decision6();
            break;
        
        default:
            break;
        }

        return BT::NodeStatus::SUCCESS;
    }
}