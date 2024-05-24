#include "node_library/time_compu_limit.hpp"

namespace BehaviorTree{

    TimeCompuLimit::TimeCompuLimit(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    rclcpp::Time ti_now = rclcpp::Clock().now();

                    std::stringstream ss;
                    ss << "time_compu_limit";
                    node = rclcpp::Node::make_shared(ss.str().c_str());
                    sub_game_time = node->create_subscription<std_msgs::msg::Int32>("/game_time",10,std::bind(&TimeCompuLimit::message_callback_game_time,this,std::placeholders::_1));
                    time_switch = 2;
                    if_start_count = false;
                    FLAG = 1;
                }

    void TimeCompuLimit::message_callback_game_time(const std_msgs::msg::Int32 &msg){
      if(!msg.data){
        return;
      }
      if(!if_start_count){
       if_start_count = true;
       time_begin = rclcpp::Clock().now();
      }
      return;
    }

    BT::NodeStatus TimeCompuLimit::tick(){
        rclcpp::spin_some(node);
        if(!if_start_count) return BT::NodeStatus::FAILURE;
        double ti_limit1 = 0,ti_limit2 = 0,ti_interval = 0;
        if(!getInput<double>("first_interval",ti_interval)) return BT::NodeStatus::FAILURE;
        if(!getInput<double>("time_limit1",ti_limit1)) return BT::NodeStatus::FAILURE;
        if(!getInput<double>("time_limit2",ti_limit2)) return BT::NodeStatus::FAILURE;
        double now_ti = (rclcpp::Clock().now() - time_begin).seconds();
        // RCLCPP_INFO(rclcpp::get_logger("time_compu_logger"),"%lf %d",now_ti,time_switch);
        if(ti_interval <= 0 && FLAG == 1){
          FLAG = 0;
            time_begin = rclcpp::Clock().now();
          getInput<int>("time_switch",time_switch);
          goto S;
        }
        if(time_switch == 2){
          if(now_ti > ti_interval){
            if(!getInput<int>("time_switch",time_switch)) return BT::NodeStatus::FAILURE;
            time_begin = rclcpp::Clock().now();
            return BT::NodeStatus::SUCCESS;
          }
          else return BT::NodeStatus::FAILURE;
        }

        S:;

        // RCLCPP_INFO(rclcpp::get_logger("time_compu_node"),"I'm ticked %d %d",time_switch,now_ti);

        if(time_switch == 0){
          if(now_ti > ti_limit1)
            time_switch^=1,time_begin = rclcpp::Clock().now();
        }

        else{
          if(now_ti > ti_limit2)
            time_switch^=1,time_begin = rclcpp::Clock().now();
        }
        
        if(time_switch <= 0){
          return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<Time_compu::time_compu>("time_compu_node");
// }