#include "node_library/nav_to_specific_place.hpp"

namespace BehaviorTree{

    NavToSpecificPlace::NavToSpecificPlace(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "nav_to_specific_place";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());                    
                    publisher_velocity_cmd = node1->create_publisher<robot_msgs::msg::WalkCmd>("decision2pathplan",10);
                    publisher_test_cmd = node1->create_publisher<std_msgs::msg::Int16>("test_cmd",10);
                    flag = 0;
                }

    BT::NodeStatus NavToSpecificPlace::tick(){
        rclcpp::spin_some(node1);
        geometry_msgs::msg::Point target_place;
        if(getInput<geometry_msgs::msg::Point>("target_place",target_place)){
            robot_msgs::msg::WalkCmd my_cmd;
            my_cmd.pos = target_place;
            my_cmd.opt = 2;
            my_cmd.radium = 360.0;
            my_cmd.velocity = 2500.0;
            //RCLCPP_INFO(rclcpp::get_logger("nav_to_specific_place"),"GET TICKED");
            publisher_velocity_cmd->publish(my_cmd);
            std_msgs::msg::Int16 test_cmd;
            test_cmd.data = 1;
            publisher_test_cmd->publish(test_cmd);
            return BT::NodeStatus::SUCCESS;
        }
        flag = 0;
        return BT::NodeStatus::FAILURE;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::NavToSpecificPlace>("NavToSpecificPlace");
// }