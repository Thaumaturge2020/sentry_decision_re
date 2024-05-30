#include "node_library/operator_node.hpp"

namespace BehaviorTree{

    OperatorNode::OperatorNode(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    std::stringstream ss;
                    ss << "operator_node";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());        
                    target_enemy = -1;
                    aerial_type = -1;
                    std::array<std::array<double,3>,3 > trans_matrix = toml::find<std::array<std::array<double,3>,3 > > (toml_file,"trans_matrix");
                    trans = Eigen::Matrix3d();
                    for(int i=0;i<3;++i) for(int j=0;j<3;++j) trans(i,j) = trans_matrix[i][j];
                    // trans = Eigen::Matrix3d(trans_matrix);
                    subscription_operator_cmd = node1->create_subscription<robot_msgs::msg::AerialCommands>("/easy_robot_commands/map_command",10,[this](const robot_msgs::msg::AerialCommands &msg){
                        switch(msg.cmd_keyboard){
                            //控符
                            case 'W': aerial_type = 0;break;
                            //强制重启
                            case 'E': aerial_type = 1;break;
                            //强制定点移动
                            case 'S': aerial_type = 2;break;
                            //强制回巡逻区巡逻
                            case 'D': aerial_type = 3;break;
                            //自主买弹
                            case 'C': aerial_type = 5;break;
                            //哨兵进攻基地
                            case 'G': aerial_type = 6;break;
                            //哨兵原地自转一秒
                            case 'Z': aerial_type = 7;break;
                            //哨兵上梯高
                            case 'V': aerial_type = 8;break;
                            //取消选择
                            case 'Q': aerial_type = -1;break;
                        }
                        target_enemy = -1;
                        if(msg.target_robot_id){
                            aerial_type = 4;
                            target_enemy = msg.target_robot_id;
                        }
                        if(aerial_type == 2){
                            Eigen::Vector3d sub_vector;
                            sub_vector = Eigen::Vector3d(msg.target_position_x,msg.target_position_y,1);
                            // RCLCPP_INFO(rclcpp::get_logger("vector_node"),"%lf %lf %lf",sub_vector[0],sub_vector[1],sub_vector[2]);
                            // for(int i=0;i<3;++i)
                            // RCLCPP_INFO(rclcpp::get_logger("vector_node"),"%lf %lf %lf",trans(i,0),trans(i,1),trans(i,2));

                            sub_vector = trans * sub_vector;
                            navigate_point.x = sub_vector[0];
                            navigate_point.y = sub_vector[1];
                            navigate_point.z = 0;
                            RCLCPP_INFO(rclcpp::get_logger("operator_node"),"%lf %lf",navigate_point.x,navigate_point.y);
                        }
                    });
                    flag = 0;
                }

    BT::NodeStatus OperatorNode::tick(){
        // RCLCPP_INFO(rclcpp::get_logger("operator_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        setOutput<int>("expected_operator_mode",aerial_type);
        if(aerial_type == -1)
        return BT::NodeStatus::FAILURE;
        if(aerial_type == 2){
            geometry_msgs::msg::Point msg;
            msg.x = navigate_point.x;
            msg.y = navigate_point.y;
            msg.z = 0;
            setOutput<geometry_msgs::msg::Point>("expected_place",msg);
        }
        if(aerial_type == 4){
            setOutput<int>("expected_enemy",decision_utils::id_mapping::referee2autoaim(target_enemy));
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::OperatorNode>("OperatorNode");
// }
