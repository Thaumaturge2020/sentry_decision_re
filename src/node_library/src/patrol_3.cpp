#include "node_library/patrol_3.hpp"

namespace BehaviorTree{

    Patrol3Node::Patrol3Node(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name,config){
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"Patrol3Node initialized");

            ti_now = rclcpp::Clock().now();
            num=0;
            clock=0;
            situation = 0;
            //初始化patrol_1_position，weight_enemy

                    std::stringstream ss;
                    ss << "patrol_3";

            node_patrol1 = std::make_shared<rclcpp::Node>(ss.str().c_str());

            RCLCPP_INFO(rclcpp::get_logger("node_patrol"),"%s",node_patrol1->get_name());

            patrol_radar_info = node_patrol1->create_subscription<robot_msgs::msg::AutoaimInfo>("radar_info",10,std::bind(&Patrol3Node::message_callback_patrol_radar_info,this,std::placeholders::_1));
            const auto map_file = toml::parse(ROOT "config/battle_information.toml");
            start_pos = toml::find<std::pair<double,double> >(map_file,"self_position");
            map_position = toml::find<std::vector<std::vector<std::pair<double,double> > > >(map_file,"navigation_position");
            time_limit = toml::find<double>(map_file,"time_limit");
            subcribe_my_pos = node_patrol1->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&Patrol3Node::message_callback_my_pos,this,std::placeholders::_1));

            for(auto vec:map_position){
                map_position_eigen.push_back(std::vector<Eigen::Vector3d>());
                auto it = map_position_eigen.end();
                --it;
                for(auto data:vec){
                    it->push_back(Eigen::Vector3d(data.first,data.second,0));
            // RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"Patrol3Node initialized position:%lf %lf",data.first,data.second);
                }
            }

            // RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"Patrol3Node initialized");
        }

    void Patrol3Node::message_callback_patrol_radar_info(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }

    void Patrol3Node::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        my_pos = msg.pose.pose.position;
        my_pos.x += start_pos.first;
        my_pos.y += start_pos.second;
        return;
    }

    BT::NodeStatus Patrol3Node::tick(){
        rclcpp::spin_some(node_patrol1);
        // RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%lf",(rclcpp::Clock().now() - ti_now).seconds());

        getInput<double>("time_limit",time_limit);


        int area_choose;
        if(!getInput<int>("area_choose",area_choose)){
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"FAILED");
            return BT::NodeStatus::FAILURE;
        }
        else if(!getInput<double>("distance_limit",distance_limit)){
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"FAILED");
            return BT::NodeStatus::FAILURE;
        }
        else if((rclcpp::Clock().now() - ti_now).seconds() < time_limit) return BT::NodeStatus::SUCCESS;
        else if((map_position_eigen[area_choose][situation]-Eigen::Vector3d(my_pos.x,my_pos.y,my_pos.z)).norm() < distance_limit){
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"I'm ticked <");
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"navigation_area : %d",area_choose);
            new_situation = situation + 1;
            new_situation = (new_situation == map_position_eigen[area_choose].size() ? 0 : new_situation);
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%lf %lf %lf %d %d %d",my_pos.x,my_pos.y,my_pos.z,new_situation ,situation, map_position_eigen[area_choose].size());
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"I'm ticked >");
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"navigation_area : %d",area_choose);
            if(area_choose >= map_position_eigen.size()) return BT::NodeStatus::FAILURE;
            new_situation = situation + 1;
            new_situation = (new_situation == map_position_eigen[area_choose].size() ? 0 : new_situation);

            
            RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%lf %lf %lf %d %d",my_pos.x,my_pos.y,my_pos.z,new_situation , map_position_eigen[area_choose].size());

            // if((map_position_eigen[area_choose][situation]-Eigen::Vector3d(my_pos.x,my_pos.y,my_pos.z)).norm() > distance_limit){
                //new_situation = situation;
            // }
        }
        
        ti_now = rclcpp::Clock().now();

        geometry_msgs::msg::Point navigation_point;
        area_choose=getInput<int>("area_choose").value();
        navigation_point.x = map_position_eigen[area_choose][new_situation].x();
        navigation_point.y = map_position_eigen[area_choose][new_situation].y();
        navigation_point.z = map_position_eigen[area_choose][new_situation].z();
        setOutput<geometry_msgs::msg::Point>("navigation_point",navigation_point);
        RCLCPP_INFO(rclcpp::get_logger("patrol_3_node"),"%lf %lf %lf",navigation_point.x,navigation_point.y,navigation_point.z);
        situation = new_situation;
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Patrol3Node>("Patrol3Node");
// }