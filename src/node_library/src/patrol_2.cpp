#include "node_library/patrol_2.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "nav_msgs/msg/odometry.hpp"

namespace BehaviorTree{

    Patrol2Node::Patrol2Node(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    //limit_distance[9]={}
                    std::stringstream ss;
                    ss << "patrol_2_node";
                    node = std::make_shared<rclcpp::Node>(ss.str().c_str());
                    subscription_id = node->create_subscription<std_msgs::msg::Int32>("self_id",10,std::bind(&Patrol2Node::message_callback_my_id,this,std::placeholders::_1));
                    subscription_robot_position = node->create_subscription<robot_msgs::msg::AutoaimInfo>("/autoaim2decision",10,std::bind(&Patrol2Node::message_callback_robot_position,this,std::placeholders::_1));
                    subscription_odometry = node->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&Patrol2Node::message_callback_self_position,this,std::placeholders::_1));
                    RCLCPP_INFO(rclcpp::get_logger("patrol_2_node"),"Patro2Node initialized");
                    state = 0;          

                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");

                    occupation_centre = toml::find<std::vector<std::pair<double,double> > >(toml_file,"occupation_centre");
                    occupation_radium = toml::find<std::vector<double> >(toml_file,"occupation_radium");

                    begin_time = rclcpp::Clock().now();

                    self_id = -1;
                }

    void Patrol2Node::message_callback_self_position(const nav_msgs::msg::Odometry &msg){
        self_position[0] = msg.pose.pose.position.x;
        self_position[1] = msg.pose.pose.position.y;
        self_position[2] = msg.pose.pose.position.z;
        return;
    }
    
    void Patrol2Node::message_callback_my_id(const std_msgs::msg::Int32 &msg){
        self_id = msg.data;
        return;
    }

    void Patrol2Node::message_callback_robot_position(const robot_msgs::msg::AutoaimInfo &msg){
        if(self_id < 0) return;
        for(int i=0,lim = msg.data.size();i<lim;++i){
            if(self_id % 100 == (msg.data[i].id - 1) % 9 +1)
            ally_position_map[msg.data[i].id] = Eigen::Vector3d(msg.data[i].pos.x,msg.data[i].pos.y,(rclcpp::Clock().now() - begin_time).seconds());
            else
            enemy_position_map[msg.data[i].id] = Eigen::Vector3d(msg.data[i].pos.x,msg.data[i].pos.y,(rclcpp::Clock().now() - begin_time).seconds());
        }
        return;
    }

    void Patrol2Node::refresh_map(){
        double time_record = (rclcpp::Clock().now() - begin_time).seconds();
        for(auto it = ally_position_map.begin();it != ally_position_map.end();){
            if((time_record - it->second[2]) > 2){
                it = ally_position_map.erase(it);
            }
            else{
                ++it;
            }
        }

        for(auto it = enemy_position_map.begin();it != enemy_position_map.end();){
            if((time_record - it->second[2]) > 2){
                it = enemy_position_map.erase(it);
            }
            else{
                ++it;
            }
        }
        return;
    }

    BT::NodeStatus Patrol2Node::tick(){

        rclcpp::spin_some(node);
        refresh_map();
        
        double distance_limit_max,distance_limit_min;
        if(!getInput<double>("distance_limit_max",distance_limit_max)
        || !getInput<double>("distance_limit_min",distance_limit_min))
        return BT::NodeStatus::SUCCESS;
        getInput<geometry_msgs::msg::Point>("navigation_point",navigation_point);
        Eigen::Vector3d self_navigation;
        self_navigation = Eigen::Vector3d(navigation_point.x,navigation_point.y,navigation_point.z);
        distance = (self_position - self_navigation).norm();

        geometry_msgs::msg::Point self_point;
        self_point.x = self_position[0];
        self_point.y = self_position[1];
        self_point.z = self_position[2];

        #define pdd std::pair<double,double>
        #define pDD std::pair<pdd,pdd>

        for(int i=0,lim = occupation_centre.size();i<lim;++i){
            if(decision_utils::judging_point::get_distance(occupation_centre[i],pdd(navigation_point.x,navigation_point.y)) < occupation_radium[i]
            && decision_utils::judging_point::get_distance(occupation_centre[i],pdd(self_position[0],   self_position[1]   )) < occupation_radium[i]){
                for(auto element:ally_position_map){
                    if(decision_utils::judging_point::get_distance(occupation_centre[i],pdd(element.second[0],element.second[1])) < occupation_radium[i])
                    setOutput<geometry_msgs::msg::Point>("self_point",self_point);
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }

        #undef pDD
        #undef pdd

        if(state == 0){
            setOutput<geometry_msgs::msg::Point>("self_point",self_point);
            if(distance>=distance_limit_max)
            state = 1;
        }
        if(state == 1){
            if(distance>=distance_limit_min){
                setOutput<geometry_msgs::msg::Point>("now_navigation_point",navigation_point);
            }
            else state = 0;
        }
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Patrol2Node>("Patrol2Node");
// }