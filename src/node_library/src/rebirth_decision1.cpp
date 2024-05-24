#include "node_library/rebirth_decision1.hpp"

namespace BehaviorTree{

    RebirthDecision1::RebirthDecision1(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    newblood=400;
                    blood=400;
                    setOutput<int>("if_rebirth",0);
                    node = rclcpp::Node::make_shared("subscriber_if_rebirth_blood");  
                    subscription_postion = node->create_subscription<nav_msgs::msg::Odometry>("Odometry_Vehicle",10,std::bind(&RebirthDecision1::message_callback_position,this,std::placeholders::_1));
                    subscription_blood = node->create_subscription<std_msgs::msg::Int32>("my_sentry_blood",10,std::bind(&RebirthDecision1::message_callback_blood,this,std::placeholders::_1));
                    publisher_point = node->create_publisher<geometry_msgs::msg::Point>("decision2pathplan",10);
                    subscription_friend_position = node->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&RebirthDecision1::message_callback_friend_position,this,std::placeholders::_1));
                    const auto map_file = toml::parse(ROOT "config/battle_information.toml");
                    position = toml::find<std::vector<std::vector<std::pair<double,double> > > >(map_file,"navigation_POS");
                    std::pair<double,double> b=position[4][0];
                    a.x=b.first;
                    a.y=b.second;
                    ////////
                }
     void RebirthDecision1::message_callback_blood(const std_msgs::msg::Int32 &msg){
        newblood=msg.data;
     }

    void RebirthDecision1::message_callback_position(const nav_msgs::msg::Odometry &msg){
        my_position=msg;
    }

    void RebirthDecision1::message_callback_friend_position(const robot_msgs::msg::AutoaimInfo &msg){
        for(auto i:msg.data){
            if(i.id/9==0){
                std::pair<double,double>a;
                a.first=i.pos.x;
                a.second=i.pos.y;
                if(i.id==2){
                    en_position=a;
                    continue;
                }
                friend_position.push_back(a);
            }
        }
    }
                
    BT::NodeStatus RebirthDecision1::tick()
    {
        std::pair<double,double> b;
        if(!getInput<int>("rebirth_decision_1",num))return BT::NodeStatus::FAILURE;
        if(num!=1)return BT::NodeStatus::FAILURE;
        if(blood==0&&newblood>50){
            blood=newblood;
            setOutput<int>("if_rebirth",3);
        }
       if(newblood==400)setOutput<int>("if_rebirth",3);
       getInput<int>("rebirth_decision_1",if_re);

        static rclcpp::Time ti;
        if(if_re==3){
            bool jud=false;
            for(auto i:friend_position)
                if(distance(i.first,a.x,i.second,a.y)<1.0)jud=true;
            if(distance(en_position.first,c.x,en_position.second,c.y)<1.0)jud=true;
            if(!jud)if_re=1;
            else if_re=4;
        }
       if(if_re==1){
          
           
            setOutput<geometry_msgs::msg::Point>("pos",a);
            ti = rclcpp::Clock().now();
            geometry_msgs::msg::Point pub_pos;
            pub_pos.x = a.x;
            pub_pos.y = a.y;
            pub_pos.z = 0;
            if(distance(my_position.pose.pose.position.x,a.x,my_position.pose.pose.position.y,a.y)<0.10){
                while((rclcpp::Clock().now() - ti).seconds() < 2){
                publisher_point->publish(pub_pos);
                }
                if_re=2;
            }
       }
       if(if_re==2){
            setOutput<geometry_msgs::msg::Point>("pos",a);
       }
        if(if_re+=4){
            setOutput<geometry_msgs::msg::Point>("pos",a);
            geometry_msgs::msg::Point pub_pos;
            pub_pos.x = a.x;
            pub_pos.y = a.y;
            pub_pos.z = 0;
            publisher_point->publish(pub_pos);
        }

       return BT::NodeStatus::SUCCESS;
    }


}