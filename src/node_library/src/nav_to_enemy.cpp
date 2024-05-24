#include "node_library/nav_to_enemy.hpp"

namespace BehaviorTree{

    NavToEnemy::NavToEnemy(const std::string&name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    rclcpp::Time ti_now = rclcpp::Clock().now();
                    std::stringstream ss;
                    ss << "nav_to_enemy";
                    node1 = rclcpp::Node::make_shared(ss.str().c_str());   
                    subscription_enemy_pos = node1->create_subscription<robot_msgs::msg::AutoaimInfo>("autoaim2decision",10,std::bind(&NavToEnemy::message_callback_enemy_pos,this,std::placeholders::_1));
                    my_pos = node1->create_subscription<nav_msgs::msg::Odometry>("/Odometry_Vehicle",10,std::bind(&NavToEnemy::message_callback_my_pos,this,std::placeholders::_1));
                    decision_pos = node1->create_publisher<robot_msgs::msg::WalkCmd>("decision2pathplan", 10);
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    self_start_point = toml::find<std::pair<double,double> >(toml_file,"self_position");
                    time_interval = 1;
                    if_aviod = false;
                }

    void NavToEnemy::message_callback_enemy_pos(const robot_msgs::msg::AutoaimInfo &msg){
        robot_pos_array = msg.data;
        return;
    }
    
    void NavToEnemy::message_callback_my_pos(const nav_msgs::msg::Odometry &msg){
        //Header header
        //string child_frame_id
       // geometry_msgs/TwistWithCovariance twist
        My_pos = msg.pose.pose.position;
        My_pos.x += self_start_point.first;
        My_pos.y += self_start_point.second;
        return;
    }

//     void NavToEnemy::avoid_fight(geometry_msgs::msg::Point Point,geometry_msgs::msg::Point Point_start,geometry_msgs::msg::Point Point_my,geometry_msgs::msg::Point &Point_pub,double d){
//        //障碍信息,高度
//        static geometry_msgs::msg::Point pos_decision;
//        double x1,x2,y1,y2,a;
//        a = atan((Point.x-Point_start.x)/(Point_start.y-Point.y));
//        x1 = Point_start.x+d*cos(a);
//        x2 = Point_start.x-d*cos(a);
//        y1 = Point_start.y+d*sin(a);
//        y2 = Point_start.y-d*sin(a);
//       if(sqrt((Point_my.x-x1)*(Point_my.x-x1)+(Point_my.y-y1)*(Point_my.y-y1))<1){
//         pos_decision.x = Point_start.x;
//         pos_decision.y = Point_start.y;
//       }
//       else if(sqrt((Point_my.x-Point_start.x)*(Point_my.x-Point_start.x)+(Point_my.y-Point_start.y)*(Point_my.y-Point_start.y))<1){
//         pos_decision.x = x1;
//         pos_decision.y = y1;
//       }
//       Point_pub = pos_decision;
       
// }

    BT::NodeStatus NavToEnemy::tick(){
        static geometry_msgs::msg::Point start_avoid_point;
        // RCLCPP_INFO(rclcpp::get_logger("move_to_enemy_node"),"I'm ticked");
        rclcpp::spin_some(node1);
        size_t array_size = robot_pos_array.size();
        int now_enemy_id;
        static int flag = 0;
        static rclcpp::Time now_ti;

        static Eigen::Vector2d prod = Eigen::Vector2d(1.0,0.0);

        // RCLCPP_INFO(rclcpp::get_logger("I am nav_to_enemy_node"),"I am nav_to_enemy");
        // RCLCPP_INFO(rclcpp::get_logger("logger???"),"%lf %lf",(rclcpp::Clock().now()-now_ti).seconds(),time_interval);
        
        if(flag == 0){
            now_ti = rclcpp::Clock().now();
            flag = 1;
        }
        else{
            if((rclcpp::Clock().now()-now_ti).seconds() > time_interval){
                now_ti = rclcpp::Clock().now();
                Eigen::Vector2d prod_vec = Eigen::MatrixXd::Random(2,1);
                prod = Eigen::Vector2d(prod[0]*prod_vec[0] - prod[1]*prod_vec[1],prod[0]*prod_vec[1] + prod[1]*prod_vec[0]);
                prod = prod/prod.norm();
            }
        }

        if(!getInput<int>("now_enemy_id",now_enemy_id)){
            RCLCPP_INFO(rclcpp::get_logger("move_to_enemy_node"),"No Response");
            return BT::NodeStatus::FAILURE;
        }

        for(int i=0;i<array_size;++i){
            if(robot_pos_array[i].id == now_enemy_id){
                enemy_pos_deal.x=robot_pos_array[i].pos.x;
                enemy_pos_deal.y=robot_pos_array[i].pos.y;
                enemy_pos_deal.z=robot_pos_array[i].pos.z;
                goto S;
            }
        }
        return BT::NodeStatus::FAILURE;

        S:;

        Eigen::Vector2d vec1 = Eigen::Vector2d(enemy_pos_deal.x,enemy_pos_deal.y),
                        vec2 = Eigen::Vector2d(My_pos.x,My_pos.y),
                        bias = (vec1-vec2);

        // RCLCPP_INFO(rclcpp::get_logger("logger"),"%lf %lf",prod[0],prod[1]);

        double distance = bias.norm();
        bias = Eigen::Vector2d(bias[0] * prod[0] - bias[1] * prod[1], bias[0] * prod[1] + bias[1] * prod[0]);
        if(distance <= 2){
            bias = bias / distance * 2;
        }
        enemy_pos_pub.x = My_pos.x + bias[0];
        enemy_pos_pub.y = My_pos.y + bias[1];
        enemy_pos_pub.z = My_pos.z;

        robot_msgs::msg::WalkCmd my_cmd;
        my_cmd.pos = enemy_pos_pub;
        my_cmd.opt = 2;
        my_cmd.radium = 360.0;
        my_cmd.velocity = 2500.0;
        my_cmd.cap_mode = 1;
        
        decision_pos->publish(my_cmd);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::NavToEnemy>("NavToEnemy");
// }