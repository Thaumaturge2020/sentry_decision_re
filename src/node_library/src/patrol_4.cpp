#include "node_library/patrol_4.hpp"


namespace BehaviorTree{

    //use for random patrol

    Patrol4Node::Patrol4Node(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    //limit_distance[9]={

                    std::random_device rd;
                    unsigned int seed = rd();
                    std::mt19937 engine(seed);
                    RCLCPP_INFO(rclcpp::get_logger("patrol_4_node"),"Patro2Node initialized");
                    UNSIGNED_MAX = 4294967295;
                    seed = my_device();
                    engine = std::mt19937(seed);
                    ti = rclcpp::Clock().now();
                    getInput<geometry_msgs::msg::Point>("navigation_point",now_navigation_point);
                    getInput<double>("random_gen_limit",random_gen_limit);

                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    std::vector<std::vector<std::pair<double,double> > > generate_graph = toml::find<std::vector<std::vector<std::pair<double,double> > > >(toml_file , "random_restrict_area");
                    int area_id;getInput<int>("area_id",area_id);

                    restrict_area = generate_graph[area_id];

                    for(auto point : restrict_area){
                        mini_X = std::min(mini_X,point.first);
                        maxi_X = std::max(maxi_X,point.first);
                        mini_Y = std::min(mini_Y,point.second);
                        maxi_Y = std::max(maxi_Y,point.second);
                    }
                }


    double Patrol4Node::get_random01(){
        return 1.*engine() / UNSIGNED_MAX;
    }

    BT::NodeStatus Patrol4Node::tick(){
        double radium;
        if((rclcpp::Clock().now() - ti).seconds() < random_gen_limit) return BT::NodeStatus::SUCCESS;
        ti = rclcpp::Clock().now();
        const double PI = 3.14159265358979323846;
        double gen_angle = 2*PI*get_random01();
        double gen_distance = radium * get_random01();
        now_navigation_point = navigation_point;
        double pos_x,pos_y;
        
        pos_x = get_random01() * (maxi_X - mini_X) + mini_X;
        pos_y = get_random01() * (maxi_Y - mini_Y) + mini_Y;

        while(!decision_utils::judging_point::judging_point(std::pair<double,double>(pos_x,pos_y),restrict_area)){
            pos_x = get_random01() * (maxi_X - mini_X) + mini_X;
            pos_y = get_random01() * (maxi_Y - mini_Y) + mini_Y;
            // for(auto point : restrict_area){
            //     RCLCPP_INFO(rclcpp::get_logger("Patorl 4"),"area:%lf %lf",point.first,point.second);
            // }
        }
        // RCLCPP_INFO(rclcpp::get_logger("Patorl 4"),"area:%lf %lf",pos_x,pos_y);
        now_navigation_point.x = pos_x;
        now_navigation_point.y = pos_y;
        now_navigation_point.z = 0;
        // RCLCPP_INFO(rclcpp::get_logger("Patorl 4"),"Patrol4 6");
    
        setOutput<geometry_msgs::msg::Point>("now_navigation_point",now_navigation_point);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::Patrol4Node>("Patrol4Node");
// }