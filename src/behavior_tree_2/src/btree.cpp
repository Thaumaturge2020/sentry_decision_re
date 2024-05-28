#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "node_library/all_type_node.hpp"
#include "node_library/time_compu_interval.hpp"
#include "node_library/get_opblood_node.hpp"
#include "node_library/judgeoutpost.hpp"
#include "node_library/judge_enemy.hpp"
// #include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a* member function as a callback from the timer. */


  static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <PipelineSequence>
      <BaseStaticAttackNode given_id="1" target_id="{attack_id}"/>
      <BaseAttackSpecificEnemyNode >
    </PipelineSequence>
  </BehaviorTree>
</root>
 )";


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);    

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BehaviorTree::DebugNode>("Debugnode");
    factory.registerNodeType<BehaviorTree::BuyBulletNode>("BuyBulletNode");
    factory.registerNodeType<BehaviorTree::build_attack_node>("BuildAttackNode");
    factory.registerNodeType<BehaviorTree::BaseDefenseCondition>("BaseDefenseCondition");
    factory.registerNodeType<BehaviorTree::ChassisAttackSpecificEnemyNode>("ChassisAttackSpecificEnemyNode");
    factory.registerNodeType<BehaviorTree::ChassisStaticAttackNode>("ChassisStaticAttackNode");
    factory.registerNodeType<BehaviorTree::ChassisDecideSpeedNode>("ChassisDecideSpeedNode");
    factory.registerNodeType<BehaviorTree::CheckValNode>("CheckValNode");
    factory.registerNodeType<BehaviorTree::CheckIfFirstBlood>("CheckIfFirstBlood");
    factory.registerNodeType<BehaviorTree::CheckIfBloodInterval>("CheckIfBloodInterval");
    factory.registerNodeType<BehaviorTree::CheckIfLostHP>("CheckIfLostHP");
    factory.registerNodeType<BehaviorTree::CheckIfMonitorEnemy>("CheckIfMonitorEnemy");
    factory.registerNodeType<BehaviorTree::ComputeAreaChoose>("ComputeAreaChoose");
    factory.registerNodeType<BehaviorTree::DebugNode>("DebugNode");
    factory.registerNodeType<BehaviorTree::DecideEnemy>("DecideEnemy");
    factory.registerNodeType<BehaviorTree::DefenceBuildingNode>("DefenceBuildingNode");
    factory.registerNodeType<BehaviorTree::GimbalChooseEnemyNode>("GimbalChooseEnemyNode");
    factory.registerNodeType<BehaviorTree::GimbalChooseBuildingNode>("GimbalChooseBuildingNode");
    factory.registerNodeType<BehaviorTree::lidar_attack_node>("LidarAttackNode");
    factory.registerNodeType<BehaviorTree::OnFireNode>("OnFireNode");
    factory.registerNodeType<BehaviorTree::nav_heal_node>("NavHealNode");
    factory.registerNodeType<BehaviorTree::NavToEnemy>("NavToEnemy");
    factory.registerNodeType<BehaviorTree::NavToSpecificPlace>("NavToSpecificPlace");
    factory.registerNodeType<BehaviorTree::NavToBuildingNode>("NavToBuildingNode");
    factory.registerNodeType<BehaviorTree::OperatorNode>("OperatorNode");
    factory.registerNodeType<BehaviorTree::ParamNode1>("ParamNode1");
    factory.registerNodeType<BehaviorTree::Patrol1Node>("Patrol1Node");
    factory.registerNodeType<BehaviorTree::Patrol2Node>("Patrol2Node");
    factory.registerNodeType<BehaviorTree::Patrol3Node>("Patrol3Node");
    factory.registerNodeType<BehaviorTree::Patrol4Node>("Patrol4Node");
    factory.registerNodeType<BehaviorTree::RadarDecision>("RadarDecision");
    factory.registerNodeType<BehaviorTree::Spin>("SpinNode");
    factory.registerNodeType<BehaviorTree::SetValNode>("SetValNode");
    factory.registerNodeType<BehaviorTree::SetValOnceNode>("SetValOnceNode");
    factory.registerNodeType<BehaviorTree::TimeBegin>("TimeBegin");
    factory.registerNodeType<BehaviorTree::TimeCompuLimit>("TimeCompuLimit");
    factory.registerNodeType<BehaviorTree::TimeCompuInterval>("TimeCompuInterval");
    factory.registerNodeType<BehaviorTree::TimeCtrl>("TimeCtrl");
    factory.registerNodeType<BehaviorTree::StayOriNode>("StayOriNode");
    factory.registerNodeType<BehaviorTree::Judgenemy>("Judgenemy");
    factory.registerNodeType<BehaviorTree::ResurrectionNode>("ResurrectionNode");
    factory.registerNodeType<BehaviorTree::PriorityOnlyNode>("PriorityOnlyNode");
    // factory.registerNodeType<BehaviorTree::GetOpbloodNode>("GetOpbloodNode");
    // factory.registerNodeType<BehaviorTree::Judgenmey_0_0>("Judgenmey_0_0");
    // factory.registerNodeType<BehaviorTree::judgeoutpost>("judgeoutpost");

    // factory.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
    RCLCPP_INFO(rclcpp::get_logger("this is my logger"),"MYMYMY");
    const auto toml_file = toml::parse(ROOT "config/config.toml");
    std::string my_str = toml::find<std::string>(toml_file,"tree_file");
    std::string cfg_str = "config/";
    auto tree = factory.createTreeFromFile(ROOT + cfg_str + my_str);
    RCLCPP_INFO(rclcpp::get_logger("this is my logger"),"FINISH TREE INITIALIZATION");
    // auto nh=std::make_shared<Checkblood::SyncActionNode>();
    //rclcpp::spin(nh);
    while(rclcpp::ok()){     //rclcpp::ok()
       tree.tickWhileRunning();
    }
    rclcpp::shutdown();
    return 0;
}
