#include "node_library/param_node_1.hpp"


namespace BehaviorTree{

    //use for random patrol

    ParamNode1::ParamNode1(const std::string& name, const BT::NodeConfig& config):
                BT::SyncActionNode(name,config){
                    getInput<int>("decision_type",decision_type);
                    const auto toml_file = toml::parse(ROOT "config/battle_information.toml");
                    const auto my_outpost_blood_lowerbound_array = toml::find<std::vector<int>>(toml_file, "my_outpost_blood_lowerbound");
                    my_outpost_blood_lowerbound = my_outpost_blood_lowerbound_array[decision_type];
                    const auto my_outpost_blood_upperbound_array = toml::find<std::vector<int>>(toml_file, "my_outpost_blood_upperbound");
                    my_outpost_blood_upperbound = my_outpost_blood_upperbound_array[decision_type];
                    const auto enemy_outpost_blood_lowerbound_array = toml::find<std::vector<int>>(toml_file, "enemy_outpost_blood_lowerbound");
                    enemy_outpost_blood_lowerbound = enemy_outpost_blood_lowerbound_array[decision_type];
                    const auto enemy_outpost_blood_upperbound_array = toml::find<std::vector<int>>(toml_file, "enemy_outpost_blood_upperbound");
                    enemy_outpost_blood_upperbound = enemy_outpost_blood_upperbound_array[decision_type];
                    const auto area_choose_array = toml::find<std::vector<int>>(toml_file, "area_choose");
                    area_choose = area_choose_array[decision_type];
                    const auto chase_distance_able_array = toml::find<std::vector<double>>(toml_file, "chase_distance_able");
                    chase_distance_able = chase_distance_able_array[decision_type];
                    const auto chase_distance_limit_array = toml::find<std::vector<double>>(toml_file, "chase_distance_limit");
                    chase_distance_limit = chase_distance_limit_array[decision_type];
                    const auto priority_input_array = toml::find<std::vector<std::string> >(toml_file, "priority_input");
                    priority_input = priority_input_array[decision_type];
                    const auto priotiry_type_array = toml::find<std::vector<std::string> >(toml_file,"priority_type");
                    priority_type = priotiry_type_array[decision_type];
                }

    BT::NodeStatus ParamNode1::tick(){

        setOutput<int>("my_outpost_blood_lowerbound",my_outpost_blood_lowerbound);
        setOutput<int>("my_outpost_blood_upperbound",my_outpost_blood_upperbound);
        setOutput<int>("enemy_outpost_blood_lowerbound",enemy_outpost_blood_lowerbound);
        setOutput<int>("enemy_outpost_blood_upperbound",enemy_outpost_blood_upperbound);
        setOutput<int>("area_choose",area_choose);
        setOutput<double>("chase_distance_able",chase_distance_able);
        setOutput<double>("chase_distance_limit",chase_distance_limit);
        setOutput<std::string>("priority_input",priority_input);
        setOutput<std::string>("gimbal_choose_enemy_priority_input",priority_type);
        return BT::NodeStatus::SUCCESS;
    }
}

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<BehaviorTree::ParamNode1>("ParamNode1");
// }