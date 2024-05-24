#ifndef ID_MAPPING
#define ID_MAPPING

#include <toml.hpp>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>

#define HEIGHT 240
#define WIDTH 240

namespace decision_utils{
    namespace id_mapping{
        int autoaim2priority(int id);
        int priority2autoaim(int id);
        int autoaim2referee(int id);
        int referee2autoaim(int id);
    };
}

#endif