#ifndef STRING_PARSER
#define STRING_PARSER

#include <toml.hpp>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>

#define HEIGHT 240
#define WIDTH 240

namespace decision_utils{
    namespace string_parser{
        std::vector<int> GetIntArray(std::string str);
    };
}

#endif