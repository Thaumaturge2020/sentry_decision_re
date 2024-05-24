#ifndef JUDEGING_POINT
#define JUDEGING_POINT

#include <toml.hpp>
#include <utility>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

#define HEIGHT 240
#define WIDTH 240

namespace decision_utils{
    namespace judging_point{
        bool judging_point(std::pair<double,double> point, std::vector<std::pair<double,double>>poin_arr);
        void draw_point(std::pair<double,double> point, std::vector<std::pair<double,double>>poin_arr);
        double calculate_area(std::pair<double,double> point , std::pair<double,double> point1,std::pair<double,double> point2);
        double get_distance(std::pair<double,double> point1,std::pair<double,double> point2);
        bool read_judging();
        
        double get_distance(std::pair<double,double> point,std::pair<std::pair<double,double>,std::pair<double,double> > segment);
        double get_distance(std::pair<double,double> point1,std::pair<double,double> point2);
    };
}

#endif