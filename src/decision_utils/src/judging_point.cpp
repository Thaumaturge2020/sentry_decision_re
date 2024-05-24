#include "decision_utils/judging_point.hpp"

namespace decision_utils{
    namespace judging_point{
        double calculate_area(std::pair<double,double> point , std::pair<double,double> point1,std::pair<double,double> point2){
            double x1=point.first-point1.first;
            double y1=point.second-point1.second;
            double x2=point.first-point2.first;
            double y2=point.second-point2.second;
            return x1*y2-x2*y1;
        }

        double get_distance(std::pair<double,double> point1,std::pair<double,double> point2){
            return sqrt((point1.first - point2.first)*(point1.first - point2.first)+(point1.second - point2.second)*(point1.second - point2.second));
        }

        bool calculate_moment(std::pair<double,double> point , std::pair<double,double> point1,std::pair<double,double> point2){
            double  x1 = point.first - point1.second,
                    y1 = point.first - point1.second,
                    x2 = point.first - point1.second,
                    y2 = point.first - point1.second;

            return x1*x2+y1*y2;
        }

        bool check_sim(double S1,double S2){
            if(fabs(S1-S2) < 1e-5 || (S1-S2)/std::max(std::fabs(S1),fabs(S2)) < 1e-5)
            return true;
            else
            return false;
        }    

        bool judging_point(std::pair<double,double> point, std::vector<std::pair<double,double>>poin_arr){
            double s=0,s0=0;
            poin_arr.push_back(poin_arr[0]);
            for (std::vector<std::pair<double,double>> ::iterator i=begin(poin_arr) ; i!=end(poin_arr)-1; i++){
                s+=fabs(calculate_area(point,*i,*(i+1)));
                s0+=fabs(calculate_area(poin_arr[0],*i,*(i+1)));
            }
            return check_sim(s,s0);
        }

        void draw_point(std::pair<double,double> point, std::vector<std::pair<double,double>>poin_arr){
            std::pair<double,double> first_point=poin_arr[0];
            cv::Mat test_image(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
            for(std::vector<std::pair<double,double>> ::iterator i=begin(poin_arr) ; i!=end(poin_arr)-1 ; i++)
            {
                cv::line(test_image, cv::Point((*i).first,(*i).second), cv::Point((*(i+1)).first, (*(i+1)).second), cv::Scalar(0, 0, 255), 1, 8);
            }
            cv::line(test_image,cv::Point ((*(end(poin_arr)-1)).first,(*(end(poin_arr)-1)).second),cv::Point (first_point.first,first_point.second),cv::Scalar(0,0,255),1,8);
            cv::circle(test_image,cv::Point (point.first,point.second),2,cv::Scalar(0, 255,0),-1);
            cv::imwrite("../test_image.png",test_image);
        }
        bool read_judging(){
            std::vector<std::pair<double,double>> poin_arr;
            std::pair<double,double> point;
            const auto points_data = toml::parse("config/points.toml");
            poin_arr = toml::find<std::vector<std::pair<double,double>>>(points_data,"points_array");
            point = toml::find<std::pair<double,double>>(points_data,"signal_point");
            draw_point(point,poin_arr);
            return judging_point(point,poin_arr);
        }

        double get_distance(std::pair<double,double> point,std::pair<std::pair<double,double>,std::pair<double,double> > segment){
            if(calculate_moment(segment.first,segment.second,point) <= 0 || calculate_moment(segment.second,segment.first,point) <= 0){
                double dis1 = get_distance(point,segment.first),dis2 = get_distance(point,segment.second);
                return dis1 < dis2 ? dis1:dis2;
            }
            
            double  A = segment.first.second - segment.second.second,B = segment.second.first - segment.first.first,
                    C = segment.first.first * segment.second.second - segment.second.first * segment.first.second;
            
            return (A*point.first + B*point.second + C) / sqrt(A*A + B*B);
        }
    }
}