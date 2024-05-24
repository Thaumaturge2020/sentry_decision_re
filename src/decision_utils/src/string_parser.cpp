#include "decision_utils/string_parser.hpp"

namespace decision_utils{
    namespace string_parser{
        std::vector<int> GetIntArray(std::string str){
            int len = str.size();
            int num = -1;
            std::vector<int> vec;
            for(int i=0;i<len;++i){
                if(str[i] < '0' || str[i] > '9'){
                    if(num >= 0)
                    vec.push_back(num);
                    num = -1;
                    continue;
                }
                if(num < 0)
                num = 0;
                num = num*10 + str[i] - '0';
            }
            return vec;
        }
    };
}