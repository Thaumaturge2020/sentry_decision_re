#include "decision_utils/id_mapping.hpp"


namespace decision_utils{
    namespace id_mapping{
        int autoaim2priority(int id){
            int ret = 0;
            switch((id-1)%9 + 1){
                case 1:ret = 0;break;
                case 2:ret = 1;break;
                case 3:ret = 2;break;
                case 4:ret = 3;break;
                case 5:ret = 4;break;
                case 7:ret = 5;break;
                case 8:ret = 6;break;
                case 9:ret = 7;break;
            }
            return ret;
        }

        int priority2autoaim(int id){
            int ret = 0;
            switch(id){
                case 0:ret = 1;break;
                case 1:ret = 2;break;
                case 2:ret = 3;break;
                case 3:ret = 4;break;
                case 4:ret = 5;break;
                case 5:ret = 7;break;
                case 6:ret = 8;break;
                case 7:ret = 9;break;
            }
            return ret;
        }

        int referee2autoaim(int id){
            int ret = 0;
            switch(id){
                case 1:ret = 1;break;
                case 2:ret = 2;break;
                case 3:ret = 3;break;
                case 4:ret = 4;break;
                case 5:ret = 5;break;
                case 7:ret = 0;break;
                case 8:ret = 7;break;
                case 9:ret = 8;break;
                case 101:ret = 9 + 1;break;
                case 102:ret = 9 + 2;break;
                case 103:ret = 9 + 3;break;
                case 104:ret = 9 + 4;break;
                case 105:ret = 9 + 5;break;
                case 107:ret = 9 + 0;break;
                case 108:ret = 9 + 7;break;
                case 109:ret = 9 + 8;break;
            }
            return ret;
        }

        int autoaim2referee(int id){
            int ret = 0;
            switch(id){
                case 1:ret = 1;break;
                case 2:ret = 2;break;
                case 3:ret = 3;break;
                case 4:ret = 4;break;
                case 5:ret = 5;break;
                case 0:ret = 0;break;
                case 7:ret = 7;break;
                case 8:ret = 8;break;
                case 10:ret = 100 + 1;break;
                case 11:ret = 100 + 2;break;
                case 12:ret = 100 + 3;break;
                case 13:ret = 100 + 4;break;
                case 14:ret = 100 + 5;break;
                case 9 :ret = 100 + 7;break;
                case 16:ret = 100 + 8;break;
                case 17:ret = 100 + 9;break;
            }
            return ret;
        }
    }
}