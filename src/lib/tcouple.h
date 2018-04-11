#ifndef V_TCOUPLE_H_
#define V_TCOUPLE_H_

#include <iostream>
#include "matcher.h"
#include <vector>


class TCouple{
    public:
        void saveToBinaryFile(std::string path);
        void loadFromBinaryFile(std::string path);
        std::vector<Matcher::p_match> matches_;
        std::vector<int32_t> features_count_;
        int p_frame_id_;
        int c_frame_id_;
        Matrix pose_;
};

#endif
