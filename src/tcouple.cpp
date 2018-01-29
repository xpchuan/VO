#include "tcouple.h"
#include <ofstream>

TCouple::saveToBinaryFile(std::string path){
    std::ofstream fout(path, std::ios::binary);
    fout.write((char*)&(f_frame_id_), sizeof(std::string));
    fout.write((char*)&(b_frame_id_), sizeof(std::string));
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++)
        fout.write((char*)&(pose.val[i][j]),sizeof(FLOAT));
    }
    for (auto it = matches.begin(), it != matches.end(), ++it){
        fout.write((char*)&(*it), sizeof(Matcher::p_match));
    }
    fout.close();
}