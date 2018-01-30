#include "tcouple.h"
#include <fstream>

void TCouple::saveToBinaryFile(std::string path){
    std::ofstream fout(path, std::ios::binary);
    fout.write((char*)&(p_frame_id_), sizeof(std::string));
    fout.write((char*)&(c_frame_id_), sizeof(std::string));
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++)
        fout.write((char*)&(pose_.val[i][j]),sizeof(FLOAT));
    }
    for (auto it = matches.begin(); it != matches.end(); ++it){
        fout.write((char*)&(*it), sizeof(Matcher::p_match));
    }
    fout.close();
}

void TCouple::loadFromBinaryFile(std::string path){
    std::ifstream fin(path, std::ios::binary);
    fin.read((char*)&(p_frame_id_), sizeof(std::string));
    fin.read((char*)&(c_frame_id_), sizeof(std::string));
    Matrix mat(4,4);
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++)
        fin.read((char*)&(mat.val[i][j]),sizeof(FLOAT));
    }
    pose_ = mat;
    for (auto it = matches.begin(); it != matches.end(); ++it){
        fin.read((char*)&(*it), sizeof(Matcher::p_match));
    }
    fin.close();
}