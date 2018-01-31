#include "tcouple.h"
#include <fstream>
        
void TCouple::saveToBinaryFile(std::string path){
    std::ofstream fout(path, std::ios::binary);
    fout.write((char*)&(p_frame_id_), sizeof(int));
    fout.write((char*)&(c_frame_id_), sizeof(int));
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++)
        fout.write((char*)&(pose_.val[i][j]),sizeof(FLOAT));
    }
    int size = matches.size();
    fout.write((char*)&(size), sizeof(int));
    for (auto it = matches.begin(); it != matches.end(); ++it){
        fout.write((char*)&(*it), sizeof(Matcher::p_match));
    }
    fout.close();
}

void TCouple::loadFromBinaryFile(std::string path){
    std::ifstream fin(path, std::ios::binary);
    fin.read((char*)&(p_frame_id_), sizeof(int));
    fin.read((char*)&(c_frame_id_), sizeof(int));
    //Matrix mat(4,4);
    pose_ = Matrix::eye(4);
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++){
            FLOAT read_buffer;
            fin.read((char*)&(read_buffer),sizeof(FLOAT));
            pose_.val[i][j] = read_buffer;
        }
    }
    //pose_ = mat;
    int size;
    fin.read((char*)&(size), sizeof(int));
    for (int i = 0; i < size; i++){
        Matcher::p_match match;
        fin.read((char*)&(match), sizeof(Matcher::p_match));
        matches.push_back(match);
    }
    fin.close();
}
