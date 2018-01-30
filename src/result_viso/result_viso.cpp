#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

#include "tcouple.h"
#include "matcher.h"

int main(int argc, char ** argv){
    std::string data_dir = "/home/hesai/project/libviso2/data/dataset/sequences/00";
    std::string result_dir = "/home/hesai/project/libviso2/result";
    std::string im_result_dir = "/home/hesai/project/libviso2/im_result";
    cv::Mat c_left,p_left,c_right,p_right;
    int i = 1;
    for(; i < 200; i++){
         std::cout<<"start:"<<std::to_string(i)<<std::endl;
        char p_base_name[256]; sprintf(p_base_name,"%06d.png",i-1);
        char c_base_name[256]; sprintf(c_base_name,"%06d.png",i);
        p_left = cv::imread(data_dir + "/image_0/" + std::string(p_base_name),cv::IMREAD_GRAYSCALE);
        p_right = cv::imread(data_dir + "/image_1/" + std::string(p_base_name),cv::IMREAD_GRAYSCALE);
        c_left = cv::imread(data_dir + "/image_0/" + std::string(c_base_name),cv::IMREAD_GRAYSCALE);
        c_right = cv::imread(data_dir + "/image_1/" + std::string(c_base_name),cv::IMREAD_GRAYSCALE);

        TCouple* couple = new TCouple();
        couple->loadFromBinaryFile(result_dir + "/couple-" + std::to_string(i-1) + "-" + std::to_string(i));
        int nRows = p_left.rows * 2;
        int nCols = p_left.cols * 2;
        std::vector<Matcher::p_match> matches = couple->matches;
        for(auto it:matches){
            p_left.at<unsigned char>(it.u1p, it.v1p) = 0;
            p_right.at<unsigned char>(it.u2p, it.v2p) = 0;
            c_left.at<unsigned char>(it.u1c, it.v1c) = 0;
            c_right.at<unsigned char>(it.u2c, it.v2c) = 0;
        }
        matches.clear();
        cv::Mat sumVleft(nRows, p_left.cols, CV_8UC1);
        cv::vconcat(p_left, c_left, sumVleft);
        cv::Mat sumVright(p_left.rows, nCols, CV_8UC1);
        cv::vconcat(p_right, c_right, sumVright);
        cv::Mat sumIm(nRows, nCols, CV_8UC1);
        cv::hconcat(sumVleft, sumVright, sumIm);
        cv::imwrite(im_result_dir +  "/im-couple-" + std::to_string(i-1) + "-" + std::to_string(i) + ".png", sumIm);
        std::cout<<"finish:"<<std::to_string(i)<<std::endl;
        delete couple;
    }
}