#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

#include "tcouple.h"
#include "matcher.h"

int main(int argc, char ** argv){
    std::string data_dir = "/home/hesai/project/VO/data";
    std::string result_dir = "/home/hesai/project/VO/result";
    std::string im_result_dir = "/home/hesai/project/VO/im_result";
    cv::Mat c_left,p_left,c_right,p_right;
    int i = 1;
    for(; i < 800; i++){
        std::cout<<"start:"<<std::to_string(i)<<std::endl;
        char p_base_name[256]; sprintf(p_base_name,"%06d.png",i-1);
        char c_base_name[256]; sprintf(c_base_name,"%06d.png",i);
        p_left = cv::imread(data_dir + "/image_0/" + std::string(p_base_name),cv::IMREAD_GRAYSCALE);
        p_right = cv::imread(data_dir + "/image_1/" + std::string(p_base_name),cv::IMREAD_GRAYSCALE);
        c_left = cv::imread(data_dir + "/image_0/" + std::string(c_base_name),cv::IMREAD_GRAYSCALE);
        c_right = cv::imread(data_dir + "/image_1/" + std::string(c_base_name),cv::IMREAD_GRAYSCALE);

        cv::Mat p_left_color;
        cv::cvtColor(p_left, p_left_color, CV_GRAY2RGB);
        cv::Mat p_right_color; 
        cv::cvtColor(p_right, p_right_color, CV_GRAY2RGB);
        cv::Mat c_left_color; 
        cv::cvtColor(c_left, c_left_color, CV_GRAY2RGB);
        cv::Mat c_right_color; 
        cv::cvtColor(c_right, c_right_color, CV_GRAY2RGB);


        TCouple* couple = new TCouple();
        couple->loadFromBinaryFile(result_dir + "/couple-" + std::to_string(i-1) + "-" + std::to_string(i));
        int nRows = p_left.rows * 2;
        int nCols = p_left.cols * 2;
        std::vector<Matcher::p_match> matches = couple->matches_;
        int value[3] = {0, 0 , 255};
        for(auto it:matches){
            cv::Point center_pl = cv::Point(it.u1p, it.v1p);
            cv::Point center_pr = cv::Point(it.u2p, it.v2p);
            cv::Point center_cl = cv::Point(it.u1c, it.v1c);
            cv::Point center_cr = cv::Point(it.u2c, it.v2c);
            cv::circle(p_left_color, center_pl, 2, cv::Scalar(value[0], value[1],value[2]));
            cv::circle(p_right_color, center_pr, 2, cv::Scalar(value[0], value[1],value[2]));
            cv::circle(c_left_color, center_cl, 2, cv::Scalar(value[0], value[1],value[2]));
            cv::circle(c_right_color, center_cr, 2, cv::Scalar(value[0], value[1],value[2]));
        }
        matches.clear();
        cv::Mat sumVleft(nRows, p_left.cols, CV_8UC3);
        cv::vconcat(p_left_color, c_left_color, sumVleft);
        cv::Mat sumVright(p_left.rows, nCols, CV_8UC3);
        cv::vconcat(p_right_color, c_right_color, sumVright);
        cv::Mat sumIm(nRows, nCols, CV_8UC3);
        cv::hconcat(sumVleft, sumVright, sumIm);
        cv::imwrite(im_result_dir +  "/im-couple-" + std::to_string(i-1) + "-" + std::to_string(i) + ".png", sumIm);
        std::cout << "output:" << i <<std::endl;
        delete couple;
    }
}