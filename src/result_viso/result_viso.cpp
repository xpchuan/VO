#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

#include "tcouple.h"
#include "matcher.h"

int main(int argc, char ** argv){
    std::string data_dir = "/home/hesai/project/libviso2/data/dataset/sequences/00";
    std::string result_dir = "/home/hesai/project/libviso2/result";
    std::string im_result_dir = "/home/hesai/project/libviso2/im_result";
    cv::Mat c_left,p_left,pp_left,c_right,p_right,pp_right;
    int i = 2;
    for(; i < 200; i++){
        std::cout<<"start:"<<std::to_string(i)<<std::endl;
        char pp_base_name[256]; sprintf(pp_base_name,"%06d.png",i-2);
        char p_base_name[256]; sprintf(p_base_name,"%06d.png",i-1);
        char c_base_name[256]; sprintf(c_base_name,"%06d.png",i);
        pp_left = cv::imread(data_dir + "/image_0/" + std::string(pp_base_name),cv::IMREAD_GRAYSCALE);
        pp_right = cv::imread(data_dir + "/image_1/" + std::string(pp_base_name),cv::IMREAD_GRAYSCALE);
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
        cv::Mat pp_right_color; 
        cv::cvtColor(pp_right, pp_right_color, CV_GRAY2RGB);
        cv::Mat pp_left_color; 
        cv::cvtColor(pp_left, pp_left_color, CV_GRAY2RGB);


        TCouple* ccouple = new TCouple();
        TCouple* pcouple = new TCouple();
        ccouple->loadFromBinaryFile(result_dir + "/couple-" + std::to_string(i-1) + "-" + std::to_string(i));
        pcouple->loadFromBinaryFile(result_dir + "/couple-" + std::to_string(i-2) + "-" + std::to_string(i-1));
        std::vector<Matcher::p_match> cmatches = ccouple->matches;
        std::vector<Matcher::p_match>  pmatches = pcouple->matches;
        int value[3] = {0, 0 , 255};
        for(auto it:cmatches){
            for(int i = 0; i < 3; i++){
                p_left_color.at<cv::Vec3b>(it.v1p, it.u1p)[i] = value[i];
                p_right_color.at<cv::Vec3b>(it.v2p, it.u2p)[i] = value[i];
                c_left_color.at<cv::Vec3b>(it.v1c, it.u1c)[i] = value[i];
                c_right_color.at<cv::Vec3b>(it.v2c, it.u2c)[i] = value[i];
            }
        }
        value[1] = 255;
        for(auto it:pmatches){
            for(int i = 0; i < 3; i++){
                p_left_color.at<cv::Vec3b>(it.v1p, it.u1p)[i] = value[i];
                p_right_color.at<cv::Vec3b>(it.v2p, it.u2p)[i] = value[i];
                pp_left_color.at<cv::Vec3b>(it.v1p, it.u1p)[i] = value[i];
                pp_right_color.at<cv::Vec3b>(it.v2p, it.u2p)[i] = value[i];
            }
        }

        pmatches.clear();
        cmatches.clear();
        cv::Mat sumVleft(p_left.rows * 2 , p_left.cols, CV_8UC3);
        cv::vconcat(c_left_color, p_left_color, sumVleft);
        cv::Mat sumVleft2(p_left.rows * 3 , p_left.cols, CV_8UC3);
        cv::vconcat(sumVleft, pp_left_color, sumVleft2);

        cv::Mat sumVright(p_left.rows, p_left.cols * 2, CV_8UC3);
        cv::vconcat(c_right_color, p_right_color, sumVright);
        cv::Mat sumVright2(p_left.rows, p_left.cols * 3, CV_8UC3);

        cv::vconcat(sumVright, pp_right_color, sumVright2);
        cv::Mat sumIm(p_left.rows * 3, p_left.cols * 3, CV_8UC3);
        cv::hconcat(sumVleft2, sumVright2, sumIm);

        cv::imwrite(im_result_dir +  "/im-couple-" + std::to_string(i-1) + "-" + std::to_string(i) + ".png", sumIm);
        std::cout<<"finish:"<<std::to_string(i)<<std::endl;
        std::cout<<"end:"<<std::to_string(i)<<std::endl;
        delete ccouple;
        delete pcouple;
    }
}
