/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <time.h>

#include <viso_stereo.h>
#include <png++/png.hpp>

#include <fstream>

//using namespace std;

std::string root_dir = "/home/hesai/project/VO";


int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  //if (argc<2) {
  //  cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
  //  return 1;
  //}

  // sequence directory
  //string dir = argv[1];
  std::string dir = root_dir + "/data";
  
  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryStereo::parameters param;

  // calibration parameters for sequence 2010_03_09_drive_0019 
  param.calib.f  = 7.188560000000e+02; // focal length in pixels
  param.calib.cu = 6.071928000000e+02; // principal point (u-coordinate) in pixels
  param.calib.cv = 1.852157000000e+02; // principal point (v-coordinate) in pixels
  param.base     = 0.5371657188644179; // baseline in meters
  
  // init visual odometry
  VisualOdometryStereo viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
  std::vector<Matrix>  pose_vec;
  // loop through all frames i=0:372
  clock_t start_time = 0;
  double sum_time = 0;
  double sum_matches = 0;
  double sum_inliers = 0;

  for (int32_t i=0; i<=1100; i++) {

    start_time = clock();
    // input file names
    char base_name[256]; sprintf(base_name,"%06d.png",i);
    std::string left_img_file_name  = dir + "/image_0/" + std::string(base_name);
    std::string right_img_file_name = dir + "/image_1/" + std::string(base_name);
    
    // catch image read/write errors here
    try {
      //init save
      TCouple* couple;
      if(i > 0)
        couple = new TCouple();
      else
        couple = NULL;

      // load left and right input image
      png::image< png::gray_pixel > left_img(left_img_file_name);
      png::image< png::gray_pixel > right_img(right_img_file_name);
      if(couple){
         couple->p_frame_id_ = i-1;
         couple->c_frame_id_ = i;
      }
      // image dimensions
      int32_t width  = left_img.get_width();
      int32_t height = left_img.get_height();

      // convert input images to uint8_t buffer
      uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      int32_t k=0;
      for (int32_t v=0; v<height; v++) {
        for (int32_t u=0; u<width; u++) {
          left_img_data[k]  = left_img.get_pixel(u,v);
          right_img_data[k] = right_img.get_pixel(u,v);
          k++;
        }
      }

      // status
      std::cout << "Processing: Frame: " << i;
      // compute visual odometry
      int32_t dims[] = {width,height,width};
      if (viso.process(left_img_data,right_img_data,dims,couple,i)) {
      
        // on success, update current pose
        pose = pose * Matrix::inv(viso.getMotion());
        sum_time += static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC;
        // output some statistics
        double num_matches = viso.getNumberOfMatches();
        double num_inliers = viso.getNumberOfInliers();
        sum_matches += num_matches;
        sum_inliers += num_inliers;
        std::cout << ", Matches: " << num_matches;
        std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << std::endl;
        if (num_matches < 1000)
            std::cout << i << " matched badly"<< std::endl;
        //std::cout << pose << std::endl;
        pose_vec.push_back(pose);
        couple->pose_ = pose;

      } else {
        std::cout << " ... failed!" << std::endl;
      }

      if(couple){
         couple->saveToBinaryFile(root_dir+ "/result/couple-" + std::to_string(i-1) + "-" + std::to_string(i));
         delete couple;
         couple = NULL;
      }
      // release uint8_t buffers
      free(left_img_data);
      free(right_img_data);

    // catch image read errors here
    } catch (...) {
      std::cerr << "ERROR: Couldn't read input files!" << std::endl;
      return 1;
    }
  }
  
  // output
  std::cout << "Demo complete! Exiting ..." << std::endl;


  //save result
  std::ofstream fout(root_dir + 
            "/Matlab_process/pose_result.bin",std::ios::binary);
  for(std::vector<Matrix>::iterator it = pose_vec.begin();
                                   it != pose_vec.end(); ++it){
    for(int i = 0; i < 4; i++){
      for(int j = 0; j < 4; j++)
        fout.write((char*)&(it->val[i][j]),sizeof(FLOAT));
    }
  }
  std::cout << "length:" << pose_vec.size() << std::endl;
  std::cout << "fps:" << 1.0 / sum_time * pose_vec.size() << std::endl;
  std::cout << "avg_matches:" << sum_matches / pose_vec.size() << std::endl;
  std::cout << "avg_inliers:" << sum_inliers / pose_vec.size() << std::endl;
  fout.close();

  // exit
  return 0;
}

