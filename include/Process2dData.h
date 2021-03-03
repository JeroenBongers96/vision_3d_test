#ifndef PROCESS2DDATA_H
#define PROCESSDDDATA_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

class Process2dData
{
    private:
        cv::Mat removeNoneRoi(const cv::Mat& original_img, std::vector<int> roi_vect);
        void edgeDetection(const cv::Mat& img);
    public:
        Process2dData();
        cv::Mat getBinaryImg(const cv::Mat& original_img, std::vector<int> roi_vect);
};

#endif