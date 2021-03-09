#ifndef GETROI_H
#define GETROI_H

#include <iostream>
#include <opencv2/opencv.hpp> 
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "vision_3d_test/GetRoi.h"

using namespace std;

class GetRoi
{
    private:

    public:
        GetRoi();
        vector<int> Yolo(int argc, char **argv, cv::Mat img, bool debug);
};

#endif