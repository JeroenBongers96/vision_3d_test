#ifndef SAVEDATA_H
#define SAVEDATA_H

#include <iostream>
#include <cstdlib> //Lib to create folder
#include <vector>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include "ImageData.h"
#include <opencv2/core/core.hpp>
#include <fstream>

using namespace std;
using namespace boost::filesystem;

class SaveData
{
    private:
        bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
        bool saveMatBinary(const std::string& filename, const cv::Mat& output);
    public:
        SaveData(ImageData &my_data);
        void saveData(ImageData &my_data);
};

#endif