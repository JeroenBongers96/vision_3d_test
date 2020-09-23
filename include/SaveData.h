#ifndef SAVEDATA_H
#define SAVEDATA_H

#include <iostream>
#include <cstdlib> //Lib to create folder
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include "ImageData.h"

using namespace std;
using namespace boost::filesystem;

class SaveData
{
    private:
    public:
        SaveData(ImageData &my_data);
        void saveData(ImageData &my_data);
};

#endif