#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>

struct ImageData {
    std::string folder_path, save_name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud;
};

#endif