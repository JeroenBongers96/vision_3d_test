#ifndef PROCESSDATA_H
#define PROCESSDATA_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <ImageData.h>

class ProcessData
{
    private:
        ;
    public:
        ProcessData();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutROI(const ImageData &my_data, std::vector<int> roi_vec);
};

#endif