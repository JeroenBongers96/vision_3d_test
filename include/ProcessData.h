#ifndef PROCESSDATA_H
#define PROCESSDATA_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <ImageData.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class ProcessData
{
    private:
        ;
    public:
        ProcessData();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutROI(const ImageData &my_data, std::vector<int> roi_vec);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlainRANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
};

#endif