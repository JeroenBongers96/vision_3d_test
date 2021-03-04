#ifndef PROCESS3DDATA_H
#define PROCESS3DDATA_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <ImageData.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

class Process3dData
{
    private:
        std::tuple<Eigen::Vector3f, Eigen::Quaternionf> getRotation(Eigen::Matrix4f mat4);
    public:
        Process3dData();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutObj(const cv::Mat &cv_img, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutROI(const ImageData &my_data, std::vector<int> roi_vec);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlainRANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        std::tuple<Eigen::Matrix4f, Eigen::Vector3f, Eigen::Quaternionf, std::vector<pcl::PointXYZ>> momentOfInertia(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};

#endif