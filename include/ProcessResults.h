#ifndef PROCESSRESULTS_H
#define PROCESSRESULTS_H

#include <iostream>
#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

class ProcessResults
{
    private:
        bool debug;
        std::tuple<Eigen::Vector3f, Eigen::Quaternionf> getRotation(Eigen::Matrix4f mat4);
        std::vector<float> getMedian(float xmin, float xmax, float ymin, float ymax, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    public:
        ProcessResults(bool debug);
        std::tuple<Eigen::Matrix4f, Eigen::Vector3f, Eigen::Quaternionf, vector<pcl::PointXYZ>> momentOfInertia(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
