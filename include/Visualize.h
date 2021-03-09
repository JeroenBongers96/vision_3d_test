#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <thread>

using namespace std;

class Visualize
{
    private:
        bool debug;       
        int cloud_counter = 0;
        vector<int> color_changer(int count);
    public:
        Visualize(bool debug);
        void visualizeCV(cv::Mat img);
        shared_ptr<pcl::visualization::PCLVisualizer> createViewer();        
        shared_ptr<pcl::visualization::PCLVisualizer> addCustomColorCloud(shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud); 
        shared_ptr<pcl::visualization::PCLVisualizer> addOriginalColorCloud(shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);  
        shared_ptr<pcl::visualization::PCLVisualizer> addOdom(shared_ptr<pcl::visualization::PCLVisualizer> viewer, vector<pcl::PointXYZ> odom);
        void visualizePCL(shared_ptr<pcl::visualization::PCLVisualizer> viewer);

};

#endif