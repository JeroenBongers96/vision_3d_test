/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"
#include "GetRoi.h"
#include "ProcessData.h"
#include <pcl/pcl_config.h>

using namespace std;

bool debug = true;
bool create_data = false; //True = create, false = load
bool save_data = false;

// vector<int> roi_vect;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);

int main(int argc, char** argv)
{
    cout << "main started" << endl;
    std::cout << "PCL version: " << PCL_VERSION << std::endl;

    ImageData my_data;
    GetRoi img_roi;
    GetData get_data(debug, create_data, save_data);
    ProcessData process;

    get_data.getData(my_data);

    //Use Yolo and draw rectangle around ROI
    // roi_vect = img_roi.Yolo(argc, argv, my_data.cv_img, debug);
    // print(roi_vect);
    cout << "CV mat rows: " << my_data.cv_img.rows << endl;
    cout << "CV mat cols: " << my_data.cv_img.cols << endl;
    vector<int> roi_vect{100, 100, 400, 400};
    cv::rectangle(my_data.cv_img, Point(roi_vect[0], roi_vect[1]), Point(roi_vect[2], roi_vect[3]), (0,255,0), 3);

    object = process.cutROI(my_data, roi_vect);

    // Display cv img in a GUI
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", my_data.cv_img);
    cv::waitKey(0);

    //Add cloud to visualizer
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // Set background of viewer to black
    viewer.setBackgroundColor (0, 0, 0); 
    // Viewer Properties
    viewer.initCameraParameters();  // Camera Parameters for ease of viewing

    // viewer.addPointCloud (my_data.original_cloud,"pcd");
    viewer.addPointCloud (object,"pcd");

    // Default size for rendered points
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pcd");

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));        
    }    

    return 0;
};