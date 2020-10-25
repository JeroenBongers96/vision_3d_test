/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"
#include "GetRoi.h"

using namespace std;

bool debug = true;
bool create_data = false; //True = create, false = load
bool save_data = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

int main(int argc, char** argv)
{
    cout << "main started" << endl;

    ImageData my_data;
    
    GetRoi img_roi;

    GetData get_data(debug, create_data, save_data);
    
    get_data.getData(my_data);

    img_roi.Yolo(argc, argv, my_data.cv_img, debug);

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

    viewer.addPointCloud (my_data.original_cloud,"pcd");

    // Default size for rendered points
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pcd");

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));        
    }    

    return 0;
};