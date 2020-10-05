/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"

using namespace std;

bool debug = true;
bool create_data = true; //True = create, false = load
bool save_data = true;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

int main()
{
    cout << "main started" << endl;
    ImageData my_data;

    GetData get_data(debug, create_data, save_data);
    
    get_data.getData(my_data);

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