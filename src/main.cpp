/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"
// #include "GetRoi.h"
#include "Process2dData.h"
#include "Process3dData.h"
#include "Visualize.h"
#include <pcl/pcl_config.h>

using namespace std;

bool debug = true;
bool create_data = false; //True = create, false = load
bool save_data = false;

// vector<int> roi_vect;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr table (new pcl::PointCloud<pcl::PointXYZRGB>);

// int main(int argc, char** argv)
int main()
{
    cout << "main started" << endl;
    cout << "-------" << endl;

    std::cout << "PCL version: " << PCL_VERSION << std::endl;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    Eigen::Vector3f rpy;
    Eigen::Quaternionf q;
    vector<pcl::PointXYZ> odom;

    ImageData my_data;
    // GetRoi img_roi;
    GetData get_data(debug, create_data, save_data);
    Process3dData process3d;
    Process2dData process2d;

    // Get data
    get_data.getData(my_data);

    // Get binary image of object
    vector<int> roi_vect{250, 100, 400, 300};
    cv::Mat bin_img = process2d.getBinaryImg(my_data.cv_img, roi_vect);

    // Get point cloud of object
    object = process3d.cutObj(bin_img, my_data.original_cloud);

    // Use Yolo and draw rectangle around ROI
    // roi_vect = img_roi.Yolo(argc, argv, my_data.cv_img, debug);
    // print(roi_vect);

    // Create own rectangle to bypass Yolo. Purely for testing.
    // cout << "CV mat rows: " << my_data.cv_img.rows << endl;
    // cout << "CV mat cols: " << my_data.cv_img.cols << endl;
    // cv::rectangle(my_data.cv_img, cv::Point(roi_vect[0], roi_vect[1]), cv::Point(roi_vect[2], roi_vect[3]), (0,255,0), 3);s

    // Cut out ROI
    // object = process.cutROI(my_data, roi_vect);

    // Get table cloud
    table = process3d.getPlainRANSAC(my_data.original_cloud);

    // Get transformation
    std::tie(transform, rpy, q, odom) = process3d.momentOfInertia(object);
    
    // Ros quaternion transformation, this can be broadcaster
    // tf2::Quaternion q_tf(q.x(), q.y(), q.z(), q.w()); 

    // Show result
    if(debug)
        {   
            cout << endl;

            cout << "===Position==================" << endl;
            cout << "X axis: " << transform(0,3) << endl;
            cout << "Y axis: " << transform(1,3) << endl;
            cout << "Z axis: " << transform(2,3) << endl << endl;
            
            cout << "===Rotations in Euler==================" << endl;
            cout << "Rotation around X axis (Roll): " << rpy[0] << "°" << endl;
            cout << "Rotation around Y axis (Pitch): " << rpy[1] << "°" << endl;
            cout << "Rotation around Z axis (Yaw): " << rpy[2] << "°" << endl << endl;

            // cout << "===Rotations in Quaternion==================" << endl;
            // cout << "Rotation quaternion x: " << q_tf[0] << endl;
            // cout << "Rotation quaternion y: " << q_tf[1] << endl;
            // cout << "Rotation quaternion z: " << q_tf[2] << endl;
            // cout << "Rotation quaternion w: " << q_tf[3] << endl;

            cout << "===Location matrix=====================" << endl << endl;
            cout << transform << endl;

            Visualize vis(debug);
            shared_ptr<pcl::visualization::PCLVisualizer> viewer = vis.createViewer();
            vis.visualizeCV(bin_img);
            // viewer = vis.addOriginalColorCloud(viewer, my_data.original_cloud);
            // viewer = vis.addCustomColorCloud(viewer, table);
            viewer = vis.addCustomColorCloud(viewer, object);
            viewer = vis.addOdom(viewer, odom);
            vis.visualizePCL(viewer);
        }

    return 0;
};