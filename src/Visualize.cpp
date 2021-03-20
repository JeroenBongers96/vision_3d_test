#include "Visualize.h"

Visualize::Visualize(bool debug)
{
    this->debug = debug;
}

/**
 * Visualize OpenCV format images
 */
void Visualize::visualizeCV(cv::Mat img)
{
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Display window", img );
    cv::waitKey(0); //0 to wait for user input to proceed
}

// ----------------------------------------------------------------------------------------------------

/**
 * Create PCL viewer
 * Red arrow = X
 * Green arrow = Y
 * Blue arrow = Z
 */
shared_ptr<pcl::visualization::PCLVisualizer> Visualize::createViewer()
{
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    viewer->addCoordinateSystem(0.5);
    viewer->initCameraParameters();    
    return viewer;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Add clouds to the viewer with a custom color
 */
shared_ptr<pcl::visualization::PCLVisualizer> Visualize::addCustomColorCloud(shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cloud_counter++;
    string cloud_name = "cloud_" + to_string(cloud_counter);

    vector<int> color = color_changer(cloud_counter);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb(cloud, color.at(0), color.at(1), color.at(2));
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, cloud_name);

    return viewer;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Add clouds to the viewer with original color
 */
shared_ptr<pcl::visualization::PCLVisualizer> Visualize::addOriginalColorCloud(shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "Original cloud");

    return viewer;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Add odom to the viewer
 */
shared_ptr<pcl::visualization::PCLVisualizer> Visualize::addOdom(shared_ptr<pcl::visualization::PCLVisualizer> viewer, vector<pcl::PointXYZ> odom)
{
    odom_counter++;
    string major_eigen_vector = "major_eigen_vector_" + to_string(odom_counter);
    string middle_eigen_vector = "middle_eigen_vector_" + to_string(odom_counter);
    string minor_eigen_vector = "minor_eigen_vector_" + to_string(odom_counter);

    viewer->addLine (odom.at(0), odom.at(1), 1.0f, 0.0f, 0.0f, major_eigen_vector);
    viewer->addLine (odom.at(0), odom.at(2), 0.0f, 1.0f, 0.0f, middle_eigen_vector);
    viewer->addLine (odom.at(0), odom.at(3), 0.0f, 0.0f, 1.0f, minor_eigen_vector);

    return viewer;
}


// ----------------------------------------------------------------------------------------------------

/**
 * Visualize PCL format images
 */
void Visualize::visualizePCL(shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while(!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }    
    viewer->close();
}

// ----------------------------------------------------------------------------------------------------

/**
 * Changes color of clouds
 */
vector<int> Visualize::color_changer(int count)
{
    vector<int> color(3);
    switch(count)
    {
        case 1: 
            color.at(0) = 204;
            color.at(1) = 102;
            color.at(2) = 0;
            break;
        case 2: 
            color.at(0) = 204;
            color.at(1) = 255;
            color.at(2) = 153;
            break;
        case 3: 
            color.at(0) = 255;
            color.at(1) = 255;
            color.at(2) = 0;
            break;
        case 4: 
            color.at(0) = 153;
            color.at(1) = 153;
            color.at(2) = 0;
            break;
        case 5: 
            color.at(0) = 255;
            color.at(1) = 128;
            color.at(2) = 0;
            break;
        case 6: 
            color.at(0) = 178;
            color.at(1) = 255;
            color.at(2) = 102;
            break;
        case 7: 
            color.at(0) = 51;
            color.at(1) = 255;
            color.at(2) = 153;
            break;
        case 8: 
            color.at(0) = 153;
            color.at(1) = 153;
            color.at(2) = 255;
            break;
        case 9: 
            color.at(0) = 255;
            color.at(1) = 153;
            color.at(2) = 204;
            break;
        case 10: 
            cout << "Max cloud_counter in visualize class is reached" << endl;
            cloud_counter = 0;
            color.at(0) = 255;
            color.at(1) = 255;
            color.at(2) = 255;
            break;
    }

    return(color);
}
