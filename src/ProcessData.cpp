#include "ProcessData.h"

ProcessData::ProcessData()
{
    ;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Cut ROI of point clouds
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProcessData::cutROI(const ImageData &my_data, std::vector<int> roi_vec)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    object->width = my_data.original_cloud->width;
    object->height = my_data.original_cloud->height;
    object->points.resize (object->width * object->height);

    cv::Mat cv_img = cv::Mat::zeros(cv::Size(object->width,object->height),CV_8UC3);

    // int i = 0;
    for(int y = 0; y < object->height; ++y)
    {
        for(int x = 0; x < object->width; ++x)
        {
            //
            if(x >= roi_vec[0] && x <= roi_vec[2] && y >= roi_vec[1] && y <= roi_vec[3])
            {

                
                object->at(x, y).x = my_data.original_cloud->at(x, y).x;
                object->at(x, y).y = my_data.original_cloud->at(x, y).y;
                object->at(x, y).z = my_data.original_cloud->at(x, y).z;
                object->at(x, y).rgb = my_data.original_cloud->at(x, y).rgb;

                pcl::PointXYZRGB point = my_data.original_cloud->at(x, y);

                // std::cout << "b [pcl] : " << (int)point.b << std::endl; 
                // std::cout << "g [pcl] : " << (int)point.g << std::endl; 
                // std::cout << "r [pcl] : " << (int)point.r << std::endl; 
            }

                // memcpy( &cv_img.at<cv::Vec3b>(y,x)[0], (int)point.b, sizeof(point.b) );
            // cv_img.at<cv::Vec3b>(y,x)[0] = (int)point.b;
            // cv_img.at<cv::Vec3b>(y,x)[1] = (int)point.g;
            // cv_img.at<cv::Vec3b>(y,x)[2] = (int)point.r;

                // std::cout << "b [cv] : " << (int)cv_img.at<cv::Vec3b>(y,x)[0] << std::endl; 
                // std::cout << "g [cv] : " << (int)cv_img.at<cv::Vec3b>(y,x)[1] << std::endl; 
                // std::cout << "r [cv] : " << (int)cv_img.at<cv::Vec3b>(y,x)[2] << std::endl; 
                // }
        }
    }

    // cv::namedWindow("PCL image", cv::WINDOW_AUTOSIZE );
    // cv::imshow("PCL image", cv_img);
    // cv::waitKey(0);

    // std::vector<int> compression_params;
    // compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    // compression_params.push_back(9);
    // std::string save_name = "/home/jeroen/cv_img.png";
    // cv::imwrite(save_name, cv_img, compression_params);


	return object;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Plane model segmentation extracts the planes of a point cloud using the RANSAC method as the robust estimator. * 
 */
// Segmentation member function
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProcessData::getPlainRANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.005); // Original: 0.02

    // int i=0, nr_points = (int) cloud->points.size ();
    // while (cloud->points.size () > 0.3 * nr_points)
    // {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    // if (inliers->indices.size () == 0)
    // {
    //     break;
    // }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_object);
    //std::cout << "PointCloud representing the segmented component: " << cloud_f->points.size () << " data points." << std::endl;
    // *cloud = *cloud_f;
    // }

    return cloud_plane;
}