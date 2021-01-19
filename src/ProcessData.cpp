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

// // ----------------------------------------------------------------------------------------------------

// /**
//  * Plane model segmentation extracts the planes of a point cloud using the RANSAC method as the robust estimator. * 
//  */
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segmentation::getPlainRANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
// {
//     std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> vec, filtered_vec;
    
//     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//     pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//     // Create the segmentation object
//     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//     // Optional
//     seg.setOptimizeCoefficients (true);
//     // Mandatory
//     seg.setModelType (pcl::SACMODEL_PLANE);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     //seg.setMaxIterations (10000);
//     seg.setOptimizeCoefficients(true);
//     seg.setDistanceThreshold (0.01); //0.01

//     // Create the filtering object
//     pcl::ExtractIndices<pcl::PointXYZRGB> extract;

//     int i = 0, nr_points = (int) cloud->points.size ();
//     // While 30% of the original cloud is still there
//     while (cloud->points.size () > 0.3 * nr_points)
//     {
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);

//         // Segment the largest planar component from the remaining cloud
//         seg.setInputCloud (cloud);
//         seg.segment (*inliers, *coefficients);
//         if (inliers->indices.size () == 0)
//         {
//         std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//         break;
//         }

//         // Extract the inliers
//         extract.setInputCloud (cloud);
//         extract.setIndices (inliers);
//         extract.setNegative (false);
//         extract.filter (*cloud_p);
//         std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;    
        
//         i++;
//     }   
    
//     return(cloud_p);
// }