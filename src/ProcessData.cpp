#include "ProcessData.h"

ProcessData::ProcessData()
{
    ;
}

// ----------------------------------------------------------------------------------------------------

/**
 * 
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