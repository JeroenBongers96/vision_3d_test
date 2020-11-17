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

    // int i = 0;
    for(int y = 0; y < object->width; ++y)
    {
        for(int x = 0; x < object->height; ++x)
        {
            if(x >= roi_vec[0] && x <= roi_vec[2] && y >= roi_vec[1] && y <= roi_vec[3])
            {

                
                object->at(y, x).x = my_data.original_cloud->at(y, x).x;
                object->at(y, x).y = my_data.original_cloud->at(y, x).y;
                object->at(y, x).z = my_data.original_cloud->at(y, x).z;
                object->at(y, x).rgb = my_data.original_cloud->at(y, x).rgb;

                std::cout << "------" << std::endl;
                std::cout << object->at(y, x).x << std::endl;
                std::cout << object->at(y, x).y << std::endl;
                std::cout << object->at(y, x).z << std::endl;
                std::cout << object->at(y, x).rgb << std::endl;
                std::cout << my_data.original_cloud->at(y, x).rgb << std::endl;

            }
        }
    }

	return object;
}