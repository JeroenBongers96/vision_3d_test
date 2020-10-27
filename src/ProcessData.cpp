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

    int i = 0;
    for(int y = 0; y < 480; ++y)
    {
        for(int x = 0; x < 640; ++x)
        {
            if(x >= roi_vec[1] && x <= roi_vec[3] && y >= roi_vec[2] && y <= roi_vec[4])
            {
                object->points[i].x = my_data.original_cloud->points[i].x;
                object->points[i].y = my_data.original_cloud->points[i].y;
                object->points[i].z = my_data.original_cloud->points[i].z;
                object->points[i].rgb = my_data.original_cloud->points[i].rgb;
            }
            ++i;
        }
    }

	return object;
}