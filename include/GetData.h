#ifndef GETDATA_H
#define GETDATA_H

#include <librealsense2/rs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

class GetData
{
    private:
        bool debug, create_data, save_data;
        std::tuple<int, int, int> rgbTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclConversion(const rs2::points& points, const rs2::video_frame& color);
    public:
        GetData(bool debug, bool create_data, bool save_data);
        void getData();

};

#endif