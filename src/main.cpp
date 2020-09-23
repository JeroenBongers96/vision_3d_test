/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"

using namespace std;

bool debug = true;
bool create_data = false; //True = create, false = load
bool save_data = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

int main()
{
    cout << "main started" << endl;
    ImageData my_data;

    GetData get_data(debug, create_data, save_data);
    
    get_data.getData(my_data);

    return 0;
};