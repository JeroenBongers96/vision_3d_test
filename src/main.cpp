/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"

using namespace std;

bool debug = true;
bool create_data = false; //True = create, false = load
bool save_data = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

int main()
{
    cout << "main started" << endl;
    
    GetData my_data(debug, create_data, save_data);
    my_data.getData();

    return 0;
};