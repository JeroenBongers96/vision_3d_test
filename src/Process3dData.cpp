#include "Process3dData.h"

Process3dData::Process3dData()
{
    ;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Cut objct out of point clouds based on getBinaryImg() from Process2dData 
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::cutObj(const cv::Mat &cv_img, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    int rows = cv_img.rows;
    int cols = cv_img.cols;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);

    // object->width = cols;
    // object->height = rows;
    // object->points.resize (object->width * object->height);
    
    const float bad_point = std::numeric_limits<float>::quiet_NaN();

    for(int y = 0; y < rows; ++y)
    {
        for(int x = 0; x < cols; ++x)
        {
            if(cv_img.at<uchar>(y,x) == 255 && cloud->at(x, y).z != 0)
            {
                object->push_back(cloud->at(x, y));
            }
        }
    }

    return( object ); 
}

// ----------------------------------------------------------------------------------------------------

/**
 * Cut ROI of point clouds
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::cutROI(const ImageData &my_data, std::vector<int> roi_vec)
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::getPlainRANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_plane->width = cloud->width;
    cloud_plane->height = cloud->height;
    cloud_plane->points.resize (cloud_plane->width * cloud_plane->height);

    cloud_object->width = cloud->width;
    cloud_object->height = cloud->height;
    cloud_object->points.resize (cloud_object->width * cloud_object->height);

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

// ----------------------------------------------------------------------------------------------------

/**
 * Derive roll pitch yaw from 4x4 matrix
 * Roll around X axis
 * Pitch around Y aixs
 * Yaw around Z axis
 * https://stackoverflow.com/questions/27508242/roll-pitch-and-yaw-from-rotation-matrix-with-eigen-library
 */
std::tuple<Eigen::Vector3f, Eigen::Quaternionf> Process3dData::getRotation(Eigen::Matrix4f mat4)
{
    //4*4 to 3*3 matrix
    Eigen::Matrix3f mat3 = Eigen::Matrix3f::Identity();
    mat3(0,0) = mat4(0,0);  mat3(0,1) = mat4(0,1);  mat3(0,2) = mat4(0,2);
    mat3(1,0) = mat4(1,0);  mat3(1,1) = mat4(1,1);  mat3(1,2) = mat4(1,2);
    mat3(2,0) = mat4(2,0);  mat3(2,1) = mat4(2,1);  mat3(2,2) = mat4(2,2);

    //Get Eulers in radians
    Eigen::Vector3f rpy = mat3.eulerAngles(0,1,2); //0 = roll, 1 = pitch, 2 = yaw
    Eigen::Quaternionf q(mat3);

    //Radians to degrees
    rpy[0] = (rpy[0]*180)/M_PI; 
    rpy[1] = (rpy[1]*180)/M_PI; 
    rpy[2] = (rpy[2]*180)/M_PI; 

    return(std::make_tuple(rpy, q));
}

// ----------------------------------------------------------------------------------------------------

/**
 * Get the transformation of a cloud
 */
std::tuple<Eigen::Matrix4f, Eigen::Vector3f, Eigen::Quaternionf, std::vector<pcl::PointXYZ>> Process3dData::momentOfInertia(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    copyPointCloud(*in_cloud, *cloud);  // Copy XYZRGB pcd to XYZ pcd ofr momentOfInertia functions

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;    
    
    feature_extractor.setInputCloud (cloud);
    feature_extractor.setNormalizePointMassFlag (true);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    //feature_extractor.setNormalizePointMassFlag(true);
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    //Points for PCL visualizer purpose
    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));    
    pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
    std::vector<pcl::PointXYZ> visualizer_odom {center, x_axis, y_axis, z_axis};

    //Matrix of cloud
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    //4x4 transformation of original cloud
    //Rotation from obb, position from mass center
    transform_1 (0,0) = rotational_matrix_OBB (0,0);  transform_1 (0,1) = rotational_matrix_OBB (0,1); transform_1 (0,2) = rotational_matrix_OBB (0,2); transform_1 (0,3) = center.x;
    transform_1 (1,0) = rotational_matrix_OBB (1,0);  transform_1 (1,1) = rotational_matrix_OBB (1,1); transform_1 (1,2) = rotational_matrix_OBB (1,2); transform_1 (1,3) = center.y;
    transform_1 (2,0) = rotational_matrix_OBB (2,0);  transform_1 (2,1) = rotational_matrix_OBB (2,1); transform_1 (2,2) = rotational_matrix_OBB (2,2); transform_1 (2,3) = center.z;
    transform_1 (3,0) = 0;  transform_1 (3,1) = 0; transform_1 (3,2) = 0; transform_1 (3,3) = 1;

    transform_1 (0,0) = -0.999722;
    transform_1 (2,0) = -0.0115021;

    std::cout << "11: " << transform_1 (0,0) << std::endl;
    // std::cout << "21: " << transform_1 (1,0) << std::endl;
    // std::cout << "22: " << transform_1 (1,1) << std::endl;
    // std::cout << "23: " << transform_1 (1,2) << std::endl;
    std::cout << "31: " << transform_1 (2,0) << std::endl;

    std::cout << transform_1 << std::endl;

    Eigen::Vector3f rpy;
    Eigen::Quaternionf q;
    std::tie(rpy, q) = getRotation(transform_1);   

    // bool debug = true; //Just comment when visualizer is required
    // if(debug)
    // {
    //     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //     viewer->setBackgroundColor (0, 0, 0);
    //     viewer->addCoordinateSystem (1.0);
    //     viewer->initCameraParameters ();
    //     viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    //     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    //     Eigen::Quaternionf quat (rotational_matrix_OBB);
    //     viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    //     viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    //     viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    //     viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    //     viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    //     while(!viewer->wasStopped())
    //     {
    //         viewer->spinOnce (100);
    //     }
    // }

    return(std::make_tuple(transform_1, rpy, q, visualizer_odom));
}