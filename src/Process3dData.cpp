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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::cutROI(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<int> roi_vec)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // object->width = cloud->width;
    // object->height = cloud->height;
    // object->points.resize (object->width * object->height);

    cv::Mat cv_img = cv::Mat::zeros(cv::Size(cloud->width,cloud->height),CV_8UC3);

    // int i = 0;
    for(int y = 0; y < cloud->height; ++y)
    {
        for(int x = 0; x < cloud->width; ++x)
        {
            if(x >= roi_vec[0] && x <= roi_vec[2] && y >= roi_vec[1] && y <= roi_vec[3] && cloud->at(x, y).z < -0.006 && cloud->at(x, y).z > -0.05)
            {                
                // object->at(x, y).x = cloud->at(x, y).x;
                // object->at(x, y).y = cloud->at(x, y).y;
                // object->at(x, y).z = cloud->at(x, y).z;
                // object->at(x, y).rgb = cloud->at(x, y).rgb;

                // pcl::PointXYZRGB point = cloud->at(x, y);
                object->push_back(cloud->at(x, y));
            }
        }
    }

    std::cout << object->size() << std::endl;

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

    Eigen::Vector3f rpy;
    Eigen::Quaternionf q;
    std::tie(rpy, q) = getRotation(transform_1);   

    return(std::make_tuple(transform_1, rpy, q, visualizer_odom));
}


// ----------------------------------------------------------------------------------------------------

/**
 * Transform a point cloud parallel to the camera using inverse matrices 
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::transformSfuToCameraOdom(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const Eigen::Matrix4f &mat)
{    
    //Take inverse from original transformation to set it paralel to the camera frame which is the origin
    Eigen::Matrix4f inverse = mat.inverse();

    // Executing the transformation to set the cloud parallel to the camera
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, inverse);

    return (transformed_cloud);
}

// ----------------------------------------------------------------------------------------------------

/**
 * Transform a point cloud back to the original matrix 
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::transformSfuToOriginalOdom(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const Eigen::Matrix4f &mat)
{    
    // Executing the transformation to set the cloud parallel to the camera
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, mat);

    return (transformed_cloud);
}

// ----------------------------------------------------------------------------------------------------

/**
 * Filter outlier from PC by using Outlier Removal filter
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Process3dData::orFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    double meanK = 30.0, mulThresh = 0.1; 

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (mulThresh);
    sor.filter (*pc_filtered);

    return pc_filtered;
}
