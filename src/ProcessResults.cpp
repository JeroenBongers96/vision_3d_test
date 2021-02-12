#include "ProcessResults.h"

ProcessResults::ProcessResults(bool debug)
{
    this->debug = debug;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Get the transformation of a cloud
 */
std::tuple<Eigen::Matrix4f, Eigen::Vector3f, Eigen::Quaternionf, vector<pcl::PointXYZ>> ProcessResults::momentOfInertia(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
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
    vector<pcl::PointXYZ> visualizer_odom {center, x_axis, y_axis, z_axis};

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

    debug = false; //Just comment when visualizer is required
    if(debug)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

        Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat (rotational_matrix_OBB);
        viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

        viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
        viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
        viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    while(!viewer->wasStopped())
    {
      viewer->spinOnce (100);
    }
    }

    return(std::make_tuple(transform_1, rpy, q, visualizer_odom));
}

// ----------------------------------------------------------------------------------------------------

/**
 * Derive roll pitch yaw from 4x4 matrix
 * Roll around X axis
 * Pitch around Y aixs
 * Yaw around Z axis
 * https://stackoverflow.com/questions/27508242/roll-pitch-and-yaw-from-rotation-matrix-with-eigen-library
 */
std::tuple<Eigen::Vector3f, Eigen::Quaternionf> ProcessResults::getRotation(Eigen::Matrix4f mat4)
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
