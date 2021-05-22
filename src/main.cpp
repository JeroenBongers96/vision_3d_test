/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"
// #include "GetRoi.h"
#include "Process2dData.h"
#include "Process3dData.h"
#include "Visualize.h"
#include <pcl/pcl_config.h>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/buffer_core.h>
// #include <tf2/exceptions.h>
#include <tf2/time.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/buffer_interface.h>
#include <tf2_ros/static_transform_broadcaster.h>
// #include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include "suii_communication/srv/vision_scan.hpp"  
#include "suii_communication/srv/yolo_service.hpp"  

using namespace std;

std::shared_ptr<rclcpp::Node> node;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr table (new pcl::PointCloud<pcl::PointXYZRGB>);

// ----------------------------------------------------------------------------------------------------

/**
 * Broadcasts object tf
 */
void rosBroadcaster(Eigen::Matrix4f transform, tf2::Quaternion q_tf)
{
    tf2_ros::StaticTransformBroadcaster stb(node);
    geometry_msgs::msg::TransformStamped ts;

    ts.header.frame_id = "Cam";
    ts.child_frame_id = "obj";
    ts.header.stamp = rclcpp::Time();
    ts.transform.translation.x = transform(0,3);
    ts.transform.translation.y = transform(1,3);
    ts.transform.translation.z = transform(2,3);
    ts.transform.rotation.x = q_tf.x();
    ts.transform.rotation.y = q_tf.y();
    ts.transform.rotation.z = q_tf.z();
    ts.transform.rotation.w = q_tf.w();

    stb.sendTransform(ts);
}

// ----------------------------------------------------------------------------------------------------

/**
 * Scan for area and publish all tf's of all found objects
 */
std::vector<int> scan_all(bool debug, bool create_data, bool save_data)
{
    cout << "main started" << endl;
    cout << "-------" << endl;

    std::cout << "PCL version: " << PCL_VERSION << std::endl;

    Eigen::Matrix4f transform_table = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_object = Eigen::Matrix4f::Identity();
    Eigen::Vector3f rpy_table, rpy_object;
    Eigen::Quaternionf q_table, q_object;
    vector<pcl::PointXYZ> odom_table, odom_object;

    ImageData my_data;
    // GetRoi img_roi;
    GetData get_data(debug, create_data, save_data);
    Process3dData process3d;
    Process2dData process2d;

    // Get data
    get_data.getData(my_data);

    //https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros
    cv::Mat img; // << image MUST be contained here
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    // sensor_msgs::msg::Image msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_data.cv_img).toImageMsg();

    // Use Yolo and draw rectangle around ROI
    // roi_vect = img_roi.Yolo(argc, argv, my_data.cv_img, debug);
    // print(roi_vect);

    // Create own rectangle to bypass Yolo. Purely for testing.
    vector<int> roi_vect{250, 100, 400, 300};
    cv::rectangle(my_data.cv_img, cv::Point(roi_vect[0], roi_vect[1]), cv::Point(roi_vect[2], roi_vect[3]), (0,255,0), 3);

    // Cut out ROI
    // object = process.cutROI(my_data, roi_vect);

    // Get table cloud
    table = process3d.getPlainRANSAC(my_data.original_cloud);

    // Get table transformation
    std::tie(transform_table, rpy_table, q_table, odom_table) = process3d.momentOfInertia(table);

    // Transform cloud to camera odom
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_on_origin = process3d.transformSfuToCameraOdom(my_data.original_cloud, transform_table);
    
    // Cut out object
    object = process3d.cutROI(cloud_on_origin, roi_vect); 

    // Transform object back to its original position
    object = process3d.transformSfuToOriginalOdom(object, transform_table);

    // object = process3d.orFilter(object);

    // Get table transformation
    std::tie(transform_object, rpy_object, q_object, odom_object) = process3d.momentOfInertia(object);

    // tf_publisher.publish_tf();

    tf2::Quaternion q_tf(q_object.x(), q_object.y(), q_object.z(), q_object.w());
    rosBroadcaster(transform_object, q_tf);
    
    // Show result
    if(debug)
        {   
            cout << endl;

            cout << "===Position==================" << endl;
            cout << "X axis: " << transform_table(0,3) << endl;
            cout << "Y axis: " << transform_table(1,3) << endl;
            cout << "Z axis: " << transform_table(2,3) << endl << endl;
            
            cout << "===Rotations in Euler==================" << endl;
            cout << "Rotation around X axis (Roll): " << rpy_table[0] << "°" << endl;
            cout << "Rotation around Y axis (Pitch): " << rpy_table[1] << "°" << endl;
            cout << "Rotation around Z axis (Yaw): " << rpy_table[2] << "°" << endl << endl;

            // cout << "===Rotations in Quaternion==================" << endl;
            // cout << "Rotation quaternion x: " << q_tf[0] << endl;
            // cout << "Rotation quaternion y: " << q_tf[1] << endl;
            // cout << "Rotation quaternion z: " << q_tf[2] << endl;
            // cout << "Rotation quaternion w: " << q_tf[3] << endl;

            cout << "===Location matrix=====================" << endl << endl;
            cout << transform_table << endl;

            Visualize vis(debug);
            shared_ptr<pcl::visualization::PCLVisualizer> viewer = vis.createViewer();
            viewer = vis.addOriginalColorCloud(viewer, table);
            // viewer = vis.addCustomColorCloud(viewer, object);
            viewer = vis.addCustomColorCloud(viewer, object);
            viewer = vis.addOdom(viewer, odom_table);
            viewer = vis.addOdom(viewer, odom_object);
            vis.visualizeCV(my_data.cv_img);
            vis.visualizePCL(viewer);
        }

    // CHANGE this return to yolo ID outcome
    std::vector<int> item_ids = {1, 2, 3};
    return item_ids;
} 

void scan_service(const std::shared_ptr<suii_communication::srv::VisionScan::Request> request,     // CHANGE
          std::shared_ptr<suii_communication::srv::VisionScan::Response>       response)  // CHANGE
{
    
    std::cout << "Debug: " << request->debug << std::endl;
    std::cout << "Create data: " << request->create_data << std::endl;
    std::cout << "Save data: " << request->save_data << std::endl;

    // Scan all objects
    std::vector<int> item_ids = scan_all(request->debug, request->create_data, request->save_data);

    // Convert vector to response array
    response->detected_objects.resize(item_ids.size()); 

    for(int i = 0; i < item_ids.size(); i++)
    {
        response->detected_objects[i] = item_ids[i];
    }
}


void node_test_service(const std::shared_ptr<suii_communication::srv::VisionScan::Request> request,     // CHANGE
          std::shared_ptr<suii_communication::srv::VisionScan::Response>       response)  // CHANGE
{
    
    std::cout << "Debug: " << request->debug << std::endl;
    std::cout << "Create data: " << request->create_data << std::endl;
    std::cout << "Save data: " << request->save_data << std::endl;

    // Scan all objects
    std::vector<int> item_ids = scan_all(request->debug, request->create_data, request->save_data);

    // Convert vector to response array
    response->detected_objects.resize(item_ids.size()); 

    for(int i = 0; i < item_ids.size(); i++)
    {
        response->detected_objects[i] = item_ids[i];
    }
}

void node_tester(std::shared_ptr<rclcpp::Node> node)
{
    rclcpp::Service<suii_communication::srv::VisionScan>::SharedPtr service_2 =                 
        node->create_service<suii_communication::srv::VisionScan>("node_test",  &node_test_service);
}

// int main(int argc, char** argv)
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("vision_server");  

    rclcpp::Service<suii_communication::srv::VisionScan>::SharedPtr service =                 
        node->create_service<suii_communication::srv::VisionScan>("vision_scan",  &scan_service);     

    
    node_tester(node);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to scan objects.");      

    // std::cout << rclcpp::Time() << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();
};