/**
 * Main code for testing box measuring with 2D depth map
 */

#include <iostream>
#include "GetData.h"
#include "ImageData.h"
#include "Process2dData.h"
#include "Process3dData.h"
#include "Visualize.h"
#include "IDtoObject.h"
#include <pcl/pcl_config.h>
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>
#include <tf2_ros/static_transform_broadcaster.h>
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
void rosBroadcaster(Eigen::Matrix4f transform, tf2::Quaternion q_tf, std::string name)
{
    tf2_ros::StaticTransformBroadcaster stb(node);
    geometry_msgs::msg::TransformStamped ts;

    ts.header.frame_id = "Cam";
    ts.child_frame_id = name;
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
 * Get the ROI's of objects from yolo_server
 */
std::vector<int> yolo_client(cv::Mat img)
{
    std::shared_ptr<rclcpp::Node> node_2 = rclcpp::Node::make_shared("yolov5_client");
    rclcpp::Client<suii_communication::srv::YoloService>::SharedPtr client =
        node_2->create_client<suii_communication::srv::YoloService>("yolo_service_msg");

    auto request = std::make_shared<suii_communication::srv::YoloService::Request>();

    std::vector<int> result_vec;

    cv_bridge::CvImage cvi;
    cvi.encoding = sensor_msgs::image_encodings::BGR8;
    cvi.image = img;

    sensor_msgs::msg::Image::SharedPtr img_msg = cvi.toImageMsg();

    request->img = *img_msg; 

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        // return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    
    std::cout <<  "printing object arr" << std::endl;

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_2, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request succeeded");

        //Read result data and print size + some data
        result_vec = result.get()->obj_roi_arr; 
    }
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        exit(0);
    }

    return result_vec;
}

// ----------------------------------------------------------------------------------------------------

/**
 * Scan for area and publish all tf's of all found objects
 */
std::vector<std::string> scan_all(bool debug, bool create_data, bool save_data)
{
    cout << "main started" << endl;
    cout << "-------" << endl;

    std::cout << "PCL version: " << PCL_VERSION << std::endl;

    Eigen::Matrix4f transform_table = Eigen::Matrix4f::Identity();
    Eigen::Vector3f rpy_table;
    Eigen::Quaternionf q_table;
    vector<pcl::PointXYZ> odom_table;
    std::vector<std::string> item_list;
    shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    ImageData my_data;
    GetData get_data(debug, create_data, save_data);
    Process3dData process3d;
    Process2dData process2d;
    IDtoObject ObjectID;
    Visualize vis(debug);

    // Get data
    get_data.getData(my_data);

    // Get object ROI from yolo
    std::vector<int> objects = yolo_client(my_data.cv_img); 

    // Get table cloud
    table = process3d.getPlainRANSAC(my_data.original_cloud);
    
    // Get table transformation
    std::tie(transform_table, rpy_table, q_table, odom_table) = process3d.momentOfInertia(table);

    if(debug)
    {
        viewer = vis.createViewer();
        viewer = vis.addOriginalColorCloud(viewer, table);
        // viewer = vis.addOdom(viewer, odom_table); // Add table odometry
    }

    // Transform cloud to camera odom
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_on_origin = process3d.transformSfuToCameraOdom(my_data.original_cloud, transform_table);

    // Process all found objects

    std::cout << "Size of result: " << sizeof(objects) << std::endl;  
    for(int x = 0; x < objects.size(); x ++)
    {
        Eigen::Matrix4f transform_object = Eigen::Matrix4f::Identity();
        Eigen::Vector3f rpy_object;
        Eigen::Quaternionf q_object;
        std::vector<pcl::PointXYZ> odom_table, odom_object;
        
        std::string name = ObjectID.ConvertIDtoObject(objects[x]);
        item_list.push_back(name);

        cout << "ID: " << objects[x] << " | name: " <<  name << endl;
        cout << "YOLO top left X coordinate: " << objects[x + 1] << endl;
        cout << "YOLO top left Y coordinate: " << objects[x + 2] << endl;
        cout << "YOLO bottom right X coordinate: " << objects[x + 3] << endl;
        cout << "YOLO bottom right Y coordinate: " << objects[x + 4] << endl;

        // Create cv rectangle from Yolo data.
        cv::rectangle(my_data.cv_img, cv::Point(objects[x+1], objects[x+2]), cv::Point(objects[x+3], objects[x+4]), (0,255,0), 3);

        // Cut out object
        vector<int> roi_vect{objects[x+1], objects[x+2], objects[x+3], objects[x+4]};
        object = process3d.cutROI(cloud_on_origin, roi_vect);

        // Transform object back to its original position
        object = process3d.transformSfuToOriginalOdom(object, transform_table);

        // Get object transformation
        std::tie(transform_object, rpy_object, q_object, odom_object) = process3d.momentOfInertia(object);

        // Broadcast transformation
        tf2::Quaternion q_tf(q_object.x(), q_object.y(), q_object.z(), q_object.w());
        name = name + "_" + to_string(x);
        rosBroadcaster(transform_object, q_tf, name);

        if(debug)
        {
            viewer = vis.addCustomColorCloud(viewer, object);
            viewer = vis.addOdom(viewer, odom_object);
        }

        // Go to next object
        x += 4; 
    }
    
    // Show results
    if(debug)
        {   
            // cout << "===Position==================" << endl;
            // cout << "X axis: " << transform_table(0,3) << endl;
            // cout << "Y axis: " << transform_table(1,3) << endl;
            // cout << "Z axis: " << transform_table(2,3) << endl << endl;
            
            // cout << "===Rotations in Euler==================" << endl;
            // cout << "Rotation around X axis (Roll): " << rpy_table[0] << "°" << endl;
            // cout << "Rotation around Y axis (Pitch): " << rpy_table[1] << "°" << endl;
            // cout << "Rotation around Z axis (Yaw): " << rpy_table[2] << "°" << endl << endl;

            // cout << "===Rotations in Quaternion==================" << endl;
            // cout << "Rotation quaternion x: " << q_tf[0] << endl;
            // cout << "Rotation quaternion y: " << q_tf[1] << endl;
            // cout << "Rotation quaternion z: " << q_tf[2] << endl;
            // cout << "Rotation quaternion w: " << q_tf[3] << endl;

            // cout << "===Location matrix=====================" << endl << endl;
            // cout << transform_table << endl;
            
            vis.visualizeCV(my_data.cv_img);
            vis.visualizePCL(viewer);
        }

    return item_list;
} 

void scan_service(const std::shared_ptr<suii_communication::srv::VisionScan::Request> request,     // CHANGE
          std::shared_ptr<suii_communication::srv::VisionScan::Response>       response)  // CHANGE
{

    int array_size = response->detected_objects.size();

    std::cout << "Array size: " << array_size << std::endl;
    std::cout << "Debug: " << request->debug << std::endl;
    std::cout << "Create data: " << request->create_data << std::endl;
    std::cout << "Save data: " << request->save_data << std::endl;

    // Scan all objects
    std::vector<std::string> scanned_items = scan_all(request->debug, request->create_data, request->save_data);

    response->detected_objects[0] = scanned_items[0];
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("vision_server");  

    rclcpp::Service<suii_communication::srv::VisionScan>::SharedPtr service =                 
        node->create_service<suii_communication::srv::VisionScan>("vision_scan",  &scan_service);     

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to scan objects.");      

    rclcpp::spin(node);
    rclcpp::shutdown();
};