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
#include "IDtoObject.h"
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
// rclcpp::Client<suii_communication::srv::YoloService>::SharedPtr client;
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
        std::cout << result_vec.size() << " = size, "  << result_vec[0] << ", " << result_vec[1] << ", " << result_vec[2] << ", " << result_vec[3] << ", " << result_vec[4] << std::endl;
        for(int i = 0; i < result_vec.size(); i++)
        {
            std::cout << "Element[" << i << "]: " << result_vec[i] << std::endl;
        }
        std::cout << "-------------" << std::endl;

    }
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
        exit(0);
    }

    std::cout << "client succeeded" << std::endl;

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
    Eigen::Matrix4f transform_object = Eigen::Matrix4f::Identity();
    Eigen::Vector3f rpy_table, rpy_object;
    Eigen::Quaternionf q_table, q_object;
    vector<pcl::PointXYZ> odom_table, odom_object;

    ImageData my_data;
    // GetRoi img_roi;
    GetData get_data(debug, create_data, save_data);
    Process3dData process3d;
    Process2dData process2d;
    IDtoObject ObjectID;

    // Get data
    get_data.getData(my_data);

    // Get object ROI from yolo
    // int* obj_roi_arr = yolo_client(my_data.cv_img);    

    std::vector<int> objects = yolo_client(my_data.cv_img);

    for(int x = 0; x < ( sizeof(objects) / 5 ); x ++)
    {
        cout << "ID: " << objects[x] << endl;
        cout << "Top left X coordinate: " << objects[x + 1] << endl;
        cout << "Top left Y coordinate: " << objects[x + 2] << endl;
        cout << "Bottom right X coordinate: " << objects[x + 3] << endl;
        cout << "Bottom right Y coordinate: " << objects[x + 4] << endl;

        x += 4; 
    }
    // rclcpp::shutdown();

    // Use Yolo and draw rectangle around ROI
    // roi_vect = img_roi.Yolo(argc, argv, my_data.cv_img, debug);
    // print(roi_vect);

    //YOLO output will be an Object ID int. This is converted to the object string with ConvertIDtoObject. like below.
    //std::string ObjName = IDconvObj.ConvertIDtoObject(ObjID);
    //std::cout << "Object name: " << ObjName << std::endl;

    // Create own rectangle to bypass Yolo. Purely for testing.
    cv::rectangle(my_data.cv_img, cv::Point(objects[0], objects[1]), cv::Point(objects[2], objects[3]), (0,255,0), 3);

    // Cut out ROI
    // object = process.cutROI(my_data, roi_vect);

    // Get table cloud
    table = process3d.getPlainRANSAC(my_data.original_cloud);

    // Get table transformation
    std::tie(transform_table, rpy_table, q_table, odom_table) = process3d.momentOfInertia(table);

    // Transform cloud to camera odom
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_on_origin = process3d.transformSfuToCameraOdom(my_data.original_cloud, transform_table);
    
    // Cut out object
    vector<int> roi_vect{objects[0], objects[1], objects[2], objects[3]};
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
    std::vector<std::string> item_ids = {"F20_20_G"};
    return item_ids;
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