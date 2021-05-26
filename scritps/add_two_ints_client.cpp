#include "rclcpp/rclcpp.hpp"
#include "suii_communication/srv/add_two_ints.hpp"
#include "suii_communication/srv/yolo_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

void set_now(builtin_interfaces::msg::Time & time)
{
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  if (now <= std::chrono::nanoseconds(0)) 
  {
    time.sec = time.nanosec = 0;
  } 
  else 
  {
    time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    time.nanosec = now.count() % 1000000000;
  }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //   if (argc != 3) {
    //       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    //       return 1;
    //   }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<suii_communication::srv::YoloService>::SharedPtr client =
        node->create_client<suii_communication::srv::YoloService>("yolo_service_msg");

    auto request = std::make_shared<suii_communication::srv::YoloService::Request>();
    //   request->a = atoll(argv[1]);
    //   request->b = atoll(argv[2]);

    std::string img_path = "/home/robohub/ws/src/vision_3d_test/test_img/test.jpeg";
        // string test_img = "/home/jeroen/cv_img.png";
    cv::Mat frame_ = cv::imread(img_path);

    // // Create a new unique_ptr to an Image message for storage.
    // sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
    // // Pack the OpenCV image into the ROS image.
    // set_now(msg->header.stamp);
    // msg->header.frame_id = "camera_frame";
    // msg->height = frame_.rows;
    // msg->width = frame_.cols;
    // // msg->encoding = mat_type2encoding(frame_.type());
    // msg->encoding = "rgba8";
    // msg->is_bigendian = false;
    // msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
    // msg->data.assign(frame_.datastart, frame_.dataend);
    // request->img = *msg; 

    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    builtin_interfaces::msg::Time time;
    time.sec = 1;
    header.stamp = time;
    header.frame_id = "camera_frame";
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame_);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    request->img = img_msg; 

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request succeeded");
    }
    else 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
}