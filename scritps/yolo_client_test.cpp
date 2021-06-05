#include "rclcpp/rclcpp.hpp"
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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<suii_communication::srv::YoloService>::SharedPtr client =
        node->create_client<suii_communication::srv::YoloService>("yolo_service_msg");

    auto request = std::make_shared<suii_communication::srv::YoloService::Request>();

    std::string img_path = "/home/robohub/ws/src/vision_3d_test/test_img/test.jpeg";
    
    cv::Mat frame_ = cv::imread(img_path);

    std::cout << frame_.type() << std::endl;

    cv_bridge::CvImage cvi;
    cvi.encoding = sensor_msgs::image_encodings::BGR8;
    cvi.image = frame_;

    sensor_msgs::msg::Image::SharedPtr img_msg = cvi.toImageMsg();

    request->img = *img_msg; 

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