#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {

        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

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

    std::string mat_type2encoding(int mat_type)
    {
        switch (mat_type) 
        {
            case CV_8UC1:
            return "mono8";
            case CV_8UC3:
            return "bgr8";
            case CV_16SC1:
            return "mono16";
            case CV_8UC4:
            return "rgba8";
            default:
            throw std::runtime_error("Unsupported encoding type");
        }
    }

    void timer_callback()
    {
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("img", rclcpp::SensorDataQoS());

        // Create a new unique_ptr to an Image message for storage.
        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());

        std::string img_path = "/home/robohub/ws/src/vision_3d_test/test_img/test.jpeg";
        // string test_img = "/home/jeroen/cv_img.png";
        cv::Mat frame_ = cv::imread(img_path);
        // Pack the OpenCV image into the ROS image.
        set_now(msg->header.stamp);
        msg->header.frame_id = "camera_frame";
        msg->height = frame_.rows;
        msg->width = frame_.cols;
        msg->encoding = mat_type2encoding(frame_.type());
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
        msg->data.assign(frame_.datastart, frame_.dataend);
        pub_->publish(std::move(msg));  // Publish.
    }
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}