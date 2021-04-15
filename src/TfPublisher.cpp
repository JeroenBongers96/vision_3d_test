#include "TfPublisher.h"

TfPublisher::TfPublisher()
{
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

/**
 * Publish TF
 */
void TfPublisher::publish_tf()
{
    // std::cout << "publishing............." << std::endl;
    // geometry_msgs::msg::TransformStamped transform_stamped; 
    // transform_stamped.header.stamp = rclcpp::Time();
    // transform_stamped.header.frame_id = "odom";
    // transform_stamped.child_frame_id = "base_link";
    // transform_stamped.transform.translation.x = 1.0;
    // transform_stamped.transform.translation.y = 1.0;
    // transform_stamped.transform.translation.z = 0.0;
    // transform_stamped.transform.rotation.x = 0.0;
    // transform_stamped.transform.rotation.y = 0.0;
    // transform_stamped.transform.rotation.z = 1.0;
    // transform_stamped.transform.rotation.w = 1.0;
    // tf_broadcaster_->sendTransform(transform_stamped);
}