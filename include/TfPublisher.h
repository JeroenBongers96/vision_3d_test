#ifndef TFPUBLISHER__H
#define TFPUBLISHER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfPublisher
{
    private:
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    public:
        TfPublisher();
        void publish_tf();
};

#endif