#!/usr/bin/env python3
...

from suii_communication.srv import AddTwoInts
from suii_communication.srv import YoloService

import rclpy
import cv_bridge
import cv2 
from rclpy.node import Node


from cv_bridge import CvBridge


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(YoloService, 'yolo_service_msg', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request successful')
        
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(request.img, desired_encoding='bgr8')
        
        cv2.imshow("test", cv_image)
        cv2.waitKey(0)

        response.succes = True 

        print("sending response ... ")

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()