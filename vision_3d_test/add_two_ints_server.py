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
        # response.sum = request.a + request.b
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Incoming request successful')
        
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(request.img, desired_encoding='passthrough')
        # cv_image = bridge.imgmsg_to_cv2(request.img, desired_encoding='bgr8')

        # cv_image = cv_bridge.imgmsg_to_cv2(request.img, "bgr8")

        cv2.imshow("test img", cv_image)

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