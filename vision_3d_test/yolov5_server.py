#!/usr/bin/env python3
...

from suii_communication.srv import YoloService
# from vision_3d_test import yolo

import rclpy
import cv_bridge
import cv2 
from rclpy.node import Node
from cv_bridge import CvBridge

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(YoloService, 'yolo_service_msg', self.add_two_ints_callback)
        self.get_logger().info('Started yolov5 server test ...')

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request successful')
        
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(request.img, desired_encoding='bgr8')
        
        # cv2.imshow(cv_image)
        # cv2.waitKey(0)

        obj_roi_arr = []

        for x in range(4):
            obj_roi_arr.append(x) # add id
            obj_roi_arr.append(x*10) # add top left X coordinate
            obj_roi_arr.append(x*10) # add top left X coordinate
            obj_roi_arr.append(x*10) # add top left X coordinate
            obj_roi_arr.append(x*10) # add top left X coordinate

        print(len(obj_roi_arr))
        print("obj_roi_arr: ", obj_roi_arr)

        response.obj_roi_arr = obj_roi_arr

        print("sending response ... ")

        # print(type(response.obj_roi_arr))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()