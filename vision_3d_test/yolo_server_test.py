#!/usr/bin/env python3
...

from suii_communication.srv import YoloService
from vision_3d_test.yolo.yolo import Yolo

import numpy
import rclpy
import cv_bridge
import cv2 
from rclpy.node import Node

from cv_bridge import CvBridge

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(YoloService, 'yolo_service_msg', self.add_two_ints_callback)
        self.yolo = Yolo()
        self.yolo.load_model()

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request successful')
        
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(request.img, desired_encoding='bgr8')

        #Run yolo
        result = self.yolo.run(cv_image, False)
        print("result: ", result)
        
        obj_roi_arr = []

        #Pick all results out off the result array and put them in a new array
        for x in result:
            #add start int
            for y in x:
                #add data
                obj_roi_arr.append(y) 

        print("obj_roi_arr: ", obj_roi_arr)

        #Send response back
        response.obj_roi_arr = obj_roi_arr

        print("sending response ... ")

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()