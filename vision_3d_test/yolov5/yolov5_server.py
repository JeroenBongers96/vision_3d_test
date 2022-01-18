#!/usr/bin/env python3
...

from suii_communication.srv import YoloService
import sys
sys.path.append("/home/robohub/colcon_ws/src/vision_3d_test/vision_3d_test/yolov5")
import detect

import rclpy
import cv_bridge
import cv2 
from rclpy.node import Node
from cv_bridge import CvBridge

WEIGHTS='/home/robohub/colcon_ws/src/vision_3d_test/weights/best_D435.pt'
SRC='/home/robohub/colcon_ws/src/vision_3d_test/tmp/tmp.png'
CONF=0.7


class Yolov5Service(Node):

    def __init__(self):
        super().__init__('yolov5_server')
        self.srv = self.create_service(YoloService, 'yolo_service_msg', self.yolov5_server_callback)
        self.get_logger().info('Started yolov5 server...')

    def yolov5_server_callback(self, request, response):
        self.get_logger().info('Incoming request successful')
        
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(request.img, desired_encoding='bgr8')

        # Check if there is a faster way to send an image to yolov5
        cv2.imwrite(SRC, cv_image)

        obj_roi_arr = detect.run(weights=WEIGHTS,source=SRC,conf_thres=CONF,nosave=True)

        print("result: {}".format(obj_roi_arr))

        response.obj_roi_arr = obj_roi_arr

        print("sending response ... ")

        return response


def main():
    rclpy.init()

    yolov5_service = Yolov5Service()

    rclpy.spin(yolov5_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()