#!/usr/bin/env python3
...

# from example_interfaces.srv import AddTwoInts
#from suii_communication.srv import AddTwoInts
from suii_communication.srv import YoloService

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('yolo_server')
        self.srv = self.create_service(YoloService, 'yolo_server_service', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # response.sum = request.a + request.b

        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Incoming request received :)')

        response = True
        
        return response

def main():
    print("YOLO server started ....")

    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()