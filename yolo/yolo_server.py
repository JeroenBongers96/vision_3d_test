#!/usr/bin/env python
import cv2
import time
import numpy
#from processing import PostProcessing
from suii_3d_vision_ros.srv import GetRoi, GetRoiResponse
from std_msgs.msg import String
from sensor_msgs.msg import Image
from network import NetworkClient
#from cv_bridge import CvBridge, CvBridgeError
from cv_bridge import CvBridge, CvBridgeError
import rospy
import roslib
#from yolo import Yolo
import struct
import socket
import base64
import json

class Yolo_Wrapper(object):
    def __init__(self):
        rospy.init_node('get_roi_server')
        s = rospy.Service('get_roi', GetRoi, self.handle_get_roi)
        print("Server is ready")
        self.client = NetworkClient("localhost", 9999)
        #data = numpy.load('/home/jeroen/catkin_ws/src/suii_3d_vision_ros/yolo/config/mtx.npz')
        #self.mtx = data['mtx']
        #data = numpy.load('/home/jeroen/catkin_ws/src/suii_3d_vision_ros/yolo/config/dist.npz')
        #self.dist = data['dist'] 
        rospy.spin()
    
    def handle_get_roi(self, req):
        cvb_de = CvBridge()
        #cv_image = bridge.imgmsg_to_cv2(req, desired_encoding="bgr8")
        newimg = cvb_de.imgmsg_to_cv2(req.input, "bgr8")

        #image = newimg
        #h,  w = image.shape[:2]
        #newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(w,h),1,(w,h))
        #dst = cv2.undistort(image, self.mtx, self.dist, None, newcameramtx)
        #x,y,w,h = roi
        #image = dst[y:y+h, x:x+w]
        
        retval, buff = cv2.imencode('.jpg', newimg)
        jpg_enc = base64.b64encode(buff)

        # Do no touch, client encoded request
        resp = self.client.networkCall(0x00, {"img": jpg_enc})
        #End client encoded request

        list_of_name = resp['names']
        arr_list = []
        for x in list_of_name:
            #add start int
            for y in x:
                #add data
                arr_list.append(y)                

        # convert to np array
        return GetRoiResponse(arr_list)

if __name__ == "__main__":
    wrapper = Yolo_Wrapper()

