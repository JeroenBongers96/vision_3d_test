#!/usr/bin/env python3
from network import NetworkManager
from yolo import Yolo
import numpy
import base64
import cv2

class YoloServer(object):
    def __init__(self):
        self.net = NetworkManager(9999)
        self.yolo = Yolo()
        self.yolo.load_model()
        self.net.addCallback(0x00, self.yolo_callback)
        while True:
            try:
                self.net.spinSocket()
            except KeyboardInterrupt:
                self.net.close()

    def yolo_callback(self, arg):
        byte_img = bytes(arg['img'], 'ascii')
        img_decoded = base64.b64decode(byte_img)
        img = numpy.frombuffer(img_decoded, dtype=numpy.uint8)
        cv_img = cv2.imdecode(img, flags=1)
        names = self.yolo.run(numpy.asarray(cv_img), False)
        return (0x00, {'names': names})


if __name__ == "__main__":
    server = YoloServer()


