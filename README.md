# vision_3d_test
Repo to develop small vision tests quickly

## How to use
* Run server: `ros2 run vision_3d_test vision_server`
* Call service: `ros2 service call /vision_scan suii_communication/VisionScan "{debug: True, create_data: False, save_data: False}"`

##YOLO
# Torch version: 1.6.0
# Torchvision version: 0.7.0
# Newer versions don't work with YOLOV3
