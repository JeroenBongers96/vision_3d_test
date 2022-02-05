# vision_3d_test
Repo to develop small vision tests quickly

## Prerequirements
Clone and build:
`https://github.com/JeroenBongers96/suii_communication.git` 

## How to use
* `cd colcon_ws`
* `colcon build`
* `. install/setup.bash`
* Run server: `ros2 run vision_3d_test vision_server`
* Run yolov5 server: `ros2 run vision_3d_test yolov5_server.py`
* Call service: `ros2 service call /vision_scan suii_communication/VisionScan "{debug: False, create_data: False, save_data: False}"`