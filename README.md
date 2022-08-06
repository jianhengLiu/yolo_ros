# YOLO-ROS

**YOLO-ROS for PyTorch**

**YOLO-ROS for TensorRT, please checkout for branch `tensorrt`**

**YOLO-ROS for HUAWEI ATLAS200, please checkout for branch `atlas200`**

Modified from [ros-yolov5](https://github.com/OuyangJunyuan/ros-yolov5), originated from [yolov5](https://github.com/ultralytics/yolov5)

1. clone `YOLO-ROS`
   ```
   cd {YOUR_WORKSPACE}/src
   git clone https://github.com/jianhengLiu/yolo_ros.git
   ```

2. install python dependencies
```
   sudo apt install ros-melodic-ros-numpy
   pip3 install --upgrade pip
   # conda create -n dvins python=3.6
   # conda activate dvins
   pip3 install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu113
   pip3 install -r yolo_ros/requirements.txt
```
3. `catkin_make`

4. `roslaunch yolo_ros yolo_service.launch`

5. play your rosbag.
