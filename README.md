# YOLO-ROS

**YOLO-ROS for HUAWEI ATLAS200**

**YOLO-ROS for PyTorch, please checkout for branch `master`**
**YOLO-ROS for TensorRT, please checkout for branch `tensorrt`**

1. clone `YOLO-ROS`
   ```
   cd {YOUR_WORKSPACE}/src
   git clone https://github.com/jianhengLiu/yolo_ros.git
   git checkout atlas200
   ```

2. `catkin_make`

3. Download pre-trained model and put it in `yolo_ros/model`
   * 百度网盘： https://pan.baidu.com/s/1m0lapSFk8KG5Z1Jo5T2VFQ  密码: wgs5

4. `roslaunch yolo_ros yolo_atlas_nodelet.launch`

5. play your rosbag.
