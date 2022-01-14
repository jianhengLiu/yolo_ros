# YOLO-ROS

**YOLO-ROS for TensorRT**

**YOLO-ROS for Pytorch, please checkout for branch `master`**

**YOLO-ROS for HUAWEI ATLAS200, please checkout for branch `atlas200`**

Modified from [tensorrtx](https://github.com/wang-xinyu/tensorrtx)

1. clone `YOLO-ROS`
   ```
   cd {YOUR_WORKSPACE}/src
   git clone https://github.com/jianhengLiu/yolo_ros.git
   cd yolo_ros
   git checkout tensorrt
   ```

2. install python dependencies
   ```
   # install python dependencies
   sudo apt install ros-melodic-ros-numpy
   sudo apt install python3-pip
   pip3 install --upgrade pip
   pip3 install rospkg
   pip3 install Cython matplotlib==3.2.2 numpy==1.18.5 Pillow PyYAML==5.4.1 scipy==1.5.4 tensorboard==1.15.0 tqdm==4.58.0 seaborn==0.11.1 pandas thop pycocotools==2.0.2
   # if you are running on xavier, please not directly install torch by pip and refer to `Possible Problems on NVIDIA Jetson AGX Xavier` section
   pip3 install torch==1.7.0 torchvision==0.8.1
   ```
3. generate .wts from pytorch with .pt, or download .wts from model zoo

   ```
   cp Thirdparty/yolov5_tensorrtx/gen_wts.py Thirdparty/yolov5
   mkdir Thirdparty/yolov5_tensorrtx/build
   python3 Thirdparty/yolov5/gen_wts.py -w Thirdparty/yolov5/yolov5n.pt -o Thirdparty/yolov5_tensorrtx/build/yolov5n.wts
   ```

4. build tensorrtx/yolov5 and run

   ```
   cd Thirdparty/yolov5_tensorrtx/build
   cmake ..
   make
   sudo ./yolov5 -s yolov5n.wts yolov5n.engine n
   ```

5. roslaunch

   ```
   cd {YOUR_WORKSPACE}/src
   catkin_make
   source devel/setup.zsh
   roslaunch yolo_ros yolo_tensorrt.launch
   ```


### Possible Problems on NVIDIA Jetson AGX Xavier

* JetPack 4.6


**1. Python**

**1.2. Illegal instruction(cpre dumped)**
```
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.zshrc
source ~/.zshrc

# echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
# source ~/.bashrc
```

**1.3. PyTorch**

Download  [PyTorch v1.7.0 pip wheel](https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl)

```
pip3 install torch-1.7.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install libopenblas-dev
git clone -b v0.8.1 https://github.com/pytorch/vision.git vision-0.8.1
cd vision-0.8.1/
sudo OPENBLAS_CORETYPE=ARMV8 python3 setup.py install
```

[reference](https://blog.csdn.net/qq_40691868/article/details/114379061?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EOPENSEARCH%7Edefault-1.no_search_link&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EOPENSEARCH%7Edefault-1.no_search_link&utm_relevant_index=1)
