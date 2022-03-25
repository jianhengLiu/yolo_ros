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
   pip3 install seaborn matplotlib pandas requests
   ```
   
3. CUDA, TensrotRT etc.: 
   - https://github.com/wang-xinyu/tensorrtx/blob/master/tutorials/install.md
   - https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing-tar

4. generate .wts from pytorch with .pt, or download .wts from model zoo

   ```
   cp Thirdparty/yolov5_tensorrtx/gen_wts.py Thirdparty/yolov5
   mkdir Thirdparty/yolov5_tensorrtx/build
   python3 Thirdparty/yolov5/gen_wts.py -w Thirdparty/yolov5/yolov5n.pt -o Thirdparty/yolov5_tensorrtx/build/yolov5n.wts
   ```

5. build tensorrtx/yolov5 and run

   ```
   cd Thirdparty/yolov5_tensorrtx/build
   cmake ..
   make
   sudo ./yolov5 -s yolov5n.wts yolov5n.engine n
   ```

6. roslaunch

   ```
   cd {YOUR_WORKSPACE}/src
   catkin_make
   source devel/setup.zsh
   roslaunch yolo_ros yolo_tensorrt.launch
   ```



## *Possible Problems on NVIDIA Jetson AGX Xavier*

* JetPack 4.6

### 1. cv_bridge

```
Project 'cv_bridge' specifies '/usr/include/opencv' as an include dir, which is not found.
```

ROS melodic's default installed `cv_bridge 1.13.0` depend on `opencv 3.2`, while the default opencv version in `Jetpack 4.6` is `4.1.1`

`sudo gedit /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake`

change `line 94` and `line 96`
from 
```
if(NOT "include;/usr/include;/usr/include/opencv " STREQUAL " ")
  set(cv_bridge_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/include;/usr/include/opencv")
```
to
```
if(NOT "include;/usr/include;/usr/include/opencv4/opencv2 " STREQUAL " ")
  set(cv_bridge_INCLUDE_DIRS "")
  set(_include_dirs "include;/usr/include;/usr/include/opencv4/opencv2")
```

change `line 119`
from 
```
set(libraries "cv_bridge;/usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0;/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0;/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0")
```
to
```
set(libraries "cv_bridge;/usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1;/usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1;/usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1")
```


or, if you reinstall an other version of opencv, change it to 
```
# find the path by 
# sudo find / -name libopencv_core.so.*
# sudo find / -name libopencv_imgproc.so.*
# sudo find / -name libopencv_imgcodecs.so.*

set(libraries "cv_bridge;/usr/local/lib/libopencv_core.so.4.5.5;/usr/local/lib/libopencv_imgproc.so.4.5.5;/usr/local/lib/libopencv_imgcodecs.so.4.5.5")
```

```
cd {YOUR_WORKSPACE}
rm -rf build devel
catkin_make
```


### 2. Python

**2.1. Illegal instruction(cpre dumped)**
```
echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.zshrc
source ~/.zshrc

# echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
# source ~/.bashrc
```

**2.2. PyTorch**

Download  [PyTorch v1.7.0 pip wheel](https://nvidia.box.com/shared/static/cs3xn3td6sfgtene6jdvsxlr366m2dhq.whl)

```
pip3 install torch-1.7.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install libopenblas-dev
git clone -b v0.8.1 https://github.com/pytorch/vision.git vision-0.8.1
cd vision-0.8.1/
sudo OPENBLAS_CORETYPE=ARMV8 python3 setup.py install
```

[reference](https://blog.csdn.net/qq_40691868/article/details/114379061?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EOPENSEARCH%7Edefault-1.no_search_link&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EOPENSEARCH%7Edefault-1.no_search_link&utm_relevant_index=1)