# FOR UBUNTU 20.04 FOXY

## 1. 准备工作
1. apriltag_ros foxy 版本 tag 3.0.0 https://github.com/christianrauch/apriltag_ros.git
2. opencv 以及 opencv_contrib 4.6.0 
3. apriltag 3.3.0 or 3.2.0（与apriltag_ws同级文件）
4. image_pipline  (内的image_proc与camera_calibration)
5. apriltag_viz 
- 首先需要对使用的相机进行内参标定，参考链接：
https://docs.ros.org/en/rolling/p/camera_calibration/doc/tutorial_mono.html
得到的内参信息首先需要被相机驱动读取（与/camera_info内参数类型一致）。
后面再实现tag标签坐标系与camera坐标系之间的tf变换时也有用到。
## 2. 注意事项
- 注意各个软件包之间的版本对应。
- 指定版本编译：
``` Bash
 colcon build --cmake-args   -DCMAKE_MINIMUM_REQUIRED_VERSION=3.5   -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```
- 注意标签的类型（用例中的是36h11），不同类型标签使用的参数不同。
- opencv配置编译：
```Bash
cd opencv-4.7.0
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib-4.7.0/modules \
      -D BUILD_EXAMPLES=OFF \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON ..
  make install
```

## 3. 启动流程
1.  启动cameracalibrator进行标定
```Bash
ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.024 --ros-args --remap image:=/image_raw --remap camera:=/camera_info
```
3. 启动usb_cam
```Bash
ros2 launch usb_cam demo_launch.py
```
这里若需要更换摄像头设备，需要去/opt/ros/foxy/share/usb_cam/config/params.yaml中修改设备号。
5. 启动image_proc
```Bash
ros2 launch image_proc image_proc.launch.py
```
6. 启动apriltag_ros 节点
```Bash
ros2 run apriltag_ros apriltag_node --ros-args \--params-file  /home/sc/Downloads/tags_36h11.yaml
```
8. 启动apriltag_viz 实现坐标系间的转换
```Bash
ros2 launch apriltag_viz viz.launch.py
```
RQT_graph
![image](https://github.com/user-attachments/assets/71afe8e2-7812-4505-9b11-a2e81219a16d)
