# 配置流程

## 工具

机载电脑 ubuntu18.04/20.04 

px4固件飞控

飞行平台

机载电脑需配置ROS+mavros+apriltag_ros

mavros安装参考:

配置流程分为源码安装和二进制（apt）安装

主要安装相机驱动和apriiltag推荐二进制安装，简单省时！

## 源码安装

相机安装
`cd catkin_ws/src  #安装在一个ROS工作空间上`
`git clone https://github.com/bosch-ros-pkg/usb_cam.git  #下载源码`
`cd ..  #回到工作空间`
`ctakin_make   #进行编译`
`cd usb_cam`
`mkdir build`
`cd build`
`cmake ..  
make`

测试
`roslaunch usb_cam usb_cam.launch` #启动launch文件

打开rqt，测试相机图像

`rqt_image_view`

apriltag安装

先安装依赖

`git clone https://github.com/AprilRobotics/apriltag.git      # 这个不是ros的功能包`

`cd apriltag`

`mkdir build`
`cd build`
`cmake ..`
`make`
`sudp make install`



`git clone https://github.com/AprilRobotics/apriltag_ros.git`

在工作空间编译

`catkin_make`



## 二进制安装

sudo apt install ros-$ROS_DISTRO-usb-cam

sudo apt install ros-$ROS_DISTRO-apriltag-ros 

在终端用roscd进入apriltag_ros

输入ls查看目录!

可以修改config配置文件和launch文件

# 控制代码讲解

手动模式：手动升高和降落

自动模式：跟随aprilta码自主定位并升高

流程：起飞上升到飞行高度然后进入自动模式，稳定在高度限制
