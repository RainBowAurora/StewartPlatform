# StewartPlatform
[![release](https://img.shields.io/github/release/RainBowAurora/StewartPlatform.svg)](https://github.com/RainBowAurora/StewartPlatform/releases)
[![bilibili](https://img.shields.io/badge/bilibili-video-red)](https://www.bilibili.com/video/BV1Gp4y1a7Tx)
<a href="https://github.com/RainBowAurora/StewartPlatform/actions"><img alt="GitHub Actions status" src="https://github.com/RainBowAurora/StewartPlatform/actions/workflows/kinetic_check.yml/badge.svg"></a>


1. **硬件: 树莓派3b/3b+**
2. **操作系统**: Ubuntu16.04
3. **ROS版本**: Kinetic

## 项目简介
六自由度平台(StewartPlatform)是1965年由德国结构工程师Stewart发明并研制的，它是由一个动平台(上平台)和一个静平台(下平台),以及６个"可伸缩"的杆件组成,六自由度平台是用于飞行器丶运动器(如飞机丶车辆)模拟训练的动感模拟装置，是一种并联运动机构，他通过改变六个可以伸缩的杆件的长度实现空间上六个自由度的运动(垂直丶横向丶纵向丶俯仰、滚转、偏行);即X,y,z方向上的平移以及绕x,y,z三个轴的旋转运动，通过这六个自由度复合实现复杂动作实现．

## 特点
1. 结构稳定，刚度大，承载能力强
2. 减小运动负荷，动力性能较好:
3. 机机构误差趋向平均化，因此误差小精度高

## 硬件列表
1. 树莓派(3B/3B+)
2. i2c扩展板(Servo Driver HAT)
3. 开关电源(5V-3A)
4. 大扭矩金属舵机(MG995/MG996R/SG90)
5. 24mm舵机舵机臂]
6. 鱼眼连杆
7. 亚克力切割板材

## 使用方法
1. [安装ubuntu 16.04系统](https://jingyan.baidu.com/article/3c48dd348bc005e10be358eb.html)
2. [安装ros-kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
3. 下载源码
```
$ mkdir -p catkin_ws/src
$ cd ~/catkin_ws
$ catkin_init_workspace
$ cd ~/catkin_ws/src
$ git clone https://github.com/RainBowAurora/StewartPlatform
$ cd ~/catkin_ws
$ catkin_make -j1
```
4. 运行
```
roslaunch stewart_platform bringup.launch
```

## 维护
如果有任何问题请联系: 851045076@qq.com

