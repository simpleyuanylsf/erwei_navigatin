# mid360二维自主避障并将cmd_vel话题转换到mavros进行控制

加了gmapping导航效果不好，所以最后取消使用 只使用了全局代价地图和局部代价地图 导航效果待更新

## 环境条件

要先安装gmapping和navigation 还有explore_lite 
mid360的包和livox_ros_driver自行安装

```
# 更新软件包索引
sudo apt update
# 安装gmapping（Noetic版本）
sudo apt install ros-noetic-gmapping
# 安装navigation包
sudo apt install ros-noetic-navigation
# 安装explore_lite（自主探索核心包）
sudo apt install ros-noetic-explore-lite
```

## lio_to_mavros

坐标转换部分 可以看我[simpleyuanylsf/lio_to_mavros: fast-lio的odom给到px4的vision/pose](https://github.com/simpleyuanylsf/lio_to_mavros)

## my_nav_demo

话题转换部分 将输出的速度给到mavros

### control.launch

cmd_vel话题转换到mavros进行控制部分的launch文件

###  explore.launch

启动explore_lite的包 订阅地图话题 自动打出目标点 完成自主探索 未使用

### gmapping_run.launch

运行gmapping 但是我最后并没有用到

### move_base.launch

这个将local_planner和global_planner启动起来

### start.launch

这个是将lio_to_mavros pointcloud_to_laserscan move_base启动起来

## pointcloud_to_laserscan-1.4.1

三维点云转二维点云 方便导航



```
roslaunch my_drone_nav 对应的.launch
```

## 反馈

若有问题 发邮件到256395180@qq.com
