# bezier_local_planner

## 1. 安装依赖
我们假设您已经创建了名为catkin_ws的工作空间并完成了初始化。如果您的ROS版本是melodic，可直接用二进制安装navigation包
```
sudo apt install ros-melodic-navigation
```
如果您的ROS版本是kinetic，请先从GitHub下载navigation源码到本地，并切换到melodic分支，再编译
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ros-planning/navigation.git
$ git checkout melodic-devel
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

完成上述准备工作之后，再将该仓库克隆到本地
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-MobFly-Robotics/dwa_planner.git
```

完成BezierLocalPlanner类之后，编译程序
```
$ cd ~/catkin_ws/
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 2. 使用benchmark
首先，请下载benchmark源码到本地并编译
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/NKU-MobFly-Robotics/local-planning-benchmark.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

然后打开move_base_benchmark.launch文件，找到
```
<param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS" />
```
将其替换为
```
<param name="base_local_planner" value="bezier_local_planner/BezierLocalPlannerROS" />
<rosparam file="$(find bezier_local_planner)/params/bezier_planner_params.yaml" command="load" />
```
接下来打开一个新的终端，运行仿真
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch move_base_benchmark move_base_benchmark.launch
```
在弹出的RViz可视化界面里，点击工具栏的2D Nav Goal并在地图中设置终点，即可运行导航程序