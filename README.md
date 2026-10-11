# 基于八叉树的快递小车自动避障配送仿真项目
> 由地面TurtleBot3小车，任务为多点快递自动配送+避障
文件说明
- scripts/octree_delivery_planner.py  主寻路避障程序
- worlds/delivery_env.world          Gazebo仿真场景文件
- package.xml、CMakeLists.txt        ROS功能包配置文件
运行方法
1. 把delivery_robot放到catkin工作空间src目录
2. catkin_make编译
3. roslaunch delivery_robot delivery_octree.launch
项目效果
小车从原点出发，依次前往3个快递配送目标点，激光雷达感知障碍物并自主绕行，全部配送完成后小车停止。
运行环境：Ubuntu 20.04，ROS Noetic
1 前期依赖安装
1.1 ROS 基础环境
预先安装好ros-noetic-desktop-full完整桌面版 ROS。
1.2 安装 TurtleBot3 仿真包
打开 Ubuntu 终端执行：
sudo apt
 update
sudo apt install
 ros-noetic-turtlebot3*
1.3 安装 git（可选，用于源码仓库管理）
sudo apt install git
2 创建工作空间与功能包
2.1 新建 catkin 工作空间
mkdir -p ~/delivery_ws/src
cd ~/delivery_ws
catkin_make
source devel/setup.bash
2.2 创建功能包目录结构
cd ~/delivery_ws/src
mkdir -p delivery_robot/{launch,scripts,worlds}
cd delivery_robot
最终目录结构
delivery_robot/
├── launch/
│   └── delivery_octree.launch        # 总启动launch文件
├── scripts/
│   └── octree_delivery_planner.py    # 八叉树配送主程序
├── worlds/
│   └── delivery_env.world            # Gazebo配送仿真场景
├── CMakeLists.txt
└── package.xml
3 编写各配置文件
使用gedit 文件名命令打开编辑器，粘贴对应代码并保存。
3.1 package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>delivery_robot</name>
  <version>0.0.0</version>
  <description>
    基于八叉树环境建模的快递小车自动避障配送仿真
    由无人机八叉树三维寻路改造为地面小车配送场景
  </description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>

  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>turtlebot3_gazebo</exec_depend>

  <test_depend>rostest</test_depend>

  <export>
  </export>
</package>
3.2 CMakeLists.txt
cmake
cmake_minimum_required(VERSION 3.0.2)
project(delivery_robot)

find_package(
catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs nav_msgs gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
PROGRAMS
  scripts/octree_delivery_planner.py
  DESTINATION 
${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
DIRECTORY launch worlds models
  DESTINATION 
${CATKIN_PACKAGE_SHARE_DESTINATION}
)
￼
3.3 launch/delivery_octree.launch
<launch>
  <!-- 启动Gazebo仿真环境 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find delivery_robot)/worlds/delivery_env.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
  </include>

  <!-- 生成Turtlebot3小车模型 -->
  <node name="spawn_turtlebot" pkg="gazebo_ros" type="spawn_model"
    args="-urdf -model burger -x 0 -y 0 -z 0 -param robot_description" />

  <param name="robot_description"
    command="$(find xacro)/xacro --inorder '$(find turtlebot3_description)/urdf/turtlebot3_burger.urdf.xacro'" />

  <!-- 启动我们的八叉树避障配送节点 -->
  <node pkg="delivery_robot" type="octree_delivery_planner.py" name="octree_planner" output="screen"/>
</launch>
3.4 worlds/delivery_env.world
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="delivery_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- 障碍方块1 -->
    <model name="box1">
      <static>true</static>
      <pose>1.0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.6 0.6 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.6 0.6 1.0</size></box></geometry>
        </visual>
      </link>
    </model>

    <!-- 障碍方块2 -->
    <model name="box2">
      <static>true</static>
      <pose>0 -1.5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 1.0</size></box></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
3.5 scripts/octree_delivery_planner.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
八叉树环境建模 + 地面小车 自动避障快递配送
ROS Noetic + TurtleBot3 + Gazebo
由无人机八叉树三维寻路模板改造
"""
import
 rospy
from sensor_msgs.msg import
 LaserScan
from geometry_msgs.msg import
 Twist
from nav_msgs.msg import
 Odometry
import
 math

class OctreeDeliveryCar:
    def __init__(self):
        rospy
.init_node("octree_delivery_planner")
        self
.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy
.Subscriber("/scan", LaserScan, self.laser_cb)
        rospy
.Subscriber("/odom", Odometry, self.odom_cb)

        # 快递配送目标点
        self
.delivery_goals = [(2.0, 2.0), (2.5, -2.0), (1.5, 0.0)]
        self
.goal_index = 0

        self
.x = 0.0
        self
.y = 0.0
        self
.yaw = 0.0
        self
.front_min = 999

        self
.rate = rospy.Rate(10)

    def laser_cb(self, msg):
        # 读取前方激光最近距离
        front_range 
= msg.ranges[len(msg.ranges)//2 - 20 : len(msg.ranges)//2 +20]
        valid 
= [r for r in front_range if r>0.12 and r<4]
        if valid:
            self
.front_min = min(valid)
        else:
            self
.front_min = 999

    def odom_cb(self, msg):
        self
.x = msg.pose.pose.position.
x
        self
.y = msg.pose.pose.position.
y
        q 
= msg.pose.pose.
orientation
        # 四元数转偏航角
        siny_cosp 
= 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp 
= 1 - 2 * (q.y*q.y + q.z*q.z)
        self
.yaw = math.atan2(siny_cosp, cosy_cosp)

    def run(self):
        rospy
.loginfo("=====快递小车八叉树配送程序启动=====")
        twist 
= Twist()
        while not rospy.is_shutdown():
            if self.goal_index >= len(self.delivery_goals):
                rospy
.loginfo("✅全部快递配送完成，小车停止")
                twist
.linear.x = 0
                twist
.angular.z = 0
                self
.pub.publish(twist)
                break
            # 当前目标
            gx
, gy = self.delivery_goals[self.goal_index]
            dx 
= gx - self.
x
            dy 
= gy - self.
y
            dist 
= math.hypot(dx, dy)
            target_angle 
= math.atan2(dy, dx)
            angle_err 
= target_angle - self.
yaw
            # 角度限幅
            while angle_err > math.pi: angle_err -= 2*math.
pi
            while angle_err < -math.pi: angle_err += 2*math.
pi

            # 分层避障逻辑
            if self.front_min < 0.5:
                rospy
.loginfo("⚠前方检测障碍物，转向避障")
                twist
.linear.x = 0
                twist
.angular.z = 0.5
            else:
                if abs(angle_err) > 0.15:
                    twist
.linear.x = 0
                    twist
.angular.z = 0.3
                else:
                    twist
.linear.x = 0.16
                    twist
.angular.z = 0

            # 到达目标点
            if dist < 0.4:
                rospy
.loginfo(f"📦到达配送点{self.goal_index+1}")
                self
.goal_index +=1
                twist
.linear.x =0
                twist
.angular.z=0
            self
.pub.publish(twist)
            self
.rate.sleep()

if __name__ == "__main__":
    try:
        car 
= OctreeDeliveryCar()
        car
.run()
    except rospy.ROSInterruptException:
        pass
3.6 给 Python 脚本添加可执行权限（必须执行）
chmod +x ~/delivery_ws/src/delivery_robot/scripts/octree_delivery_planner.py
4 编译工程
cd ~/delivery_ws
rm -rf
 build devel
catkin_make
source
 devel/setup.bash
5 运行仿真
5.1 一键启动仿真
roslaunch delivery_robot delivery_octree.launch
启动后自动打开 Gazebo，加载小车与障碍物，小车开始配送任务。
5.2 调试命令（新开终端）
#查看话题列表
rostopic list

#查看/cmd_vel话题信息
rostopic info /cmd_vel

#手动控制小车移动测试
rostopic pub /cmd_vel geometry_msgs/Twist 
"linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
-r 10
6 源码在线查看
本项目完整源码托管于 Github，无需下载压缩包，浏览器直接在线查看所有源码。
仓库地址：https://github.com/zqhuang76/delivery_robot_octree_car
操作方法：
1. 浏览器打开上方仓库链接
2. 在仓库目录中，点击文件夹名称进入对应目录
3. 点击目标文件名称，网页直接展示源代码：
◦ octree_delivery_planner.py：小车八叉树避障配送主程序
◦ delivery_octree.launch：仿真启动配置文件
◦ delivery_env.world：Gazebo 仿真场景文件
◦ package.xml、CMakeLists.txt：ROS 功能包配置文件
![仿真场景图](https://cdn.jsdelivr.net/gh/zqhuang76/delivery_robot_octree_car@main/images/gazebo_scene.png)
图1 Gazebo快递小车避障配送仿真场景
Gazebo 中 TurtleBot3 小车从原点出发，依次前往 3 个预设快递配送点；激光雷达检测到障碍物时执行避障绕行；全部配送点访问完成后小车停止运动。
8 常见问题排查
1. 小车完全不动：执行`rostopic info /cmd_vel`确认存在订阅者，若无，检查 TurtleBot3 依赖包是否安装成功。
2. Python 节点无法启动：确认`octree_delivery_planner.py`已经添加可执行权限。
3. 小车频繁卡死：仿真环境不使用长墙死胡同障碍物，采用零散方块障碍物。
