用于记录代码书写过程中的必要指令。
# 工程文件夹创建
## 源文件夹
mkdir -p ~/catkin_ws/src
## 初始化文件夹
cd ~/catkin_ws/  catkin_make，这个make指令以后都是在这个根目录执行
如果有clangd，则catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=YES
## 环境变量
source /opt/ros/noetic/setup.bash
source devel/setup.bash
建议写入bashrc
## 如果cmake文件有写include规则时
catkin_make install，在根文件目录下，一般也可以直接运行这个。
## 创建功能包
catkin_create_pkg <包名> [依赖项1] [依赖项2] [依赖项n]
后续要新增依赖通过修改package.xml中添加```<build_depend>新依赖</build_depend> 和 <exec_depend>新依赖</exec_depend>。```和在CMakeLists.txt中find_package(catkin REQUIRED COMPONENTS ...)
一般新建功能包会使用的依赖为：
roscpp: 如果你要写 C++。

rospy: 如果你要写 Python。

std_msgs: 几乎必带，包含常用的字符串、整型等基础消息。

sensor_msgs: 如果你要处理雷达、摄像头数据。

geometry_msgs: 如果你要处理位姿、速度（Twist）等。

message_generation: 如果要自定义消息，这个必带。

## 编译单个包

这个其实蛮重要的，特别是工程越来越大的时候：catkin_make --only-pkg-with-deps [包名]

## 消息和服务
1. 在包内 mkdir msg/srv
2. 写相应文件vim MyData.msg，并修改cmake文件和package.xml，由于相应规则有点多，单独写一个说明文档在docs

# 其他辅助命令
1. rqt_graph;rqt_console
2. rostopic echo /topic_name；rostopic hz /topic_name；rostopic pub /topic_name [类型] [数据]
3. rostopic list；rosnode info /node_name；rosnode cleanup
4. rosparam list;rosparam get /param_name;rosparam set /param_name [值]；rosparam dump [文件名.yaml]；rosparam load [文件名.yaml]
5. rviz
6. rosrun tf tf_echo [父坐标系] [子坐标系];rosrun tf view_frames && evince frames.pdf
7. rosservice list；rosservice info /服务名；rosservice call /服务名 [数据]；rosservice type /服务名
8. rosmsg show geometry_msgs/PoseStamped；rossrv show mavros_msgs/SetMode
9. rosbag record -a；rosbag record /mavros/local_position/pose /vins/odometry；rosbag play [文件名.bag]；rosbag info [文件名.bag]

# 代码启动
## 命令行方法
roscore
rosrun [包名] [节点名]
## launch方法
roslaunch [包名] [文件名.launch]
```
<launch>
    <!-- 1. 启动节点 -->
    <!-- pkg: 包名,create_pkg命令生成的 type: 可执行文件名，cmake生成的那个目标, name: 启动后在系统里的别名 ，name的存在使得可以同时执行多次可执行文件。-->
    <node pkg="mavros" type="mavros_node" name="mavros" output="screen">
        <!-- 2. 加载私有参数 -->
        <param name="fcu_url" value="/dev/ttyUSB0:921600" />
        <!-- 语法：from="代码里写的名字" to="实际运行的名字" -->
    <remap from="current_pose" to="/vins_estimator/odometry" />
    </node>

    <!-- 3. 命名空间组：代码编写中的话题名字路径往往采用相对路径，如果没有命名则默认在/下面，ros::NodeHandle nh;使用这个默认套上group路径，使用ros::NodeHandle nh_p("~");则会再套上节点名字防止组内名字重复。 -->
    <group ns="uav_1">
        <node pkg="my_pkg" type="my_control_node" name="controller" />
    </group>

    <!-- 4. 包含另一个 Launch 文件：嵌套使用 -->
    <include file="$(find ego_planner)/launch/vins_config.launch" />
</launch>
```

# 代码编写
## 创建节点
#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "mytest_node");
  ros::NodeHandle nh;
  return 0;
}

## 创建或使用话题
不过是订阅还是发布如果没有这个话题都会创建。且默认使用相对路径，如果launch时不加组则默认根路径。
```
ros::Publisher pub = nh.advertise<Message_Type>("topic_name", queue_size, [latch]);
ros::Subscriber sub = nh.subscribe<Message_Type>("topic_name", queue_size, callback_function);

```
显然要现有消息数据。这里官方提供了一些模板消息。
std_msgs：基础类型（int, float, string, bool）。

geometry_msgs：几何坐标（Point, Vector3, PoseStamped（无人机最常用）, Twist）。

sensor_msgs：传感器数据（LaserScan, Image, Imu）。

mavros_msgs：专门给 MAVROS 用的控制和状态消息。
## 自定义消息
在包下msg/MyData.msg，会自动把文件夹当成命名空间。
键入：
```
float64 frequency
float64 phase_diff
int32 signal_level
string status
```
然后：
package.xml：添加 message_generation（构建时）和 message_runtime（运行时）。

CMakeLists.txt：

find_package 里加入 message_generation。

add_message_files(FILES MyData.msg)。

generate_messages(DEPENDENCIES std_msgs)。
## 消息的使用
把他当成一个类，可以.访问。也可以直接pub.publish(msg);
## 服务自定义
package不需要重新写，依赖还是那个，这里历史遗留问题，有些人喜欢msg和srv分开，有些则是混在一起。
你只需要在cmake里补上add_service_files(FILES YourService.srv)。即可
```
# 请求部分（Request）
uint16 command
float32 param1
float32 param2
---
# 响应部分（Response）
bool success
uint8 result
```
会生成srv_name::Request和srv_name::Response。
## 服务与客户函数
他与订阅发布这种多多对不一样他是一对多的。只能有一个服务端。但是实际上为一对一，多个client呼叫时也是一个一个来处理的。
ros::ServiceServer service = nh.advertiseService("name", callback);
callback需要传入srv的request和Response的引用。，request是发来的你要在函数里处理，然后发回Response。回去。但是函数返回值是bool返回是否处理完毕即可。因为传入的是引用实际上已经修改了
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
## 辅助函数
1. ros::ok()，判断当前节点是否存活。
2. ros::Rate rate(20.0);创建20HZ的定时器对象，rate.sleep();
3. ros::shutdown()
4. ros::spinOnce();订阅话题，如果话题有数据并不会立刻调用回调函数而是放入队列，执行该语句才开始处理。会全执行完再继续走。没有数据则不执行。
5. ros::spin()，死等回调。阻塞，数据太多会丢掉老数据。第四点和第五点都不能解决高实时性要求，ros1的解决方法是ros::AsyncSpinner多线程以及专属等待队列ros::CallbackQueue。ros2的Executor
6. ros::Timer timer = nh.createTimer(ros::Duration(0.033), &EgoPlanner::checkCollisions, this);

# 阅读技巧
## ROS_PACKAGE_PATH
ros相关的查找方法很多都是依赖这个环境变量。
echo $ROS_PACKAGE_PATH
## 寻找指令
rospack find mavros && roscd mavros,如果你使用vscode，可以使用code filename在窗口打开
有些特殊的功能包不由catkin_make编译因此找不到，而px4采用了强行塞入package方法变成功能包，但是roscd是找不到的。如果只是要找launch文件可以 find / -name "posix_sitl.launch" 2>/dev/null。
其实你还可以看到环境变量里export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PX4_SRC_DIR}
# 模型教程
https://jszn.csdn.net/6a053f4354b52172bc740663.html
# 端口占用
## windows
netstat -ano | findstr :端口号
## linux
1. sudo lsof -i :7890
2. ss -tunlp
## 常见端口
1. 14550 udp qgc监听qcs发送数据
2. 4560和14580 tcp gazebo使用
## 虚拟机网络
1. NAT与Bridge，虚拟机与宿主机的localhost不同。mirrored则相同。
2. ifconfig 如果看得到主机网卡就是mirrored。
3. cat /etc/resolv.conf 获取在wsl眼中的主机地址。

1. NAT：wsl通过cat /etc/resolv.conf这里面的地址和主机发送数据。windows访问wsl则需要使用他的内网ip。
2. Bridge:像正常的局域网通信。且处于同一网段。
3. mirrored当成一个电脑用。
4. 前两种需要考虑防火墙，请注意检查。