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
4. rosparam list;rosparam get /param_name;rosparam set /param_name [值]
5. rviz
6. rosrun tf tf_echo [父坐标系] [子坐标系];rosrun tf view_frames && evince frames.pdf
7. top

# 代码编写
## 创建节点
#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "mytest_node");
  ros::NodeHandle nh;
  return 0;
}
