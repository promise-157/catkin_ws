这里将会介绍rviz的一些知识
# 坐标系与TF
## TF坐标树
1. 记录坐标系之间的关系，父坐标系可以有多个子坐标系，子坐标系只能有一个。但是可以发现任何一个坐标系基本都能用其他坐标系表示。因此坐标系怎么连接的成为问题。
2. 常见标准为：REP 105 标准：详述 ROS 官方规定的标准坐标系拓扑结构（map -> odom -> base_link -> sensor_link）。
3. 坐标系之间的表示关系其实就是一个矩阵，传递好这个矩阵就能知道怎么换算。如果你算法使用了跳级的坐标映射关系请转换为单级的映射。
4. 解释一下odom坐标，他的原点是机器人的起点，通常map的原点也是以机器人为起点。咋一看好像一样，但是map坐标代表实际的坐标，gazebo里一般直接给出。现实中则是slam建图后的坐标。而odom不管是现实还是仿真都是速度积分出来的，他可能和map的坐标有误差。
因此不仅是map的起点选取导致和odom与T矩阵的换算关系，就算起点一样他们之间也会存在换算关系。
5. odom是连续的，在现实中问问map这个由点云更新的不能立刻得出。所以需要用odom
## tf存在的作用
1. 记录相对位置关系
2. 利用map坐标修正相对位置关系。
## 时间插值
显然积分和实际坐标不可能是同步的，在时间上肯定有延迟。他会去预测，如果预测的时间间隔过大那就会报错
## 角度表示
使用四元数，因为欧拉角有万向节死锁的情况。
## 代码相关
1. 节点话题 /tf与/tf_static
2. 消息：geometry_msgs/TransformStamped
3. 发布 tf2_ros::TransformBroadcaster，订阅 tf2_ros::TransformListener,tf2_ros::Buffer tfBuffer;
4. 头文件：``` #include <tf2_ros/transform_listener.h> #include <tf2_geometry_msgs/tf2_geometry_msgs.h> ```
5. demo,发布，其实就是给好坐标系名字然后填好矩阵。
```
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_pub_node");
    ros::NodeHandle nh;

    // 1. 实例化动态 TF 广播器
    tf2_ros::TransformBroadcaster dynamic_broadcaster;

    ros::Rate rate(50); // 50Hz 高频发布
    double x = 0.0;

    while (ros::ok()) {
        // 模拟小车以 0.1m/s 的速度向前匀速滑行
        x += 0.002; 

        // 2. 填充 TF 核心数据结构
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";       // 父坐标系
        transformStamped.child_frame_id = "base_link";   // 子坐标系

        // 填充平移 (Translation)
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        // 填充旋转 (Rotation)：强制要求四元数，此处模拟无旋转
        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // Roll, Pitch, Yaw
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // 3. 发布出去（底层自动打包送入全局 /tf 话题）
        dynamic_broadcaster.sendTransform(transformStamped);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```

demo订阅
```
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_sub_node");
    ros::NodeHandle nh;

    // 1. 定义档案库与监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer); // 此句执行后，后台线程自动订阅 /tf

    ros::Rate rate(10); // 10Hz 周期性主动查询
    while (ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            // 2. 主动向 Buffer 库查询当前最新的 odom 到 base_link 的总变换矩阵
            // ros::Time(0) 代表获取“当前缓冲区里最新的一帧”
            transformStamped = tf_buffer.lookupTransform("odom", "base_link", ros::Time(0));

            ROS_INFO("成功获取变换！当前小车在 odom 系下的 X 坐标为: %.2f", transformStamped.transform.translation.x);
        } 
        catch (tf2::TransformException &ex) {
            // 刚开机时如果 pub 节点还没发数据，Buffer 里是空的，会报 LookupException，属于正常现象
            ROS_WARN("等待 TF 树连通: %s", ex.what());
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
```
## 命令行
1. rosrun tf view_frames
2. rosrun tf tf_echo <父系> <子系>
3. rosrun tf2_ros tf2_monitor 查看时延

# rviz与tf

## Fixed Frame
rviz世界的坐标系，固定为一个，其他发布轨迹要想正常显示都得转换为这个坐标系下的。发布方需要附带他处于哪个坐标系下：frame_id。
## 使用rviz
1. 只允许显示他规定的话题数据，因此自定义话题数据往往要转换为官方话题。
2. 写好frame_id和tf映射的才能正常显示。
3. rviz只认urdf和xacro生成的两种方式的模型，sdf不行，如果想要在rviz中显示该模型请先启动节点（sdf是gazebo独有的模型类型）
```
<!-- 1. 搬运肉体：把 3D 模型文件读进 ROS 核心的全局参数服务器（通常名字就叫 robot_description） -->
<param name="robot_description" command="$(find xacro)/xacro '$(find my_drone_description)/urdf/drone.xacro'" />
或者直接使用urdf：
<param name="robot_description" textfile="$(find my_drone_description)/urdf/drone.urdf" />

<!-- 2. 计算关节：启动关节状态发布器（如果是机械臂这种不是fixed安装的角度可变，它负责发角度） -->
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

<!-- 3. 编织成树：硬核节点！它负责把机器人的 URDF 结构和关节角度融合成实时的 TF 树广播出去 -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```
4. urdf里面的link就是坐标系，robot_state_publisher发读取这个把tf状态发布出去，请使用<group ns="drone_alpha"></group>把第三点的示例代码包入不然多个模型发布的坐标就重名了。同时启动转换节点时<param name="tf_prefix" value="drone_alpha" />
5. <node pkg="tf2_ros" type="static_transform_publisher" name="world_to_alpha" 
      args="0 0 0 0 0 0 map drone_alpha/base_link" /> 与rviz世界坐标系建立联系。
6. gazebo和mavros进行绑定。mavros的frame_id是urdf对应的link坐标，gazebo是对应的group名就算绑定成功。