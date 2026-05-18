这里将会介绍一下常用的库。
# signal.h
这是Linux自带的库。用于处理crtl c终止时结束节点防止意外。
## 用法
```
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

signal(SIGINT, mySigintHandler);
```