这里将会记录在学习ego中遇到的语法问题。
# h文件和hpp文件的区别
h用在c或c与c++混编的情况。hpp是纯正C++情况的
## 模板的声明和使用必须要在一个文件以及内联情况
### 编译流程
- 预处理：把cpp文件中的include等下内容展开，处理得到obj
- 链接： include使用到别人的头文件但是函数cpp不知道需要对面把cpp编译输出为静态库lib。把obj和lib链接起来就是可执行文件了。使用静态库链接出来的可执行文件直接就能执行。
如果使用动态库链接出来的文件则需要本地有对应的.so这些动态库文件。
当然了生成自己代码的时候cpp文件就在附近直接哪来编译就好了不用放到lib
### 模板
模板如果定义里不马上使用由于类型不确定，所以不会生成任何机器码，所以需要明确把cpp写到hpp里确保声明和实现同时引用
### 内联
另外内联函数这种写在头文件里的其实违背了头文件的定位。但是现代链接器会把重复的函数删除冗余，只会造成编译时间的浪费而他本身节约调用时间的特性还在。
实际上是因为模板的存在放在必须要大段的塞入，干脆沿用这种，不过在使用这种的时候一般会使用hpp后缀这样别人就知道是c++的头文件格式他内部可能有很多函数实现。
# 文件介绍
1. polynomial是使用minisnap生成
2. uniform_bspline是实现B样条
## raycast
在三维栅格（Voxel/Grid）环境中，给出一条线段的起点 start 和终点 end，这两个函数能够高效地计算出这条三维线段到底穿过了哪些所有的栅格（体素）
## obj_predictor
fastplanner代码
## obj_generator.cpp和linear_obj_model.hpp
plan_env他是生成节点的。
## lbfgs
拟牛顿法求解优化问题。
## 核心算法涉及文件
1. plan_manage
- node节点
- fsm状态机，使用manage去调用底层的算法实现轨迹。
- manager管理，将底层的A* B样条优化，地图，数学求解器串联起来
2. traj——utils
- polynomial生成初始轨迹，一切的起点使用的地方很多，主要为管理和优化处。
- plan_container.hpp管理轨迹，使用地方为B样条优化与规划的manager
- plannig_visualization生成可视化轨迹，核心算法不涉及到但是我误放入了。
- msg，字面意思
2. bspline_opt
- lbfgs.hpp 最优化数学工具
- uniform，生成标准的B样条
- optimizer，优化B样条
3. path_searching
A*算法
4. plan_env
- grid_map,记录地图信息，不起到生成地图的作用。
- raycast，算法记录线段占据栅格。
- predictor,使用fastplanner，预测障碍物轨迹。
5. 
- grid_map和raycast更新哪里有障碍物，如果有移动障碍物则预测未来障碍物。
- fsm收到任务，manage使用traj_utils中的poly生成多项式轨迹，看是否这个轨迹经过障碍物，如果有就使用A*绕开并得到控制点。
- 将控制点丢给bspline_opt生成初始B样条，Optimizer 根据给定参数如碰撞距离等构建最优化问题使用lbfgs求解
- plan_container记录轨迹等待下一个周期同时可以考虑其他操作如可视化。
## 算法 to sim
1. traj_server 把订阅的B样条多项式方程转换成位置速度加速度发布
2. rosmsg_tcp_bridge 集群时使用用于可靠的广播自己的轨迹。
3. drone_detect 集群时使用，利用视觉识别周围无人机。
4. linear_obj_model.hpp，线性运动物体运动预测，他虽然引用在了obj预测里但是实际上代码没有用到。
5. gradient_descent_optimizer.h / cpp轻量化的数学求解器同样没用到。
## traj_server
1. quadrotor_msgs功能包定义仿真用指令信息。这是一个传承很久的包里面有着大量的历史遗留数据，没必要全看。这里只使用了#include "quadrotor_msgs/PositionCommand.h"
里面有的cpp文件是用来和真机传递数据的