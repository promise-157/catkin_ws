这里讲解cmake和package
# 寻找包逻辑
一般情况下其他包在编译的时候会选择输出add_library这些指令，产生编译过程文件。如果install了则在usr文件夹。没install则在/opt可能有。
对于lib，使用find_package找到的则用包::库目标名字。本工程的则直接目标名字
## find_package
获取包路径，即获取include和lib的路径变量这时cmake中的变量会被赋值。参数为包名。
至于如何输出包名则需要用到make install去生成对应的.cmake脚本，找包其实上就是去找这些脚本，除了环境变量找那些脚本还能通过CMAKE_PREFIX_PATH规定寻找路径。
例如：install(EXPORT MyRobotTargets
        DESTINATION lib/cmake/MyRobot
        NAMESPACE MyRobot::
        FILE MyRobotConfig.cmake)。
由于ros抽象出来了功能包的概念，他要求各个功能包都像正常cmake包一样，所以catkin会自动执行install到devel文件夹，因此才需要source devel/setup.bash，他会设置CMAKE_PREFIX_PATH。
还预留了catkin_ws install可以真正打包到usr文件夹
## include_directories和target_link_libraries
使用find指令获取的变量传递给预处理器include以及编译器lib的路径
## package.xml
就像他的后缀，他只是起到标记管理作用，因为功能包可能会有很多，所以需要这个。有了这个可以执行rosdep一键完成依赖。这是个生态位工具，很多ros相关的指令都要查询这个。
同时他又不可缺少，因为他指定了依赖关系，这也让他同时拥有多个main函数，package能确定这么多编译目标的依赖先后关系。
## 将功能包生成lib给其他包find_package
# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
#  DEPENDS system_lib
)
把上面的注释解除时就能把LIBRARIES ego_planner这个指定的依赖包名字即本包的include暴露给find_package.第三行是依赖，如果想要使用暴露的include，不仅要LIBRARIES ego_planner包含，依赖也要有。