
#include "controller.h"

Controller::Controller(ros::NodeHandle& nh) : nh_(nh) {
    Init();
    // 设置默认的深度图像话题
    std::string depth_topic = "/camera/depth/image_raw";
    // 从ROS参数服务器获取各种参数，如果未设置，则使用默认值
    nh_.param("depth_topic", depth_topic, std::string("/camera/depth/image_raw"));
    nh_.param("cameras_param/rgb_K", cam.rgb_K, std::vector<double>());
    nh_.param("cameras_param/depth_K", cam.depth_K, std::vector<double>());
    nh_.param("cameras_param/rgb2depth_R", cam.rgb2depth_R, std::vector<double>());
    nh_.param("cameras_param/rgb2depth_T", cam.rgb2depth_T, std::vector<double>());
    nh_.param("rflysim/enable", rflysim_p.enable, true);
    nh_.param("rflysim/f_rgb", rflysim_p.f_rgb, 320.);
    nh_.param("rflysim/f_depth", rflysim_p.f_depth, 320.);
    nh_.param("rflysim/rgb_image_w", rflysim_p.rgb_image_width, 640);
    nh_.param("rflysim/rbg_image_h", rflysim_p.rgb_image_height, 480);
    nh_.param("rflysim/depth_image_w", rflysim_p.depth_image_width, 640);
    nh_.param("rflysim/depth_image_h", rflysim_p.depth_image_height, 480);
    nh_.param<std::vector<double>>(
        "rflysim/cam2body_R", rflysim_p.depth_cam2body_R, std::vector<double>());
    nh_.param<std::vector<double>>(
        "rflysim/cam2body_T", rflysim_p.depth_cam2body_T, std::vector<double>());
    nh_.param("rflysim/depth_down_sample", rflysim_p.depth_down_sample, 5);
    nh_.param("auto_arming", auto_arming, false);
    nh.param("takeoff_h", takeoff_h, 0.5);
    nh.param("takeoff_yaw", takeoff_yaw, 0.);
    nh.param("kx", kx, 0.0);
    nh.param("ky", ky, 0.0);
    nh.param("vx_max", vx_max, 0.0);
    nh.param("vy_max", vy_max, 0.0);
    nh.param("/rflysim/goal_x_t", rflysim_p.goal_x_t, 0.);
    nh.param("hight_max", rflysim_p.hight_max, 3.0);
    nh.param("rflysim/rgb_ppx", rflysim_p.rgb_cnt.x,
        rflysim_p.rgb_image_width / 2.);
    nh.param("rflysim/rgb_ppy", rflysim_p.rgb_cnt.y,
        rflysim_p.rgb_image_height / 2.);
    nh.param("rflysim/depth_ppx", rflysim_p.depth_cnt.x,
        rflysim_p.depth_image_width / 2.);
    nh.param("rflysim/depth_ppy", rflysim_p.depth_cnt.y,
        rflysim_p.rgb_image_height / 2.);
    nh.param("rflysim/min_score", rflysim_p.min_score, 0.7);
    nh.param("rflysim/rgb_fov_h", rflysim_p.rgb_fov_h, 90.);
    nh.param("rflysim/rgb_fov_v", rflysim_p.rgb_fov_v, 90.);
    nh.param("rflysim/is_sim", rflysim_p.is_sim, true);
    nh.param("is_S", rflysim_p.is_S, false);
    // 将水平和垂直视场角从度转换为弧度
    rflysim_p.rgb_fov_h *= (M_PI / 180);
    rflysim_p.rgb_fov_v *= (M_PI / 180);

    // 以下代码被注释掉，用于计算相机中心点坐标
    //  rflysim_p.rgb_cnt.x   = int(rflysim_p.rgb_image_width / 2);
    //  rflysim_p.rgb_cnt.y   = int(rflysim_p.rgb_image_height / 2);
    //  rflysim_p.depth_cnt.x = int(rflysim_p.depth_image_width / 2);
    //  rflysim_p.depth_cnt.y = int(rflysim_p.depth_image_height / 2);
    // 订阅深度图像话题
    depth_img_sub = nh.subscribe<sensor_msgs::Image>(
        depth_topic, 10,
        std::bind(&Controller::DepthImgCB, this, std::placeholders::_1));
    // 订阅飞行控制器状态、位置、轨迹、对象检测和里程计等话题
    fcu_state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10,
        std::bind(&Controller::RecvFcuState, this, std::placeholders::_1));
    fcu_pose_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10,
        std::bind(&Controller::RecvPose, this, std::placeholders::_1));
    tra_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "/planning/pos_cmd", 10,
        std::bind(&Controller::RecvTra, this, std::placeholders::_1));
    obj_sub = nh.subscribe<common_msgs::Objects>(
        "/objects", 10,
        std::bind(&Controller::RecvAiCB, this, std::placeholders::_1));
    
    odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/Odometry", 10,
        std::bind(&Controller::RecvLIO, this, std::placeholders::_1));
    aruco_sub = nh.subscribe<common_msgs::Aruco>(
        "/Aruco", 10,
        std::bind(&Controller::RecvAruco, this, std::placeholders::_1));
    // 发布姿态、控制指令和目标等话题
    gpio_cmd_pub = nh.advertise<std_msgs::String>("/drop_cmd", 10); 
    pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    ctrl_cmd_pub =
        nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    goal_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // 发布可视化和点云数据
    vis_pub = nh.advertise<sensor_msgs::PointCloud2>("/test", 10);
    full_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/full_points", 10);
    task_state_pub = nh.advertise<common_msgs::MissionState>("/task_state", 10);
    // 创建设置飞行模式和解锁服务客户端
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 创建定时器，以固定频率调用Run函数
    run_timer =
        nh_.createTimer(ros::Duration(0.03),
            std::bind(&Controller::Run, this,
                std::placeholders::_1));
    ROS_INFO("!!!!!!!!!!!!--------------!!!!!!!!!!!!!!construct  finished");
}

/**
 * @brief 初始化控制器的状态变量和参数
 *
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool Controller::Init() {
    // ctrl_source = CtrlSource::Servo;
    // //使用伺服控制，但前提是相机的视场角要够大，下视相机视场角够大，所以在降落的时候可以使用伺服控制

    // 初始化各种状态变量
    is_recv_pose = false;
    is_recv_state = false;
    is_recv_tra = 0;
    is_aruco = false;
    is_H = false;
    arming = false;
    goal_flag = 0;
    send_goal_flag = 0;

    // 设置初始控制源为轨迹控制
    ctrl_source = CtrlSource::Trajectroy;

    // 设置任务状态为起飞
    mission = Mission::takeoff;

    // 初始化自动起飞标志
    auto_arming = false;

    // 初始化帧接收标志
    is_rec_frame1 = false;
    is_rec_frame2 = false;

    // 设置OFFBOARD模式请求
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // 设置自动着陆请求
    land.request.custom_mode = "AUTO.LAND";
    // 设置解锁命令请求
    arm_cmd.request.value = true;
    //  mission_points;
     // 初始化命令消息
    cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
    cmd.type_mask =
        ~uint16_t(0) & ~(uint16_t(0xff) << 12);  // 设置type_mask为0000 1111 1111 1111
    cmd.type_mask &= (~uint16_t(Command::FORCE)); // 屏蔽FORCE位


    // 设置命令消息的frame_id
    cmd.header.frame_id = "odom";

    // 初始化控制增益
    kx = 0.;
    ky = 0.;

    // 打印初始化完成的日志
    ROS_INFO("finished init1111111111111111111");

    // 返回初始化成功
    return true;
}



/*
 * 函数作用：根据当前任务状态执行相应的控制命令，以实现无人机的不同飞行任务。
 * 关键参数：
 * - mission: 当前任务状态，决定了无人机执行的具体操作。
 * - fcu_pose: 无人机当前位置信息。
 * - cmd: 控制命令对象，用于发送控制指令。
 * - ctrl_cmd_pub: 发布控制命令的ROS Publisher。
 * - is_aruco: 是否检测到Aruco标记。
 * - is_H: 是否检测到H标记。
 * - rflysim_p: 配置参数对象，包含最大高度等信息。
 * - obj: 检测到的目标对象信息。
 * - vx_max, vy_max: 最大速度限制。
 * - fcu_state: 无人机当前状态信息，包括模式等。
 */
int PointIndex = 0;
double currentHight = 1.0;
int class_name = 0;
std_msgs::String gpio_cmd;

// {-1.62, -2.80,1.11},  //A 
// {-1.69, -2.08,1.08},  //B 
// {-1.66, -3.50,1.11},  //C 
std::array<std::array<double,3>,13> targetPoints = {{
    {0.0, 0.0,currentHight},  // 无动作点
    
    {-1.00, 0.07,currentHight},
    {-1.58, -0.08,currentHight},  // 无动作点
    {-2.63, 0.05,currentHight},  // 无动作点
    {-3.57, -0.00,currentHight},  // 无动作点

    //
    {-3.60, -1.11,currentHight},  // 
    {-3.66, -1.89,currentHight},  // 无动作点
    {-3.70, -2.94,currentHight}, // kai qi xia shi
    {-3.70, -2.94,currentHight}, // kai qi xia shi

    {-2.58, -2.58,currentHight},  // 
    {-1.66, -3.50,1.11},  //mu biao dian

    {-1.66, -3.50,0.5},  // 无动作点
    {-1.66, -3.50,0} 
    }};


int MaxIndex = targetPoints.size();
void Controller::Run(const ros::TimerEvent& e) {
    //如果有定位数据
    if (!is_recv_pose || !is_recv_state) return;
    if (!fcu_state.connected) return;
    //  PrintMission();

    StateChange();  //zhuai tai 
    switch (mission) {
    case Mission::takeoff:
        if (fcu_state.mode != "OFFBOARD") {  //还没及切换模式，先发目标值给非控制切换模式
            cmd.type_mask &=
                ~(Command::IGNORE_PX | Command::IGNORE_PY | Command::IGNORE_PZ
                    | Command::IGNORE_YAW);  // 使用位置控制起飞
            for (int i = 0; i < 10; ++i) {
                cmd.position.x = 0;
                cmd.position.y = 0;
                cmd.position.z = takeoff_h;  // 起飞高度是0.5m
                cmd.yaw = takeoff_yaw;
                ctrl_cmd_pub.publish(cmd);
                ros::spinOnce();
                sleep(0.01);
            }
            if (auto_arming && set_mode_client.call(offb_set_mode)
                && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
        }
        if (fcu_state.mode == "OFFBOARD" && auto_arming && !fcu_state.armed) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
        }
        if (std::abs(fcu_pose.pose.position.z - takeoff_h) < 0.1) {  // 飞到指定高度，切换模式
            ROS_INFO("CURRENT_takeoff_yaw: %f",takeoff_yaw);
            sleep(5);
            mission = Mission::fly_to_points;
        }
        ctrl_cmd_pub.publish(cmd);
        break;
         
    case Mission::fly_to_points:
        if (fcu_state.mode == "OFFBOARD" && fcu_state.armed) {
            geometry_msgs::Point target;
            if(PointIndex < MaxIndex) {
                target.x = targetPoints[PointIndex][0];
                target.y = targetPoints[PointIndex][1];
                target.z = targetPoints[PointIndex][2];
            }

            // 发布目标位置 
            cmd.type_mask &= ~(Command::IGNORE_PX | Command::IGNORE_PY | Command::IGNORE_PZ | Command::IGNORE_YAW);
            cmd.position.x = target.x;
            cmd.position.y = target.y;
            cmd.position.z = target.z;
            cmd.yaw = takeoff_yaw;  // 保持原有的yaw角度
 
            // 检查是否到达目标点
            double diff_x = std::abs(fcu_pose.pose.position.x - target.x);
            double diff_y = std::abs(fcu_pose.pose.position.y - target.y);
            double diff_z = std::abs(fcu_pose.pose.position.z - target.z);
            // ROS_INFO("---------PointIndex,diff_x,diff_y,diff_z:  %d,%.2f,%.2f,%.2f :",PointIndex,diff_x,diff_y,diff_z);
            // ROS_INFO("CURRENT_takeoff_yaw: %f",takeoff_yaw);
            if (diff_x < 0.13 && diff_y < 0.13 && diff_z < 0.1 && PointIndex< MaxIndex) {
                common_msgs::MissionState state; 
                ROS_INFO("Arrive***the point***************************************");
                switch(PointIndex){
                    case 4:
                        std::this_thread::sleep_for(std::chrono::milliseconds(2800));
                        break;
                    case 7:  // kai xia shi 
                        sleep(1);
                        state.header.stamp = ros::Time::now();
                        state.task = uint8_t(5);
                        task_state_pub.publish(state);
                        ros::spinOnce();
                        sleep(1);
                        // ROS_INFO("CURRENT_TASK______________: %d",state.task);
                        break;
                    case 8:
                        std::this_thread::sleep_for(std::chrono::milliseconds(4600));
                        // PointIndex = PointIndex+1;
                        break;
                    case 9:  // kai  qian shi 
                        sleep(1);
                        state.header.stamp = ros::Time::now();
                        state.task = uint8_t(4);
                        task_state_pub.publish(state);
                        ros::spinOnce();
                        sleep(1);
                        break;
                    case 10: 
                        std::this_thread::sleep_for(std::chrono::milliseconds(666));
                        gpio_cmd.data = "Blink";
                        gpio_cmd_pub.publish(gpio_cmd);
                        ros::spinOnce();
                        std::this_thread::sleep_for(std::chrono::milliseconds(6000));
                        break;
                    case 12:
                        sleep(1);
                        ROS_INFO("preparing to land!");
                        arming = true;
                        if (arming && fcu_pose.pose.position.z < 0.1){
                            ROS_INFO("land land land");
                            set_mode_client.call(land);
                        }
                        break;                   
                    default:
                        break;
                }       
                ++PointIndex;
            }
        }
        ctrl_cmd_pub.publish(cmd);
        break;
    

    case Mission::land:
        //降落模式 切换到伺服控制
        // cmd.type_mask |= Command::IGNORE_PX | Command::IGNORE_PY
        //                  | Command::IGNORE_PZ;
        // cmd.type_mask &=
        //   (Command::IGNORE_VX | Command::IGNORE_VY | Command::IGNORE_VZ);
       // cmd.coordinate_frame =
        //  Command::FRAME_BODY_NED;  // 速度控制基于body坐标系控制
        //if(ros::Time::now().toSec() - cmd.header.stamp.toSec() > 10)
        //{  //如果超过两秒中没有更新指令，直接降落
          //set_mode_client.call(land);
        //}
        cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
        cmd.type_mask &= ~(Command::IGNORE_VX | Command::IGNORE_VY
            | Command::IGNORE_VZ);  // 使用速度控制；
        cmd.header.frame_id = "base_link";
        if (arming) {
            cmd.velocity.x = 0;
            cmd.velocity.y = 0;
            cmd.velocity.z = -0.4;
            ROS_INFO("fu zhi succfull");
            if(fcu_pose.pose.position.z < 0.1)
            {
                ROS_INFO("land land land");
               set_mode_client.call(land);
               break;
           }

        }
        ROS_INFO("TO LAND vx:%.3f,vy:%.3f,vz:%.3f", cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
        ctrl_cmd_pub.publish(cmd);

        break;
    case Mission::end:
        //发生异常情况，直接降落
        ROS_INFO("zhi jie land");
        break;
    case Mission::cross_frame1:
    case Mission::cross_frame2:
        ROS_WARN("send cmd to contrl drone");
        //发送控制命令前，得先发目标点给规划节点
        ctrl_cmd_pub.publish(cmd);
        break;
    case Mission::recognize_aruco:
        //如果到识别二维码的状态，可能二维码没在下视相机视场角内，这个时候应该飞高一点，使二维码或者降落标志在视场内
        cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
        cmd.type_mask &= ~(Command::IGNORE_VX | Command::IGNORE_VY
            | Command::IGNORE_VZ);  // 使用速度控制；
        cmd.header.frame_id = "base_link";
        cmd.velocity.x = 0.01;
        cmd.velocity.y = 0.4;  //二维码趋势在此时机体坐标系的y方向
        if (!is_aruco) {  //此时二维码没在视场角内，应该飞高一点，但是如果识别到了降落标志，还没识别到二维码，说明图像清晰度不够，应超降落标志飞去，逐渐降低高度
            ROS_INFO("find aruco");
            if (rflysim_p.is_S) {
                cmd.velocity.y *= -1;
            }
            cmd.velocity.z = 0.3;  //速度不能太快，当然高度需要加上一个约束值，与此同时需要考虑气流的影响；
            if (fcu_pose.pose.position.z > rflysim_p.hight_max) {  //此时高度，不能再高了,此时降落表示应该识别了
                ROS_ERROR("vichle'z is too height");
                cmd.velocity.z = 0;

            }
            if (fcu_pose.pose.position.y > 1.5) {
                cmd.velocity.y = 0;
            }
            /*
            if(is_H)
            {  //发现降落标识，没识别出二维码
              //视觉伺服控制，x,y 往降落标志点飞行
              auto dx =
                (obj.left_top_x + obj.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
              auto dy =
                (obj.left_top_y + obj.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
              cmd.velocity.y =
                -dx * kx;  //至于方向，需要确定下视单目图像坐标系与机体坐标系的关系
              cmd.velocity.x = dy * ky;
              if(fcu_pose.pose.position.z < 2.5)
                cmd.velocity.z = 0.1;  //继续上升以便识别二维码
              else
              {  //这种情况放弃识别二维码
                mission = Mission::land;
              }
              cmd.header.stamp = ros::Time::now();
            }*/
        }
        ctrl_cmd_pub.publish(cmd);
        ROS_INFO("Recon arco vx:%f,vy:%f, vz:%f", cmd.velocity.x, cmd.velocity.y, cmd.velocity.z);
        ros::spinOnce();
        cmd.velocity.y = 0;
        cmd.velocity.z = 0;
        break;
    case Mission::recognize_H:
        //如果识别出了二维码，没扫描到降落表示，可能降落标志没在视场内，此时应飞高，并往识别了二维码的位置飞去
        // cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
        cmd.type_mask &= ~(Command::IGNORE_VX | Command::IGNORE_VY
            | Command::IGNORE_VZ);  // 使用速度控制；
        cmd.coordinate_frame = Command::FRAME_BODY_NED;
        if (!is_H) {  //没识别到降落标志
            /*
              static  bool is_cnt_aurco = false;
              std::cout << " aruco: " << aruco.cnt_x << ", " << aruco.cnt_y
                        << std::endl;
              std::cout << " rgb_cnt: " << rflysim_p.rgb_cnt.x << ", "
                        << rflysim_p.rgb_cnt.y << std::endl;
              int    dx = int(aruco.cnt_x - rflysim_p.rgb_cnt.x);
              int    dy = int(aruco.cnt_y - rflysim_p.rgb_cnt.y);
              double vx = -dy * kx;
              double vy = -dx * ky;
              if(std::abs(vx) > vx_max)
                vx = vx / std::abs(vx) * vx_max;
              if(std::abs(vy) > vy_max)
                vy = vy / std::abs(vy) * vy_max;
              std::cout << "dx: " << dx << ", dy: " << dy << std::endl;
              std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
                        << std::endl;
              cmd.velocity.x   = vx;
              cmd.velocity.y   = vy;
              cmd.velocity.z   = 0.05;

              if(fcu_pose.pose.position.z > rflysim_p.hight_max)
              {  //此时高度，不能再高了,此时降落表示应该识别了
                ROS_ERROR("vichle'z is too height");
                cmd.velocity.z = 0;
              }
              if(std::abs(dx) < 50 && std::abs(dy) < 50)
              {
                is_cnt_aurco = true;
              }

              if(is_cnt_aurco)
              {
                cmd.velocity.x = 0.2;
              }
              */
            cmd.velocity.x = 0.2;
            cmd.velocity.y = 0;
            cmd.header.stamp = ros::Time::now();
            std::cout << "vz: " << cmd.velocity.z << std::endl;
            ctrl_cmd_pub.publish(cmd);
        }
        is_H = false;
        break;
    };
    ros::spinOnce();
}


/**
 * @brief 深度图像回调函数
 *
 * 该函数在接收到深度图像数据时被调用。它的主要作用是处理接收到的深度图像数据，
 * 包括根据时间戳更新数据队列，以确保处理的图像数据是最近的。
 *
 * @param depth 深度图像消息指针
 */
void Controller::DepthImgCB(const sensor_msgs::Image::ConstPtr& depth) {
    // 如果在真机模式下运行，则不执行任何操作并直接返回
    if (!rflysim_p.is_sim) {
        return;
    }
    // 锁定互斥锁以确保线程安全
    std::unique_lock<std::mutex> lock(depth_mtx);
    // 将接收到的深度图像数据添加到队列中
    depth_queue.push(*depth);
    // 循环检查并移除队列中超过一秒的旧数据
    while (true) {
        // 获取队列中最前面的深度图像数据
        auto front = depth_queue.front();
        // 如果当前时间与数据时间戳的差大于一秒，则移除该数据
        if (ros::Time::now() - front.header.stamp > ros::Duration(1)) {  //把超过一秒外的数据丢弃
            depth_queue.pop();
        } else {
            // 如果数据时间戳在可接受范围内，则退出循环
            return;
        }
    }
}
/**
 * @brief 接收并处理无人机的姿态信息
 *
 * 该函数通过订阅ROS主题获取无人机的里程计信息，包括位置和姿态，并将其保存在类成员变量中
 * 此外，它还会更新接收状态标志，以指示已成功接收到姿态信息
 *
 * @param pose 指向里程计消息的常量指针，包含无人机的位置和姿态信息
 */
void Controller::RecvPose(const nav_msgs::Odometry::ConstPtr& pose) {
    fcu_pose.pose = pose->pose.pose;
    fcu_pose.header = pose->header;
    is_recv_pose = true;
}


/**
 * @brief 接收并处理无人机的姿态信息
 *
 * 该函数通过订阅ROS主题获取无人机的里程计信息，包括位置和姿态，并将其保存在类成员变量中
 * 此外，它还会更新接收状态标志，以指示已成功接收到姿态信息
 *
 * @param pose 指向里程计消息的常量指针，包含无人机的位置和姿态信息
 */
void Controller::RecvFcuState(const mavros_msgs::State::ConstPtr& state) {
    fcu_state = *state;
    is_recv_state = true;
    ;
}


/**
 * @brief 处理聚类结果并转换坐标系
 *
 * 该函数首先检查聚类结果是否成功。如果成功，将传感器坐标系下的坐标转换为机体坐标系，再通过飞机的位姿信息将机体坐标系下的坐标转换为世界坐标系。最后，根据任务状态发送目标点。
 *
 * @param tmp 指向聚类结果的指针，包含传感器坐标系下的点云数据
 * @param frame_cnt 聚类后的中心点坐标
 * @param rflysim_p 包含深度相机到机体坐标系的旋转和平移矩阵
 * @param fcu_pose 无人机的位姿信息
 * @param goal_point 目标点消息，用于发布到规划器
 * @param mission 当前任务状态
 * @param goal_flag 目标点状态标志
 * @param send_goal_flag 发送目标点的状态标志
 * @param goal_1 第一个目标点
 * @param goal_2 第二个目标点
 * @param is_rec_frame1 是否已接收第一个框
 * @param is_rec_frame2 是否已接收第二个框
 * @param goal_pub 目标点消息的发布者
 */
void Controller::RecvAiCB(const common_msgs::Objects::ConstPtr& objs) {  //假设已经收到了这样的结构体std::vector<frame>，frame:left_top_x left_top_y
    //,right_button_x, right_button_y;
    if (objs->objects.empty())
        return;
    if (objs->objects[0].class_name == "A"){
        class_name = 1;
    }
    if (objs->objects[0].class_name == "B"){
        class_name = 2;
    }
    if (objs->objects[0].class_name == "C"){
        class_name = 3;
    }
    // ROS_INFO("class name: %d",class_name);
    // ROS_INFO("class name: %s",objs->objects[0].class_name.c_str());
    /*
   if(mission == Mission::recognize_aruco)
   {
     if(objs->source_id == 1)
         return;
     if(objs->objects.size() == 1 && objs->objects[0].score > rflysim_p.min_score
        && objs->objects[0].class_name == "land")
     {
       static int recon_times = 0;
       recon_times++;
       static auto t = ros::Time::now();
       if(ros::Time::now().toSec() - t.toSec() > 0.5)
       {
         recon_times = 0;
         t = ros::Time::now();
       }
       if(recon_times > 5)
       {
         is_H = true;
         obj  = objs->objects[0];
       }
     }

   }
   */
    if (mission == Mission::recognize_H || mission == Mission::land) {  //进入降落模式,使用伺服控制，计算vx,vy， 基于body 坐标系
        if (objs->source_id == 1)
            return;
        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.velocity.z = 0;
        for (int i = 0; i < objs->objects.size(); ++i) {
            if (objs->objects[i].class_name == "land"
                && objs->objects[i].score > rflysim_p.min_score) {
                static int times = 0;
                times++;
                static auto t = ros::Time::now();
                if (ros::Time::now().toSec() - t.toSec() > 0.5) {
                    times = 0;
                    t = ros::Time::now();
                }
                if (times < 5 && !is_H) {//会把目标小车的轨迹当成land目标，因此连续5帧识别才有效
                    ROS_ERROR("not available land object");
                    return;
                }
                is_H = true;
                ROS_INFO("recv H flag");
                auto obj = objs->objects[i];
                auto dx =
                    (obj.left_top_x + obj.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
                auto dy =
                    (obj.left_top_y + obj.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
                std::cout << "land dx:  " << dx << " dy: " << dy << std::endl;
                double vx = -dy * kx;
                double vy = -dx * ky;
                if (std::abs(vx) > vx_max)
                    vx = vx / std::abs(vx) * vx_max;
                if (std::abs(vy) > vy_max)
                    vy = vy / std::abs(vy) * vy_max;
                cmd.velocity.x = vx;
                cmd.velocity.y = vy;
                std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
                    << std::endl;
                if (std::abs(dx) < 20 && std::abs(dy) < 20) {
                    cmd.velocity.z = -0.4;  // 符合降落条件了
                    //set_mode_client.call(land);
                    ROS_INFO("inter land mode, use velocity control");
                    if (fcu_pose.pose.position.z < 1.2)
                        arming = true;
                    mission = Mission::land;
                }
                cmd.header.stamp = ros::Time::now();
            }
        };
        return;
    }
    //  if((mission != Mission::cross_frame1 || mission != Mission::cross_frame2)
    //     && mission_points.find(mission) != mission_points.end())
    //  {  //如果不是穿框任务，或者已经找到目标框的中心位置；
    //    return;
    //  }

    if (mission != Mission::cross_frame1 && mission != Mission::cross_frame2) {  //如果不是穿框或者穿环,下面的流程不用走了。
        return;
    }

    if (!rflysim_p.is_sim) {  //如果是真机模式需要使用深度读取
        pcl::PointXYZ cnt;
        ObjConterRGB(objs, &cnt);
        ROS_INFO("camera corr p: x:%f, y:%f, z:%f", cnt.x, cnt.y, cnt.z);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
            new pcl::PointCloud<pcl::PointXYZ>());
        //因为直接使用的检测框作为目标框，得加上一个往前的偏移量
        cnt.z += rflysim_p.goal_x_t;
        ROS_INFO("cnt.z:%f goal_x_t:%f", cnt.z, rflysim_p.goal_x_t);
        tmp->points.push_back(cnt);
        CoordinateTrans(tmp); // jiao dian jian ce 
        VisionPointCloud(tmp, vis_pub);
        goal_point.pose.position.x = tmp->points[0].x;
        goal_point.pose.position.y = tmp->points[0].y;
        goal_point.pose.position.z = tmp->points[0].z;
        ROS_INFO("goal: x:%f,y:%f,z:%f", goal_point.pose.position.x, goal_point.pose.position.y, goal_point.pose.position.z);
        if (mission == Mission::cross_frame1) {
            goal_flag = 1;
            if (send_goal_flag == 0) {
                goal_1 = goal_point;
                is_rec_frame1 = true;
                goal_pub.publish(goal_1);
                ROS_INFO("goal1: x:%f,y:%f,z:%f", goal_1.pose.position.x, goal_1.pose.position.y, goal_1.pose.position.z);
                ros::spinOnce();
                send_goal_flag = 1;
            }
        } else if (mission == Mission::cross_frame2) {
            if (send_goal_flag == 1) {
                double dx = goal_1.pose.position.x - goal_point.pose.position.x;
                double dy = goal_1.pose.position.y - goal_point.pose.position.y;
                double dz = goal_1.pose.position.z - goal_point.pose.position.z;
                if (std::sqrt(dx * dx + dy * dy) > 1) {  //表示以及识别出第二个框
                    goal_2 = goal_point;
                    goal_2.pose.position.x += 0.5;
                    is_rec_frame2 = true;

                    ROS_INFO("goal2: x:%f,y:%f,z:%f", goal_2.pose.position.x, goal_2.pose.position.y, goal_2.pose.position.z);
                    ROS_ERROR("REPEAT SEND POSE");
                    ROS_WARN("goal_x_t: %f", rflysim_p.goal_x_t);
                    goal_pub.publish(goal_2);
                    ros::spinOnce();
                    send_goal_flag = 2;
                    goal_flag = 2;
                }
            }
        }
        return;
    }
    sensor_msgs::Image depth;
    //这里需要做坐标转换，从RGB像素到深度图像里面的像素
    {  //需要这么几个矩阵，两个相机的畸变矫正矩阵，RGB相机到深度相机的变换矩阵，因为仿真里面没有畸变，也不产生旋转与平移，那么可以直接计算。
        if (depth_queue.empty()) {
            ROS_WARN("current depth queue is empty!");
            return;
        }
        //找与图像目标检测时间最近的深度图
        bool                         is_find_data = false;
        std::unique_lock<std::mutex> lock(depth_mtx);
        while (depth_queue.size() > 0)

        {
            auto   front = depth_queue.front();
            double dt = front.header.stamp.toSec() - objs->header.stamp.toSec();
            if (std::abs(dt) < 0.1) {
                depth = front;
                depth_queue.pop();
                is_find_data = true;
                break;
            } else {
                ROS_WARN("depth_queue size: %d, %f ", depth_queue.size(), dt);
                depth_queue.pop();
            }
            if (depth_queue.empty()) {

                // ROS_WARN("not find the stamp recentest depth image");
                // sleep(0.1); // 睡眠一个时间，让深度图赋值
                return;
            }
        }
        lock.unlock();
        if (is_find_data == false) {
            return;
        }
    }
    if (mission == Mission::cross_frame1 && is_rec_frame1)
        return;
    if (mission == Mission::cross_frame2 && is_rec_frame2)
        return;
    std::cout << " find recent depth image: " << depth.width << ", "
        << depth.height << std::endl;

    if (rflysim_p.enable) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        static auto scale = rflysim_p.f_depth / rflysim_p.f_rgb;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr =
                cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat depth_img = cv_ptr->image;
        //    cv::imshow("depth", depth_img);
        //    cv::waitKey(0);
        for (size_t i = 0; i < objs->objects.size(); ++i) {
            if (objs->objects[i].score < rflysim_p.min_score) {  //置信度低的不做考虑，按原指令执行
                continue;
            }
            //截取深度图部分点云
            std::cout << "det left_top: " << objs->objects[i].left_top_x << ", "
                << objs->objects[i].left_top_y << std::endl;
            std::cout << "det right_bottom: " << objs->objects[i].right_bottom_x
                << ", " << objs->objects[i].right_bottom_y << std::endl;
            std::cout << "rgb_cnt: " << rflysim_p.rgb_cnt.x << ","
                << rflysim_p.rgb_cnt.y << std::endl;
            std::cout << "depth_cnt: " << rflysim_p.depth_cnt.x << ","
                << rflysim_p.depth_cnt.y << std::endl;

            int dx_left = objs->objects[i].left_top_x - rflysim_p.rgb_cnt.x;
            int dy_left = objs->objects[i].left_top_y - rflysim_p.rgb_cnt.y;
            int dx_right = objs->objects[i].right_bottom_x - rflysim_p.rgb_cnt.x;
            int dy_right = objs->objects[i].right_bottom_y - rflysim_p.rgb_cnt.y;

            cv::Point2i left;
            cv::Point2i right;
            left.x = int(scale * dx_left + rflysim_p.depth_cnt.x);
            left.y = int(scale * dy_left + rflysim_p.depth_cnt.y);
            right.x = int(scale * dx_right + rflysim_p.depth_cnt.x);
            right.y = int(scale * dy_right + rflysim_p.depth_cnt.y);
            std::cout << " lddeft: " << left << " , right: " << right << std::endl;
            if (left.x < 0 || left.x >= depth_img.cols || left.y < 0
                || left.y >= depth_img.rows || right.x < 0 || right.x >= depth_img.cols
                || right.y < 0 || right.y >= depth_img.cols) {
                ROS_ERROR("please check params is correct");
                return;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
                new pcl::PointCloud<pcl::PointXYZ>());

            //      cv::Mat frame = depth_img(cv::Rect(left, right)).clone();
            //      DepthImgToCloud(tmp, &frame, left, right);
            //      left.x  = 0;
            //      left.y  = 0;
            //      right.x = depth_img.cols;
            //      right.y = depth_img.rows;
            //      DepthImgToCloud(tmp, &depth_img, left, right);
            //      cloud->points.insert(tmp->points.begin(), tmp->points.end());
            {
                pcl::PointXYZ cnt;
                ObjConterRGB(objs, &cnt);
                ROS_ERROR("rgb obj cnt: x:%f,y:%f,z:%f", cnt.x, cnt.y, cnt.z);
                tmp->points.push_back(cnt);
                CoordinateTrans(tmp);
                VisionPointCloud(tmp, vis_pub);

                pcl::PointCloud<pcl::PointXYZ>::Ptr full(
                    new pcl::PointCloud<pcl::PointXYZ>());

                DepthImgToCloud(full, &depth_img, cv::Point2i(0, 0), cv::Point2i(0, 0));
                CoordinateTrans(full);
                VisionPointCloud(full, full_points_pub);
            }
            return;
            cv::Point3d frame_cnt;
            if (Cluster(tmp, &frame_cnt)) {  //当前仅仅是传感器坐标系的坐标标，应该转换成世界坐标系下的坐标，然后往规划器里面发送
                //具体操作如下，1.把传感器坐标系的坐标转到机体坐标系,2.通过飞机位姿获得旋转与平移,将机体坐标系下的目标点位置转换成世界坐标系的位置
                std::cout << frame_cnt << std::endl;
                //对这个点进行坐标转；
                //获取飞机的在全局坐标系下的位姿
                std::vector<double> R = rflysim_p.depth_cam2body_R;
                std::vector<double> T = rflysim_p.depth_cam2body_T;
                double              x =
                    frame_cnt.x * R[0] + frame_cnt.y * R[1] + frame_cnt.z * R[2] + T[0];
                double y =
                    frame_cnt.x * R[3] + frame_cnt.y * R[4] + frame_cnt.z * R[5] + T[1];
                double z =
                    frame_cnt.x * R[6] + frame_cnt.y * R[7] + frame_cnt.z * R[8] + T[2];
                std::cout << ">>>>>>>>>>>>>>bc: " << x << "," << y << "," << z
                    << std::endl;
                //从body坐标系到世界坐标系
                tf2::Vector3    bc(x, y, z);
                tf2::Quaternion q;
                tf2::convert(fcu_pose.pose.orientation, q);
                tf2::Vector3 t;
                tf2::convert(fcu_pose.pose.position, t);

                tf2::Transform trans;
                trans.setOrigin(t);
                trans.setRotation(q);
                tf2::Vector3  wc = trans * bc;
                pcl::PointXYZ cnt_p;
                cnt_p.x = bc.getX();
                cnt_p.y = bc.getY();
                cnt_p.z = bc.getZ();
                {
                    //@back
                    //          for(size_t i = 0; i < tmp->points.size(); ++i)
                    //          {
                    //            auto         p = tmp->points[i];
                    //            double       x = p.x * R[0] + p.y * R[1] + p.z * R[2] +
                    //            T[0]; double       y = p.x * R[3] + p.y * R[4] + p.z *
                    //            R[5] + T[1]; double       z = p.x * R[6] + p.y * R[7] +
                    //            p.z * R[8] + T[2]; tf2::Vector3 b_p(x, y, z);

                    //            tf2::Vector3 w_p = trans * b_p;
                    //            // tf2::Vector3 w_p            = b_p;
                    //            tmp->points[i].x = w_p.getX();
                    //            tmp->points[i].y = w_p.getY();
                    //            tmp->points[i].z = w_p.getZ();
                    //          }
                    //          VisionPointCloud(tmp);
                }

                // if(std::abs(goal_point.pose.position.x - wc.getX()) > 0.31 ||
                //    std::abs(goal_point.pose.position.y - wc.getY()) > 0.31 ||
                //    std::abs(goal_point.pose.position.z - wc.getZ()) > 0.31)
                //    {//识别目标中心位置有更新，重新规划路径
                //     goal_point.pose.position.x = wc.getX();
                //     goal_point.pose.position.y = wc.getY();
                //     goal_point.pose.position.z = wc.getZ();
                //     goal_pub.publish(goal_point);
                //     ros::spinOnce();
                //    }

                goal_point.pose.position.x = wc.getX();
                goal_point.pose.position.y = wc.getY();
                goal_point.pose.position.z = wc.getZ();
                {
                    // pcl::PointXYZ p;
                    // p.x = wc.getX();
                    // p.y = wc.getY();
                    // p.z = wc.getZ();
                    // VisionPointCloud(&p);
                }
                if (mission == Mission::cross_frame1) {
                    goal_flag = 1;
                    if (send_goal_flag == 0) {
                        goal_1 = goal_point;
                        is_rec_frame1 = true;
                        goal_pub.publish(goal_1);
                        ros::spinOnce();
                        send_goal_flag = 1;
                    }
                } else if (mission == Mission::cross_frame2) {
                    if (send_goal_flag == 1) {
                        double dx = goal_1.pose.position.x - goal_point.pose.position.x;
                        double dy = goal_1.pose.position.y - goal_point.pose.position.y;
                        double dz = goal_1.pose.position.z - goal_point.pose.position.z;
                        if (std::sqrt(dx * dx + dy * dy) > 1) {  //表示以及识别出第二个框
                            tf2::Vector3 goal_x(rflysim_p.goal_x_t, 0, 0);
                            tf2::Vector3 p = trans * goal_x;
                            goal_point.pose.position.x + p.getX();
                            goal_point.pose.position.y + p.getY();
                            goal_point.pose.position.z + p.getZ();
                            goal_2 = goal_point;
                            is_rec_frame2 = true;
                            ROS_ERROR("REPEAT SEND POSE");
                            ROS_WARN("goal_x_t: %f", rflysim_p.goal_x_t);
                            goal_pub.publish(goal_2);
                            ros::spinOnce();
                            send_goal_flag = 2;
                            goal_flag = 2;
                        }
                    }
                }
            } else {
                ROS_ERROR("cluser fail");
            }
        }
    }
    std::cout << ">>>>>>>>frame position in world: " << goal_point.pose.position.x
        << "," << goal_point.pose.position.y << ","
        << goal_point.pose.position.z << std::endl;
}



/**
 * 根据检测到的对象信息计算目标中心点的RGB相机坐标系坐标。
 * 该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标定），
 * 严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置。
 *
 * @param objs 检测到的对象信息，包含多个对象的检测框、得分等信息。
 * @param cnt 输出参数，用于存储计算得到的目标中心点坐标。
 */
void Controller::ObjConterRGB(const common_msgs::Objects::ConstPtr objs,
    pcl::PointXYZ* cnt) {  //该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标定），严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置

    //static double scale_ =
    //  rflysim_p.rgb_image_width / rflysim_p.depth_image_height;
    static double scale_ = 1;

    if (objs->objects.empty())
        return;

    for (int i = 0; i < objs->objects.size(); ++i) {  //可能会检测出多个目标，刷选的方方法有的有很多，例如通过score
        //过滤,，然后通过框的相对位置，比如第一个框一定在第二个框的右边等等
        auto& obj = objs->objects[i];
        if (obj.score < rflysim_p.min_score)
            continue;

        auto width = obj.right_bottom_x - obj.left_top_x;
        auto height = obj.right_bottom_y - obj.left_top_y;

        auto ret = width / height;
        if (ret - scale_ > 0.5) {
            //如果比大很多，那可以判断这个目标不是我们需要的目标；
            continue;
        }

        //计算传感器的宽度和高度，unit:mm
        static double s_w = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_h / 2);
        static double s_h = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_v / 2);
        static double pw = s_w / rflysim_p.rgb_image_width;   //像素的宽度
        static double ph = s_h / rflysim_p.rgb_image_height;  //像素的高度
        static double wf = 1.3 * rflysim_p.f_rgb;


        //目标宽，与高比值1：1，1.3m, 默认目标框的高度是没有遮挡的，
        //先还原如果目标框没有被遮挡，应该在图像上的什么位置；
        int left_or_right = (obj.left_top_x + obj.right_bottom_x) / 2 - rflysim_p.rgb_cnt.x;
        int det_x = 0;
        if (ret < scale_) {
            det_x = int(height * scale_ - width);
            // if(obj.left_top_x - det_x < 0)
            // {  // 目标根据场景布置，目标只有可能被左边的柱子遮挡，如果场景移动了，那就是右边,
            //   //所以这种情况是月边界不再图像内
            //   continue;
            // }
            ROS_INFO("restruct frame ===============");
            if (rflysim_p.is_S)
                ROS_INFO("l_or_r: %d", left_or_right);
            if (!rflysim_p.is_S && mission == Mission::cross_frame1 && left_or_right > 0 ||
                !rflysim_p.is_S && mission == Mission::cross_frame2 && left_or_right < 0 ||
                rflysim_p.is_S && mission == Mission::cross_frame1 && left_or_right < 0 ||
                rflysim_p.is_S && mission == Mission::cross_frame2 && left_or_right > 0
                ) { // 通过当前飞机正在执行的任务与飞机的轨迹形状，再接目标检测的结果判断，该目标是否有效
                continue;
            }
            width += det_x;

        }
        //根据焦距计算相机坐标系的宽中心点位置（x,y,z）
        double        z = wf / width;

        //需要考虑遮挡情况
        double cx = (obj.right_bottom_x + (obj.left_top_x - det_x)) / 2;
        double cy = (obj.right_bottom_y + obj.left_top_y) / 2;
        if (!rflysim_p.is_S && mission == Mission::cross_frame2 || rflysim_p.is_S && mission == Mission::cross_frame1) {
            cx = ((obj.right_bottom_x + det_x) + obj.left_top_x) / 2;
        }

        double xc = (cx - rflysim_p.rgb_cnt.x) * pw;
        double yc = (cy - rflysim_p.rgb_cnt.y) * ph;
        double x = xc * z / rflysim_p.f_rgb;
        double y = yc * z / rflysim_p.f_rgb;
        //至此，一致计算得到目标宽中心位置在相机坐标系里的位置了；

        cnt->x = x;
        cnt->y = y;
        cnt->z = z;

        //    pcl::PointXYZ p;
        //    p.x = x;
        //    p.y = y;
        //    p.z = z;
        //    VisionPointCloud(&p, vis_pub);  // 结合深度图像看看位置是否准确
    }
}


/**
 * @brief 接收位置命令回调函数
 *
 * 该函数用于接收位置命令，并根据当前的任务状态更新接收标志和命令数据。
 * 它直接将接收到的位置命令赋值给内部的cmd变量，同时根据任务状态更新接收标志。
 *
 * @param msg 指向位置命令的常量指针，包含位置和偏航角信息。
 */
void Controller::RecvTra(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    //使用直接赋值，默认认为给飞控的odom数据与给ego-planner的数据是同一坐标系的

    if (send_goal_flag == 1 && mission == Mission::cross_frame1) {
        is_recv_tra = 1;
    }
    if (send_goal_flag == 2 && mission == Mission::cross_frame2) {
        is_recv_tra = 2;
    }
    cmd.position.x = msg->position.x;
    cmd.position.y = msg->position.y;
    cmd.position.z = msg->position.z;
    cmd.yaw = takeoff_yaw;
    cmd.header.stamp = msg->header.stamp;
}



/**
 * @brief 接收并处理本地里程计数据
 *
 * 该函数根据接收到的导航定位数据（Odometry），进行坐标系转换和姿态调整，
 * 以更新发送的本地姿态信息。主要用于将机器人在地图坐标系中的姿态进行调整，
 * 以便于后续处理和显示。
 *
 * @param odom 指向导航定位数据的常量指针，包含机器人的位置和姿态信息
 */
void Controller::RecvLIO(const nav_msgs::Odometry::ConstPtr& odom) {
    send_local_pose.pose.position.x = odom->pose.pose.position.y;
    send_local_pose.pose.position.y = -odom->pose.pose.position.x;
    send_local_pose.pose.position.z = odom->pose.pose.position.z;

    tf2::Quaternion q;
    tf2::fromMsg(odom->pose.pose.orientation, q);
    tf2::Matrix3x3 att(q);
    double         roll, pitch, yaw;
    att.getRPY(roll, pitch, yaw);
    yaw += 1.5707;
    tf2::Quaternion q_;
    q_.setRPY(roll, pitch, yaw);

    // send_local_pose.pose.orientation = odom->pose.pose.orientation;
    send_local_pose.pose.orientation = tf2::toMsg(q_);

    send_local_pose.header.frame_id = "map";
    send_local_pose.header.stamp = ros::Time::now();

    pose_pub.publish(send_local_pose);
    ros::spinOnce();
    ;
}





/**
 * @brief 接收Aruco标记信息的回调函数
 *
 * 该函数在接收到Aruco标记信息时被调用。它的主要作用是判断是否在执行识别Aruco标记的任务，
 * 并在满足一定条件时，更新内部状态以反映Aruco标记的检测结果。
 *
 * @param msg 指向接收到的Aruco标记信息的指针
 */
void Controller::RecvAruco(const common_msgs::Aruco::ConstPtr& msg) {
    if (mission == Mission::recognize_aruco) {
        static int times = 0;
        auto t = ros::Time::now();
        times++;
        if (ros::Time::now().toSec() - t.toSec() > 0.5) {
            times = 0;
            t = ros::Time::now();
        }
        if (times > 5) {
            is_aruco = true;
            aruco = *msg;
            
            //ROS_INFO("Recived Aruco data : %s",msg->data.c_str());
            
        }
    }
}


// 控制器状态变化处理函数
/**
 * @brief 接收Aruco标记信息的回调函数
 *
 * 该函数在接收到Aruco标记信息时被调用。它的主要作用是判断是否在执行识别Aruco标记的任务，
 * 并在满足一定条件时，更新内部状态以反映Aruco标记的检测结果。
 *
 * @param msg 指向接收到的Aruco标记信息的指针
 */

void Controller::StateChange() {

    // ROS_INFO("fcu_p
    // %f,%f,%f",fcu_pose.pose.position.x,fcu_pose.pose.position.x,fcu_pose.pose.position.x);
    if (mission == Mission::cross_frame1) {  //收到到fram1的轨迹，且离目标点很近了
        double dx = fcu_pose.pose.position.x - goal_1.pose.position.x;
        double dy = fcu_pose.pose.position.y - goal_1.pose.position.y;
        double dz = fcu_pose.pose.position.z - goal_1.pose.position.z;
        if (std::abs(dx) < 0.4 && std::abs(dy) < 0.4 && std::abs(dz) < 0.4) {
            ROS_INFO("to cross fram2");
            mission = Mission::cross_frame2;
        }
        ROS_WARN("distance :%f, %f, %f", dx, dy, dz);
        ROS_INFO("fcu_p %f,%f,%f", fcu_pose.pose.position.x,
            fcu_pose.pose.position.y, fcu_pose.pose.position.z);
        ROS_INFO("goal_1  %f,%f,%f", goal_1.pose.position.x, goal_1.pose.position.y,
            goal_1.pose.position.y);
    } else if (mission == Mission::cross_frame2) {  //收到到fram2的轨迹，且离目标点很近了
        double dx = fcu_pose.pose.position.x - goal_2.pose.position.x;
        double dy = fcu_pose.pose.position.y - goal_2.pose.position.y;
        double dz = fcu_pose.pose.position.z - goal_2.pose.position.z;
        ROS_WARN("distance :%f, %f, %f", dx, dy, dz);
        ROS_INFO("fcu_p %f,%f,%f", fcu_pose.pose.position.x,
            fcu_pose.pose.position.y, fcu_pose.pose.position.z);
        ROS_INFO("goal_2 %f,%f,%f", goal_2.pose.position.x, goal_2.pose.position.y,
            goal_2.pose.position.z);
        if (std::abs(dx) < 0.2 && std::abs(dy) < 0.2 && std::abs(dz) < 0.2) {
            ROS_INFO("to recognize_aruco");
            mission = Mission::recognize_aruco;
        }
    } else if (mission == Mission::recognize_aruco && is_aruco) {
        mission = Mission::recognize_H;
        //进入视觉伺服控制
    } else if (mission == Mission::recognize_H && is_H) {
        mission = Mission::land;
    }
    common_msgs::MissionState state;  // 0 1 2 3
    state.header.stamp = ros::Time::now();
    state.task = uint8_t(mission);
    task_state_pub.publish(state);
    ros::spinOnce();

    PrintMission();
}

void Controller::DepthImgToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const cv::Mat* img, const cv::Point2i& left,
    const cv::Point2i& right) {
    //我们需要获得相机坐标系下的点云坐标，等计算中心点，只需要转换一个点坐标即可
    cloud->is_dense = false;
    //  cloud->points.resize(cloud->width * cloud->height);
    //  cv::imshow("depth_frame", *img);
    //  cv::waitKey(0);
    int           dx = left.x - rflysim_p.depth_cnt.x;
    int           dy = left.y - rflysim_p.depth_cnt.y;
    pcl::PointXYZ point;
    for (int row = 0; row < img->rows; row += rflysim_p.depth_down_sample) {
        for (int col = 0; col < img->cols; col += rflysim_p.depth_down_sample) {
            float depth =
                img->at<uint16_t>(row, col) * 0.001f;  // rflysim 精度为0.001
            if (depth > 0 && depth < 7)  //目标不能在7米外，减少计算量
            {
                point.z = depth;
                point.x = (col + dx) * depth / rflysim_p.f_depth;
                point.y = (row + dy) * depth / rflysim_p.f_depth;

                cloud->points.push_back(point);
            }
        };
    }
}


// 控制器中的聚类方法，用于处理点云数据
bool Controller::Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    cv::Point3d* cnt) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices>                 cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.4);  // 设置聚类的欧几里得距离阈值为 50cm
    ec.setMinClusterSize(10);     // 设置一个聚类需要的最小点数
    ec.setMaxClusterSize(10000);  // 设置一个聚类需要的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    // 提取聚类的索引（以点云簇的形式返回）
    ec.extract(cluster_indices);
    double obs_dist = FLT_MAX;
    std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
    // 输出每个聚类的点数和中心点
    pcl::PointCloud<pcl::PointXYZ>::Ptr ret_pc(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it =
        cluster_indices.begin();
        it != cluster_indices.end(); ++it) {  //我们需要对目标应该是点最多的，还可以进一步求出聚类点角点,然后求里飞机最近的目标点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto min_x = FLT_MAX;
        auto max_x = FLT_MIN;
        auto min_y = FLT_MAX;
        auto max_y = FLT_MIN;
        for (std::vector<int>::const_iterator pit = it->indices.begin();
            pit != it->indices.end(); ++pit) {
            if (min_x > cloud->points[*pit].x)
                min_x = cloud->points[*pit].x;
            if (max_x < cloud->points[*pit].x)
                max_x = cloud->points[*pit].x;
            if (min_y > cloud->points[*pit].y)
                min_y = cloud->points[*pit].y;
            if (max_y < cloud->points[*pit].y)
                max_y = cloud->points[*pit].y;
            cluster->points.push_back(
                cloud->points[*pit]);  // 将聚类中的点添加到点云中
        }

        auto width = max_x - min_x;
        auto height = max_y - min_y;
        //    std::cout << " cloud w: " << width << " , h:" << height << std::endl;
        //    if(abs(width - 1.3) > 0.5 || abs(height - 1.3) > 0.5)
        //    {  //计算框的宽和高，以此过滤不符合要求的目标
        //      continue;
        //    }

        cluster->width = cluster->points.size() + 1;
        cluster->height = 1;
        cluster->is_dense = true;

        std::cout << "Cluster size: " << cluster->size() << std::endl;
        //    std::cout << "Cluster center: " << std::endl;
        Eigen::Vector4f centroid;
        auto            ret = pcl::compute3DCentroid(*cluster, centroid);
        if (ret == 0) {
            continue;
        }
        //    std::cout << "x: " << centroid[0] << ", y: " << centroid[1]
        //              << ", z: " << centroid[2] << std::endl;

        if (centroid[2] < obs_dist) {  //最后，距离最近的目标将被选出
            obs_dist = centroid[2];
            cnt->x = centroid[0];
            cnt->y = centroid[1];
            cnt->z = centroid[2];
            pcl::PointXYZ p;
            p.x = cnt->x;
            p.y = cnt->y;
            p.z = cnt->z;
            cluster->points.push_back(p);
            ret_pc->points.swap(cluster->points);
        }
        //    VisionPointCloud(cluster);
        // ret_pc->points.insert(ret_pc->points.begin() +
        // ret_pc->points.size(),cluster->points.begin(),cluster->points.end());
    }
    //  VisionPointCloud(ret_pc);
    cloud->points.swap(ret_pc->points);
    if (obs_dist > 9) {
        ROS_ERROR("not cluster objects");
        return false;
    }
    return true;
}

// 控制器类中的云关键点检测方法
// 该方法用于检测点云中的关键点，输入为点云数据
void Controller::CloudKeyPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 估计法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
        new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);

    // 计算 ISS 角点
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss;
    iss.setInputCloud(cloud);
    iss.setNormals(cloud_normals);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(
        new pcl::PointCloud<pcl::PointXYZ>);
    iss.setSalientRadius(0.05);  // 设置关键点的最小和最大尺度
    iss.setNonMaxRadius(0.05);   // 设置非极大值抑制的搜索半径
    iss.compute(*keypoints);

    // 输出角点数量
    std::cout << "Number of keypoints: " << keypoints->size() << std::endl;
    ;
}

void Controller::PrintMission() {
    switch (mission) {
    case Mission::init:
        ROS_INFO("init");
        break;
    case Mission::takeoff:
        ROS_INFO("takeoff");
        break;
    case Mission::cross_corridor:
        ROS_INFO("cross_corridor");
        break;
    case Mission::cross_frame1:
        ROS_INFO("cross_frame1");
        break;
    case Mission::cross_frame2:
        ROS_INFO("cross_frame2");
        break;
    case Mission::recognize_aruco:
        ROS_INFO("recognize_aruco");
        break;
    case Mission::recognize_H:
        ROS_INFO("recognize_H");
        break;
    case Mission::land:
        ROS_INFO("land");
        break;
    case Mission::end:
        ROS_INFO("end");
        break;
    }
}


/**
 * @brief 将点云数据转换并发布为ROS消息
 *
 * 本函数接收一个pcl::PointCloud<pcl::PointXYZ>类型的点云数据指针，并通过ROS发布出去。
 * 它首先将点云数据转换为ROS消息类型，然后设置消息的帧ID，最后通过给定的发布器发布消息。
 *
 * @param cloud 点云数据指针，包含点云中的所有点
 * @param pub ROS发布器对象，用于发布转换后的点云数据
 */
void Controller::VisionPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const ros::Publisher& pub) {
    sensor_msgs::PointCloud2 data;
    pcl::toROSMsg(*cloud, data);
    data.header.frame_id = "map";
    pub.publish(data);
    ros::spinOnce();
}


/**
 * @brief 将单个点封装到点云并发布
 *
 * 该函数接收一个指向pcl::PointXYZ的指针和一个ROS发布器，
 * 创建一个包含该点的pcl::PointCloud，并调用另一个同名函数发布点云。
 *
 * @param p 指向pcl::PointXYZ的指针，代表要发布的点
 * @param pub ROS发布器，用于发布点云
 */
void Controller::VisionPointCloud(const pcl::PointXYZ* p,
    const ros::Publisher& pub) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.push_back(*p);
    VisionPointCloud(cloud, pub);
}

bool Controller::RePlanReq() {
    //有时候因为遮挡的问题，随着飞机靠近，遮挡解除，目标点中心位置也会发送变化，如果当前目标点无法通过，这时候需要重新发送目标点，

    if (mission == Mission::cross_frame1) {
        double dx = goal_1.pose.position.x - goal_point.pose.position.x;
        double dy = goal_1.pose.position.y - goal_point.pose.position.y;
        double dz = goal_1.pose.position.z - goal_point.pose.position.z;
        if (std::abs(dx) > 0.5 || std::abs(dy) > 0.5 || std::abs(dz) > 0.5) {  //此时需要重新规划路径
            goal_1 = goal_point;
            goal_pub.publish(goal_1);
        }
    } else if (mission == Mission::cross_frame2) {
        double dx = goal_2.pose.position.x - goal_point.pose.position.x;
        double dy = goal_2.pose.position.y - goal_point.pose.position.y;
        double dz = goal_2.pose.position.z - goal_point.pose.position.z;
        if (std::abs(dx) > 0.5 || std::abs(dy) > 0.5 || std::abs(dz) > 0.5) {  //此时需要重新规划路径

            goal_2 = goal_point;
            goal_pub.publish(goal_2);
        }
    }
    ros::spinOnce();
    return true;
}


/**
 * @brief 将点云坐标从深度相机坐标系转换到机体坐标系。
 *
 * 该函数对输入点云中的每个点应用坐标变换，将其从深度相机坐标系转换到机体坐标系。
 * 变换包括旋转和平移，其中旋转由四元数表示，平移由向量表示。
 *
 * @param cloud 指向点云数据的指针，使用PCL（Point Cloud Library）点云数据结构。
 */
void Controller::CoordinateTrans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<double> R = rflysim_p.depth_cam2body_R;
    std::vector<double> T = rflysim_p.depth_cam2body_T;
    //        tf2::Vector3        bc(x, y, z);
    tf2::Quaternion q;
    tf2::convert(fcu_pose.pose.orientation, q);
    tf2::Vector3 t;
    tf2::convert(fcu_pose.pose.position, t);

    tf2::Transform trans;
    trans.setOrigin(t);
    trans.setRotation(q);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        auto         p = cloud->points[i];
        double       x = p.x * R[0] + p.y * R[1] + p.z * R[2] + T[0];
        double       y = p.x * R[3] + p.y * R[4] + p.z * R[5] + T[1];
        double       z = p.x * R[6] + p.y * R[7] + p.z * R[8] + T[2];
        tf2::Vector3 b_p(x, y, z);

        tf2::Vector3 w_p = trans * b_p;
        cloud->points[i].x = w_p.getX();
        cloud->points[i].y = w_p.getY();
        cloud->points[i].z = w_p.getZ();
    }
    //  std::cout << "xxxxxx: " << tmp->points.size() << std::endl;
    ;
}

