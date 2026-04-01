#ifndef CONTROLBEANEX_H
#define CONTROLBEANEX_H

/* -----------------------------------------------------------------------
名称: 四轴485机械臂C++接口库
所属: 深圳市慧灵科技有限公司
适用: linux[ubuntu], 不适用于window
修改: 26/03/18  dawson
内容: 网络通信[1]/初始设置[12]/查询设置[10]/拖动示教[2]/碰撞设置[3]
      /停止恢复[6]/限位判断[2]/运动判断[3]/运动指令[13]/运动设置[4]
      /第五轴设置[6]/io设置[3]/脉冲夹爪[5]/485夹爪[17]/吸盘设置[1]
      /正逆输出[0]/工具设置[4]/日志设置[0]/存储设置[2]
------------------------------------------------------------------------ */

class ControlBeanEx {
public:
    float x, y, z, goal_angle1, goal_angle2, rotation;
    bool communicate_success, initial_finish, servo_off_flag, move_flag;
    float angle1_after_judge, angle2_after_judge;
    bool isReach_after_judge;
    float real_x, real_y, real_z, real_angle1, real_angle2, real_rotation;
    int tool_hand;
    float Tool_L;
    float Tool_R;
    float base_x, base_y, base_r;
    
    /* ----- linux 新增变量 ------------------------------------------------------------------------------------*/ 
    float world_x, world_y, world_z, world_r;
    float user_x, user_y, user_z, user_r;

    ControlBeanEx();
    ~ControlBeanEx();

    /* ----- 网络通信 [total: 1] ------------------------------------------------------------------------------*/
    //      是否建立连接
    bool    is_connected();

    /* ----- 初始设置 [total: 12] ------------------------------------------------------------------------------*/
    //      初始化
    int     initial(int generation, float z_travel);
    //      设置杆长
    void    set_arm_length(float l1, float l2);
    //      屏蔽某轴
    bool    check_joint(int joint_num,bool state);
    //      设置插补队列长度
    int     set_Interpolation_position_buf_catch_count(int count);
    //      设置环形队形长度
    int     set_robot_buf_size(int bufsize);
    //      位置解锁
    int     unlock_position();
    //      关节回位
    int     joint_home(int joint_num);
    //      设置到位精度
    void    set_allow_distance_at_target_position(float x_distance, float y_distance, float z_distance, float r_distance);
    //      关节监测
    int     start_joint_monitor();
    //      设置抓取释放精度 
    void    set_catch_or_release_accuracy(float accuracy);
    //      清除运动缓存
    void    clear_move_list_buf();
    //      速度限制
    int     limited_speed(bool state);
    //      自动设零
    int     auto_set_z_zero();
    //      关节1限制
    void    set_j1_limit(float j1_deg);
    //      PID参数设置
    int     set_pid_by_id(int num);

    /* ----- 查询设置 [total: 10] -----------------------------------------------------------------------------*/
    //      位置状态查询
    void    get_scara_param();
    //      获取编码器反馈位置
    void    get_robot_real_coor();
    //      获取关节状态
    int     get_joint_state(int joint_num);
    //      获取错误码
    int     get_error_code();
    //      获取关节力矩设置
    int     get_robot_joint_torque_value(int *j1_torque, int *j2_torque, int *j3_torque, int *j4_torque);
    //      获取id
    int     get_card_num();
    //      获取杆1长度
    float   get_arm1_length();
    //      获取杆2长度
    float   get_arm2_length();
    //      读取id
    int     get_robot_id();
    //      读取通讯报错
    float   get_current_comm_error(int joint_num);

    /* ----- 拖动示教 [total: 2] -----------------------------------------------------------------------------*/
    //      开关拖动示教
    bool    set_drag_teach(bool state);
    //      获取示教状态
    bool    get_drag_teach();

    /* ----- 碰撞设置 [total: 3] -----------------------------------------------------------------------------*/
    //      设置协作状态 [碰撞检测]
    bool    set_cooperation_fun_state(bool state);
    //      获取协作状态
    bool    get_cooperation_fun_state();
    //      是否碰撞
    bool    is_collision();

    /* ----- 停止恢复 [total: 6] -----------------------------------------------------------------------------*/
    //      停止运动
    void    stop_move();
    //      停止运动
    void    new_stop_move();
    //      暂停运动
    int     pause_move();
    //      恢复运动
    int     resume_move();
    //      获取急停状态
    int     get_hard_emergency_stop_state();
    //      清除急停状态
    int     clear_hard_emergency_stop();

    /* ----- 限位判断 [total: 2] -----------------------------------------------------------------------------*/
    //      限位判断
    bool    judge_in_range(float x, float y, float z, float ratation);
    //      限位判断
    int     judge_in_range(float angle1, float angle2, float ratation);

    /* ----- 运动判断 [total: 3] -----------------------------------------------------------------------------*/
    //      到位等待
    bool    wait_stop();
    //      是否到位
    bool    is_robot_goto_target();
    //      是否有再运动
    bool    judge_position_gesture(float x, float y);

    /* ----- 运动指令 [total: 13] ----------------------------------------------------------------------------*/
    //      直线运动
    int     movel_xyz(float goal_x, float goal_y, float goal_z, float rotation, float speed);
    //      关节运动 - 过时，不推荐使用
    int     movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);
    //      弧线运动 - 过时，不推荐使用
    int     movej_xyz(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float rough);
    //      弧线运动 - 过时，不推荐使用
    int     movej_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float roughly, int lr);
    //      关节运动
    int     new_movej_angle(float angle1, float angle2, float goal_z, float goal_r, float speed, float rough);
    //      弧线运动
    int     new_movej_xyz_lr(float goal_x, float goal_y, float goal_z, float goal_r, float speed, float roughly, int lr);
    //      单向运动
    int     xyz_move(int direction, float distance, float speed);
    //      单轴运动
    int     single_joint_move(int axis, float distance, float speed);
    //      点动运动
    int     jog_move_2(float x_a1_dir, float y_a2_dir, float z_dir, float r_dir, float speed, int type);
    //      角度运动
    int     set_angle_move(float angle1, float angle2, float z, float rotation,float speed);
    //      单轴运动
    int     single_axis_move(int axis, float distance);
    //      轨迹运动
    int     trail_move(int point_number, float* x, float* y, float* z, float* r, float speed);
    //      弧线运动 - 可选插补方式
    int     set_position_move(float goal_x, float goal_y, float goal_z, float rotation, float speed, float acceleration,int interpolation, int move_mode);

    /* ----- 运动设置 [total: 4] ---------------------------------------------------------------------------*/
    //      手系切换
    int     change_attitude(float speed);
    //      加速度设置
    int		new_set_acc(int j1_max_acc, int j2_max_acc, int j3_max_acc, int j4_max_acc);
    //      设置关节力矩
    int     set_robot_joint_torque_value(int j1_torque, int j2_torque, int j3_torque, int j4_torque);
    //      插补发送
    int     hi_position_send(double j1_deg,double j2_deg,double z_mm,double j4_deg);

    /* ----- 第五轴设置 [total: 6] -------------------------------------------------------------------------*/
    //      获取第五轴位置
    float   get_j5_positon(int type);
    //      设置第五轴零位
    int     j5_motor_zero();
    //      设置第五轴位置
    int     set_j5_motor_pos(float deg, float speed);
    //      获取第五轴参数
    float   get_j5_parameter();
    //      移动第五轴
    int     movej_j5(float j5_pos, float speed);
    //      第五轴联动
    int     movel_xyz_with_joint5(float goal_x, float goal_y, float goal_z, float goal_r, float joint5, float speed);

    /* ----- io设置 [total: 3] ----------------------------------------------------------------------------*/  
    //      设置io输出
    bool    set_digital_out(int io_number, bool value);
    //      获取io输出
    int     get_digital_out(int io_out_num);
    //      获取io输入
    int     get_digital_in(int io_in_number);

    /* ----- 脉冲夹爪 [total: 5] --------------------------------------------------------------------------*/
    //      设置脉冲夹爪夹持距离
    int     set_efg_state(int type, float distance);
    //      获取脉冲夹爪夹持距离
    int     get_efg_state(int type, float* distance);
    //      获取脉冲夹爪夹持距离
    int     get_efg_state(int* type, float* distance);
    //      获取脉冲夹爪夹持距离
    int     get_efg_state_dji(int* type, float* distance);
    //      设置脉冲夹爪夹持距离
    int     set_efg_state_dji(int type, float distance);

    /* ----- 485夹爪 [total: 17] -------------------------------------------------------------------------*/ 
    //      485夹爪初始化设置
    int     com485_initial(int  baudRate);
    //      485发送
    int     com485_send(unsigned char * data,unsigned char len);
    //      485接收
    int     com485_recv(unsigned char* data);
    //      手动初始化
    int     com485_init();
    //      设置旋转速度
    int     com485_set_rotation_speed(float speed);
    //      设置夹持速度
    int     com485_set_clamping_speed(float speed);
    //      设置夹持距离
    int     com485_set_clamping_distance(float distance);
    //      获取夹持距离 
    float   com485_get_clamping_distance();
    //      获取夹持距离
    int     com485_get_clamping_distance(float* clamping);
    //      设置旋转角度
    int     com485_set_rotation_angle(float angle);
    //      设置相对旋转角度
    int     com485_set_relative_rotation_angle(float angle);
    //      获取旋转角度
    float   com485_get_rotation_angle();
    //      获取旋转角度
    int     com485_get_rotation_angle(float* rotation);
    //      获取夹持状态
    int     com485_get_clamping_state();
    //      获取旋转状态
    int     com485_get_rotation_state();
    //      设置夹持电流
    int     com485_set_clamping_current(float current);
    //      设置旋转电流
    int     com485_set_rotation_current(float current);

    /* ----- 吸盘设置 [total: 1] ------------------------------------------------------------------------*/
    //      设置吸盘
    int     com485_suction_cup_control(int type);

    /* ----- 工具设置 [total: 4] ------------------------------------------------------------------------*/
    //      设置工具参数
    int	    set_tool_fun1(float Tool_L,float Tool_R);
    //      设置工具参数
    int	    set_tool_fun2(float p_x, float p_y);
    //      设置工具参数
    int	    set_tool_fun3(float p1_x, float p1_y, float p1_r, float p2_x, float p2_y, float p2_r);
    //      获取工具参数
    int     get_tool(float *tool_length, float *tool_angle);

    /* ----- 存储设置 [total: 4] ------------------------------------------------------------------------*/
    //      读取eeprom
    int     read_efrom(int addr);
    //      写入eeprom
    int     write_eform(int addr, int value); 

private: // 发现类成员变量顺序导致的指针偏移
    void* bean;
    float target_x, target_y, target_z, target_r;
    float x_distance;
    float y_distance;
    float z_distance;
    float r_distance;
};


#endif
