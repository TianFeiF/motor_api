# nex_robot_client.py 中调用的 nrc 接口清单

本文档仅罗列 nex_robot_client.py 中“被直接调用”的 `nrc.*` 接口（不包含该文件中二次封装的 Python 方法）。

## 连接与状态
- `nrc.connect_robot(ip, port)`：连接控制器并返回 `socket_fd`
- `nrc.disconnect_robot(socket_fd)`：断开连接
- `nrc.get_connection_status(socket_fd)`：查询连接状态

## 报警与伺服
- `nrc.clear_error(socket_fd)`：清除错误/报警
- `nrc.set_servo_state(socket_fd, state)`：设置伺服状态
- `nrc.get_servo_state(socket_fd, status_out)`：获取伺服状态（以“返回码 + 出参”形式使用）
- `nrc.set_servo_poweron(socket_fd)`：伺服上电
- `nrc.set_servo_poweroff(socket_fd)`：伺服下电

## 运行状态/速度/模式/坐标系
- `nrc.get_robot_running_state(socket_fd, status_out)`：获取运行状态
- `nrc.set_speed(socket_fd, speed)`：设置速度
- `nrc.get_speed(socket_fd, speed_out)`：获取速度
- `nrc.set_current_coord(socket_fd, coord)`：设置当前坐标系
- `nrc.get_current_coord(socket_fd, coord_out)`：获取当前坐标系
- `nrc.set_current_mode(socket_fd, mode)`：设置当前模式
- `nrc.get_current_mode(socket_fd, mode_out)`：获取当前模式

## 系统变量
- `nrc.set_global_variant(socket_fd, name, value)`：设置全局变量
- `nrc.get_global_variant(socket_fd, name, value_out)`：获取全局变量

## 位姿读取
- `nrc.get_current_position(socket_fd, coord, pos_out)`：获取当前位置
- `nrc.get_current_extra_position(socket_fd, pos_out)`：获取外部轴当前位置

## 单次运动
- `nrc.robot_movej(socket_fd, move_cmd)`：MoveJ 关节运动
- `nrc.robot_movel(socket_fd, move_cmd)`：MoveL 直线运动
- `nrc.robot_extra_movej(socket_fd, move_cmd)`：带外部轴 MoveJ
- `nrc.robot_extra_movel(socket_fd, move_cmd)`：带外部轴 MoveL

## 队列运动
- `nrc.queue_motion_set_status(socket_fd, enabled)`：开关队列运动模式
- `nrc.queue_motion_get_status(socket_fd, status_out)`：查询队列运动模式状态
- `nrc.queue_motion_push_back_moveJ_extra(socket_fd, move_cmd)`：追加带外部轴 MoveJ 到本地队列
- `nrc.queue_motion_push_back_moveL_extra(socket_fd, move_cmd)`：追加带外部轴 MoveL 到本地队列
- `nrc.queue_motion_send_to_controller(socket_fd, size)`：发送本地队列前 `size` 条到控制器执行
- `nrc.queue_motion_stop(socket_fd)`：停止队列运动

## 点动 Jog
- `nrc.robot_start_jogging(socket_fd, axis, direction)`：开始点动
- `nrc.robot_stop_jogging(socket_fd, axis)`：停止点动

## IO
- `nrc.set_digital_output(socket_fd, port, value)`：设置数字输出
- `nrc.get_digital_output(socket_fd, out)`：读取数字输出
- `nrc.get_digital_input(socket_fd, in_)`：读取数字输入
- `nrc.get_analog_input(socket_fd, ain)`：读取模拟输入
- `nrc.set_analog_output(socket_fd, port, value)`：设置模拟输出

## 工具参数
- `nrc.set_tool_hand_number(socket_fd, tool_num)`：设置工具号
- `nrc.get_tool_hand_number(socket_fd, tool_num_out)`：获取工具号
- `nrc.set_tool_hand_param(socket_fd, tool_num, tool_param)`：设置工具参数
- `nrc.get_tool_hand_param(socket_fd, tool_num, tool_param_out)`：获取工具参数

## 作业文件 Job
- `nrc.job_upload_by_file(socket_fd, file_path)`：上传作业文件
- `nrc.job_sync_job_file(socket_fd)`：同步作业文件
- `nrc.job_open(socket_fd, job_name)`：打开作业文件
- `nrc.job_run_times(socket_fd, times)`：设置运行次数
- `nrc.job_run(socket_fd, job_name)`：运行作业文件
- `nrc.job_stop(socket_fd)`：停止作业
- `nrc.job_get_current_line(socket_fd, line_out)`：获取当前行号
- `nrc.job_delete(socket_fd, job_name)`：删除作业文件

## 速度监测
- `nrc.get_curretn_line_speed_and_joint_speed(socket_fd, line_speed_out, joint_speed_out, joint_speed_sync_out)`：获取线速度与关节速度

## 回调
- `nrc.set_receive_error_or_warnning_message_callback(socket_fd, callback)`：注册错误/告警消息回调

## nrc 中用到的构造器/容器类型
- `nrc.VectorDouble()`：double 向量容器，用于位置/速度等出参
- `nrc.VectorInt()`：int 向量容器，用于数字 IO 出参
- `nrc.MoveCmd()`：运动指令结构体
- `nrc.ToolParam()`：工具参数结构体

