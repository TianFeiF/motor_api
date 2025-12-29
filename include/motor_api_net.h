#ifndef MOTOR_API_NET_H
#define MOTOR_API_NET_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_api_types.h"

/*
 * 本头文件声明 motor_api 的网络相关接口。
 *
 * 设计目标：
 * - 将 HTTP 服务线程、JSON 解析、socket 收发等网络代码从 EtherCAT 核心逻辑中剥离；
 * - 让不需要网络能力的场景可以只关注 EtherCAT API，代码结构更清晰。
 *
 * 注意：
 * - 网络实现为一个轻量级 HTTP 服务（socket + 简易字符串解析），不依赖第三方 HTTP/JSON 库。
 * - 网络线程内部会调用 motor_api_set_command / motor_api_set_axis_command 修改运动指令。
 */

EXTERNFUNC ma_status_t motor_api_start_http(struct motor_api_handle *handle, int port);
EXTERNFUNC ma_status_t motor_api_stop_http(struct motor_api_handle *handle);

EXTERNFUNC ma_status_t motor_api_set_command(struct motor_api_handle *handle,
                                             bool run,
                                             int dir,
                                             int step);

EXTERNFUNC ma_status_t motor_api_set_axis_command(struct motor_api_handle *handle,
                                                  int axis_idx,
                                                  bool run,
                                                  int dir,
                                                  int step);

#ifdef __cplusplus
}
#endif

#endif
