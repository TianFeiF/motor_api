# motor_api

## 1. 项目概述

`motor_api` 是一个面向 EtherCAT 伺服驱动器的通用电机控制库（C 语言）。它将 EtherCAT 主站生命周期、ENI（EtherCAT Network Information）解析、PDO 动态映射、DC 同步、CiA-402 状态机推进、CSP/CSV（当前实现以 CSP 为主）周期控制，以及一个轻量级 HTTP 控制/诊断服务封装为可复用 API，便于在上位机侧快速搭建电机控制程序与联调工具。

核心能力：

- 从 ENI XML 读取从站信息（VendorId/ProductCode/Position）以及 Rx/Tx PDO Entry 列表
- 根据 ENI 动态注册 PDO 映射并初始化 EtherCAT 主站/Domain
- 周期性控制入口 `motor_api_run_once()`：接收域数据、推进 CiA-402、写入 CSP 目标
- 内置 HTTP 服务：提供运行指令下发与诊断查询（/status、/diag 等）

主要技术栈与依赖：

- 语言/标准：C99
- 构建：CMake（`cmake_minimum_required(VERSION 3.15)`）
- 依赖库：IgH EtherCAT Master 用户态库（`ecrt.h` / 链接 `ethercat`）、`pthread`
- 运行环境：Linux（需要 EtherCAT 主站与网卡/实时性相关配置）

## 2. 文件结构说明

项目目录结构（以当前仓库为准）：

```text
.
├── CMakeLists.txt              # CMake 构建入口：生成静态/动态库与示例程序
├── README.md                   # 项目文档（本文件）
├── .gitignore                  # 忽略构建产物（build/ 等）
├── include/
│   └── motor_api.h             # 对外 API 头文件：句柄、状态码、ENI 结构、函数声明
├── src/
│   └── motor_api.c             # 库实现：ENI 解析、PDO 注册、DC、状态机、HTTP 服务等
├── examples/
│   ├── example_csp.c           # 示例：按固定周期运行 CSP，支持指定 ENI 路径
│   ├── test_read.c             # 示例：读取并打印 ENI 解析结果与 PDO Entry 列表
│   └── test_eth_initdll.c      # 示例：调用 eth_initDLL() 进行主站初始化与从站枚举
├── doc/
│   └── HCFAX3E.xml             # 示例 ENI 文件（现场需替换为实际 ENI）
└── build/                      # 本地构建目录（建议由 CMake 生成，不建议提交到仓库）
```

关键文件说明：

- `include/motor_api.h`
  - API 入口：`motor_api_create()` / `motor_api_destroy()`
  - 周期控制：`motor_api_run_once()` / `motor_api_set_command()`
  - HTTP 服务：`motor_api_start_http()` / `motor_api_stop_http()`
  - ENI 解析：`motor_api_read_eni()` / `motor_api_free_eni_slaves()`
  - 诊断输出：`motor_api_format_diag_json()`
  - 兼容接口：`eth_initDLL()`
- `src/motor_api.c`
  - 通过 `ecrt.h` 调用 IgH EtherCAT Master API
  - 按 ENI 动态注册 PDO entry，并将域偏移缓存为内部 offsets
  - 周期调用中推进 CiA-402 控制字，并更新目标位置（CSP）
  - 内置 socket 实现简易 HTTP 服务器与 JSON 解析

## 3. 安装与配置指南

### 3.1 环境要求

- Linux 开发环境（推荐 x86_64）
- 工具链：`gcc`（支持 C99）、`cmake >= 3.15`、`make`/`ninja`
- EtherCAT：IgH EtherCAT Master（需要 `ecrt.h` 头文件与 `-lethercat` 链接库）

说明：本项目使用 `ecrt.h`，对应 IgH EtherCAT Master 的用户态接口。你需要确保系统已正确安装并能链接 `ethercat` 库，同时 EtherCAT 主站驱动/内核模块与网卡已按你的现场要求配置完成。

### 3.2 构建（推荐 out-of-source）

在仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
```

构建产物（默认在 `build/`）：

- `libmotor_api.so`：动态库（由 `motor_api_shared` 目标生成，输出名为 `motor_api`）
- `libmotor_api_static.a`：静态库（由 `motor_api_static` 目标生成）
- 示例程序：`example_csp`、`test_read`、`test_eth_initdll`

### 3.3 安装（可选）

将库与头文件安装到系统（默认前缀通常为 `/usr/local`）：

```bash
cmake --install build
```

自定义安装前缀示例：

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/opt/motor_api
cmake --build build -j
cmake --install build
```

### 3.4 环境变量

当前代码版本不依赖必需的环境变量。

运行时你需要自行提供/指定 ENI XML 路径（例如通过示例程序参数传入），并确保 EtherCAT 主站环境已就绪（主站加载、网卡绑定、权限等）。

## 4. 使用说明

### 4.1 作为库集成

头文件：`#include "motor_api.h"`

最小使用流程（伪代码级示例）：

```c
#include <unistd.h>
#include "motor_api.h"

int main(void) {
    const char *eni = "doc/HCFAX3E.xml";
    uint32_t cycle_us = 4000;
    uint16_t slave_count = 0;
    struct motor_api_handle *h = NULL;

    if (motor_api_create(eni, cycle_us, &slave_count, &h) != MA_OK || !h) return 1;

    motor_api_set_command(h, true, 1, 500); /* run=true, dir=1, step=500 */
    while (1) {
        motor_api_run_once(h);
        usleep(cycle_us);
    }

    motor_api_destroy(h);
    return 0;
}
```

注意：

- `motor_api_run_once()` 需要以固定周期调用，并与驱动器插值周期（对象 `0x60C2`）配置保持一致
- 与 EtherCAT 主站交互通常需要足够权限（不少环境下需要 `sudo` 运行示例）

### 4.2 运行示例程序

1) 读取并打印 ENI 内容与 PDO 映射：

```bash
./build/test_read ./doc/HCFAX3E.xml
```

2) 初始化主站并枚举从站信息（示例：超时 5000ms）：

```bash
./build/test_eth_initdll 5000
```

3) 以 CSP 周期控制方式运行示例：

```bash
sudo ./build/example_csp ./doc/HCFAX3E.xml
```

`examples/example_csp.c` 默认使用 `cycle_us=4000`，并会在解析到特定 `ProductCode`（`0x00002406`）时自动将周期调整为 `1000us`（参考 `examples/example_csp.c`）。

### 4.3 HTTP 控制与诊断

调用 `motor_api_start_http(handle, port)` 启动 HTTP 服务后，可通过以下端点访问（详见 `include/motor_api.h` 与 `src/motor_api.c`）：

- `GET /`：健康检查，返回 `motor_api running`
- `GET /status`：返回当前运行参数（`run/dir/step`）
- `GET /diag`：返回诊断 JSON（状态字/模式/跟随误差/错误码等）
- `POST /control`：下发运行指令，Body 示例：`{"direction":"forward","step":500}`
- `POST /stop`：停止运行
- `POST /shutdown`：关闭 HTTP 服务线程

`curl` 示例（假设端口为 8080）：

```bash
curl http://127.0.0.1:8080/
curl http://127.0.0.1:8080/status
curl http://127.0.0.1:8080/diag

curl -X POST http://127.0.0.1:8080/control -d '{"direction":"forward","step":500}'
curl -X POST http://127.0.0.1:8080/stop
curl -X POST http://127.0.0.1:8080/shutdown
```

## 5. 开发指南

### 5.1 代码贡献规范

- 建议使用统一格式：4 空格缩进、保持现有命名风格与错误码返回模式（`ma_status_t`）
- 编译选项启用了 `-Wall -Wextra -Werror`，新增代码需保证无告警
- 对外 API 优先放在 `include/motor_api.h`，实现放在 `src/motor_api.c`

### 5.2 测试方法说明

当前仓库未集成单元测试框架，主要通过示例程序进行联调验证：

- ENI 解析：`./build/test_read <eni.xml>`
- EtherCAT 初始化：`./build/test_eth_initdll [timeout_ms]`
- 周期控制：`sudo ./build/example_csp [eni.xml]`

建议在每次修改后至少执行一次完整构建：

```bash
cmake -S . -B build
cmake --build build -j
```

### 5.3 部署流程

推荐的部署方式是通过 CMake 安装：

```bash
cmake --install build
```

随后在你的应用工程中链接 `motor_api` 并包含安装后的头文件路径。若安装到非标准路径（如 `/opt/motor_api`），需要在编译/运行时显式配置 include/lib 路径以及动态库搜索路径（例如 `LD_LIBRARY_PATH` 或系统 `ld.so.conf.d`）。

## 6. 其他信息

### 6.1 许可证

当前仓库未包含 `LICENSE` 文件，因此尚未对外明确授权条款。若你计划公开发布或供他人使用，建议补充合适的开源许可证并在仓库根目录加入 `LICENSE`。

### 6.2 联系方式

- 建议通过 GitHub Issue/PR 进行问题反馈与贡献：`https://github.com/TianFeiF/motor_api`

### 6.3 已知问题与未来计划

已知问题（基于当前代码）：

- `GET /diag` 的 JSON 输出目前按固定 3 轴格式化（见 `src/motor_api.c` 中 `format_diag()`），当从站数量不为 3 时信息不完整
- HTTP 服务为最小实现：无鉴权、无 TLS、JSON 解析容错有限，建议仅用于内网联调
- `motor_api_destroy()` 不会自动停止 HTTP 线程；若启用了 HTTP 服务，建议在销毁前显式调用 `motor_api_stop_http()`

