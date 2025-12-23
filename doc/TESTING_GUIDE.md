# 硬件功能测试套件使用手册

本文档详细说明如何配置、编译及运行 `motor_api` 硬件功能测试套件。该套件位于 `test/` 目录下，用于验证 EtherCAT 主站通信、运动控制逻辑及 API 稳定性。

## 1. 测试套件结构

| 文件 | 说明 |
| :--- | :--- |
| `test/test_main.c` | 测试入口程序，负责参数解析和报告生成 |
| `test/test_motor_api.c` | 具体测试用例实现（ENI解析、硬件初始化、HTTP接口、运动控制等） |
| `test/test_config.h` | 配置文件，定义测试参数（ENI路径、VendorID、周期时间等） |

## 2. 编译测试程序

测试程序已集成到 CMake 构建系统中。请在项目根目录下执行以下命令：

```bash
# 1. 生成构建系统
cmake -S . -B build

# 2. 编译项目（包含测试程序）
cmake --build build -j
```

编译成功后，将在 `build/` 目录下生成可执行文件 `hardware_test`。

## 3. 运行测试

测试程序支持两种运行模式：**在线模式（Hardware Mode）** 和 **离线模式（Offline Mode）**。

### 3.1 离线模式 (Offline Mode)
**用途**：在没有连接 EtherCAT 硬件的情况下，验证文件解析、参数校验和错误处理逻辑。
**命令**：
```bash
./build/hardware_test --offline
```
**预期输出**：
- ENI XML 解析通过
- 错误处理测试通过
- 硬件初始化及运动控制测试被跳过

### 3.2 在线模式 (Hardware Mode)
**用途**：连接实际硬件后，进行全功能验证（EtherCAT 通信、状态机切换、PDO 数据交换）。
**前置条件**：
1. 确保 EtherCAT 网卡已连接并配置好驱动。
2. 确保 `doc/HCFAX3E.xml` 文件存在且与实际硬件匹配。
3. 需要 `root` 权限（EtherCAT 主站运行要求）。

**命令**：
```bash
sudo ./build/hardware_test
```
**测试流程**：
1. **ENI 解析**：读取 XML 配置。
2. **硬件初始化**：创建主站，扫描从站。
3. **HTTP 接口**：启动 Web 服务（端口 8088），验证服务启停。
4. **运动控制**：
   - 使能伺服（切换至 Op 状态）
   - 运行 CSP 模式 5 秒（发送位置指令）
   - 验证周期数据交换稳定性
5. **资源清理**：停止主站，释放资源。

## 4. 配置文件说明 (`test/test_config.h`)

若测试环境发生变化（如更换驱动器型号或 XML 文件路径），请修改 `test/test_config.h`：

```c
/* 硬件参数配置 */
#define TEST_ENI_PATH       "doc/HCFAX3E.xml"  // ENI 文件路径
#define TEST_CYCLE_US       4000               // 控制周期 (微秒)
#define TEST_VENDOR_ID      0x00000025         // 预期从站厂商 ID
#define TEST_PRODUCT_CODE   0x00000530         // 预期从站产品码

/* 测试控制参数 */
#define TEST_HTTP_PORT      8088               // HTTP 测试端口
#define TEST_MOTION_STEPS   500                // 单次运动步长
#define TEST_DURATION_SEC   5                  // 运动测试持续时间 (秒)
```

修改后需重新编译才能生效。

## 5. 常见问题排查

| 错误信息 | 可能原因 | 解决方法 |
| :--- | :--- | :--- |
| `motor_api_create failed` | EtherCAT 主站未启动或被占用 | 检查网线连接；确保没有其他程序占用主站；尝试重启 EtherCAT 服务 |
| `Vendor ID mismatch` | 实际连接的设备与配置不符 | 使用 `./build/test_read` 查看实际 ID，并更新 `test_config.h` |
| `Assertion failed` | 测试条件未满足 | 查看日志中具体的失败行号，对照源码分析原因 |
| `Permission denied` | 权限不足 | 请使用 `sudo` 运行测试程序 |

## 6. 测试报告示例

测试结束后，终端会输出简要统计报告：

```text
[INFO] ========================================
[INFO] Test Summary:
  Passed: 5
  Failed: 0
[INFO] ========================================
```
若 `Failed` 数为 0，则表示所有测试通过。
