# Motor API 复杂场景使用指南

本文档旨在通过一个典型的工业自动化场景（龙门架 + 机械臂 + IO 控制），详细介绍如何使用 `motor_api` 库构建复杂的 EtherCAT 控制应用。

## 1. 场景描述

预设场景包含以下设备：

1.  **双轴龙门架 (Gantry)**
    -   **X轴**：单电机驱动（Slave 0）。
    -   **Y轴**：双电机同步反向驱动（Slave 1, Slave 2）。
2.  **6轴机械臂 (Robot Arm)**
    -   由3个从站驱动，每个从站控制2个关节（Slave 3, 4, 5）。
    -   共6个关节电机 (J1-J6)。
3.  **IO 控制板**
    -   2块独立 IO 板（Slave 6, Slave 7）。
    -   用于控制夹爪、读取传感器等。

## 2. 系统配置 (Configuration)

使用 JSON 配置文件 (`complex_config.json`) 定义网络参数与轴映射。

### 关键配置项说明

-   **unit_per_rev**: 用户单位每圈数值。
    -   对于丝杠（龙门架），设为 `5000.0` (表示 1圈 = 5000um = 5mm)，从而实现微米级控制。
    -   对于旋转关节（机械臂），设为 `360000.0` (表示 1圈 = 360000 mdeg = 360度)，实现 0.001 度级控制。
-   **offset**: 多轴驱动器中，第二轴通常偏移 `2048 (0x800)`。
-   **axis_id**: 全局唯一的逻辑轴 ID，决定 API 调用时的 `axis_idx`。

### 配置文件示例

见同目录下的 `complex_config.json`，它指向包含了自定义 IO 板定义的 `HCFAX3E_complex.xml`。

```json
{
  "network": { "eni_path": "motor_api/doc/HCFAX3E_complex.xml" },
  "slaves": [
    { "id": 0, "type": "cia402", "axes": [ { "axis_id": 0, "unit_per_rev": 5000.0 } ] },
    ...
    { "id": 6, "type": "io", "axes": [ { "axis_id": 9, "offset": 0 } ] }
  ]
}
```

### IO 板支持

针对 `INEXBOT-IO-R4` (Slave 6) 和 `F2838x` (Slave 7)，库已内置自适应映射逻辑：
- **Slave 6 (INEXBOT)**: 输入 `0x6000:00` (U32), 输出 `0x7000:01` (U32)。
- **Slave 7 (F2838x)**: 输入 `0x6001:01` (U16), 输出 `0x7001:01` (U16)。

使用 `motor_api_set_io_output` 和 `motor_api_get_io_input` 即可控制，无需关心底层 PDO 差异。

## 3. 编程实现

完整代码见同目录下的 `example_complex.c`。

### 3.1 初始化

使用 `motor_api_create_from_config` 一键加载配置并初始化主站。

```c
struct motor_api_handle *h = NULL;
motor_api_create_from_config("complex_config.json", &h);
```

### 3.2 轴索引映射

建议使用宏定义将逻辑 ID 映射为有意义的名称：

```c
#define AXIS_GANTRY_X   0
#define AXIS_GANTRY_Y1  1
#define AXIS_GANTRY_Y2  2
#define AXIS_IO_1       9
```

### 3.3 运动控制

使用 `motor_api_set_axis_command` 下发位置增量。
**注意**：API 接受的 `step` 参数为配置文件中定义的“用户单位”的整数值。

例如，配置 `unit_per_rev = 5000.0` (um/rev) 时，`step = 10` 表示移动 10um。

```c
/* 移动 10um */
motor_api_set_axis_command(h, AXIS_GANTRY_X, true, 1, 10);
```

### 3.4 多轴同步 (Y轴龙门)

对于龙门架 Y 轴的双电机同步，需要在应用层同时下发指令。如果机械安装为对向安装，则方向需取反。

```c
/* Y1 正向移动 */
motor_api_set_axis_command(h, AXIS_GANTRY_Y1, true, 1, step);
/* Y2 反向移动 (同步) */
motor_api_set_axis_command(h, AXIS_GANTRY_Y2, true, -1, step);
```

### 3.5 IO 控制

库提供了专用 IO 接口：

-   `motor_api_set_io_output(h, axis_idx, value)`: 设置数字输出。
-   `motor_api_get_io_input(h, axis_idx, &value)`: 读取数字输入。

```c
/* 打开夹爪 (Bit 0) */
motor_api_set_io_output(h, AXIS_IO_1, 0x0001);

/* 读取传感器 */
uint32_t val;
motor_api_get_io_input(h, AXIS_IO_1, &val);
```

## 4. 编译与运行

在 `motor_api/build` 目录下，如果已将示例代码加入 CMake (需修改 CMakeLists.txt)，可以直接编译。

或者手动编译：

```bash
gcc -o example_complex ../doc/example_complex.c -I../include -L. -lmotor_api -lpthread -lrt -lm
```

运行：

```bash
sudo ./example_complex ../doc/complex_config.json
```

## 5. 总结

通过合理的 `config.json` 规划，可以将不同类型（单轴、多轴、IO）的 EtherCAT 从站统一映射为线性索引的逻辑轴。结合 `motor_api` 提供的运动与 IO 接口，可以轻松实现复杂的自动化控制逻辑。
