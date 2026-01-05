# 配置文件说明文档 (CONFIG_GUIDE)

本文档详细说明了 `motor_api` 库使用的 JSON 配置文件格式。通过配置文件，您可以定义 EtherCAT 网络参数、从站拓扑结构、逻辑轴映射关系以及电机机械参数，而无需修改或重新编译源代码。

## 文件路径

默认配置文件路径通常为项目根目录下的 `config/config.json`。
测试程序 `test_config` 会默认加载该路径，您也可以通过命令行参数指定其他路径。

## 根对象结构

配置文件的根是一个 JSON 对象，包含以下主要部分：

| 字段名 | 类型 | 说明 | 是否必须 |
| :--- | :--- | :--- | :--- |
| `network` | Object | EtherCAT 网络与主站参数配置 | 否 (有默认值) |
| `slaves` | Array | 从站列表及轴映射配置 | 是 |

---

## 1. 网络配置 (network)

用于配置 EtherCAT 主站的基础参数。

```json
"network": {
  "eni_path": "doc/HCFAX3E.xml",
  "cycle_us": 4000
}
```

| 字段名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `eni_path` | String | `"doc/HCFAX3E.xml"` | EtherCAT 网络信息 (ENI) XML 文件的路径。该文件描述了总线上的从站信息。 |
| `cycle_us` | Integer | `4000` | 主站控制周期，单位为微秒 (us)。例如 `4000` 代表 4ms。请确保该值与驱动器的插值周期匹配。 |

---

## 2. 从站与轴配置 (slaves)

`slaves` 是一个数组，数组中的每一项代表一个物理 EtherCAT 从站。
**注意**：数组的顺序并不代表物理连接顺序，但通常建议按物理顺序（Alias/Position）排列以方便管理。逻辑上的绑定是通过 `id` (Slave ID) 确定的。

### 从站对象结构

```json
{
  "id": 0,
  "type": "single_axis",
  "axes": [ ... ]
}
```

| 字段名 | 类型 | 说明 |
| :--- | :--- | :--- |
| `id` | Integer | 物理从站的索引 ID (Slave Index)。通常对应 ENI 文件中定义的物理位置顺序 (0, 1, 2...)。 |
| `type` | String | 从站类型描述。目前支持 `"io"` (IO设备) 或其他任意字符串 (默认为 CiA402 伺服驱动器)。 |
| `axes` | Array | 该从站下挂载的逻辑轴列表。 |

### 轴对象结构 (Axis)

`axes` 数组定义了该物理从站包含的逻辑轴。
- 对于单轴驱动器，数组通常只有 1 个元素。
- 对于多轴驱动器（如二合一），数组会有多个元素。

```json
{
  "axis_id": 0,
  "offset": 0,
  "encoder_res": 131072,
  "gear_ratio": 10.0,
  "unit_per_rev": 360.0
}
```

| 字段名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `axis_id` | Integer | - | **全局逻辑轴 ID**。这是您在程序中控制电机时使用的索引 (0 ~ 31)。每个轴必须拥有唯一的 `axis_id`。 |
| `offset` | Integer | `0` | 对象字典的基地址偏移量。例如：<br>- 单轴驱动器通常为 `0` (0x6040)。<br>- 多轴驱动器的第二轴可能为 `2048` (0x800, 即 0x6840)。 |
| `encoder_res` | Integer | `131072` | 编码器分辨率（单圈脉冲数）。例如 17-bit 编码器为 $2^{17} = 131072$。 |
| `gear_ratio` | Float | `1.0` | 减速比。定义为：电机转数 / 负载转数。例如 10:1 减速机应填 `10.0`。 |
| `unit_per_rev` | Float | `1.0` | 用户单位每圈数值。用于将用户指令转换为脉冲。<br>- 若控制角度 (度)，填 `360.0`。<br>- 若控制直线 (mm)，填丝杠导程 (如 `5.0`)。<br>- 若填 `1.0` 或 `0`，则用户单位 = 电机/减速机负载圈数。 |

### 脉冲换算公式

库内部会自动根据上述参数计算比例因子 (`scale`)：

$$ \text{Scale} = \frac{\text{encoder\_res} \times \text{gear\_ratio}}{\text{unit\_per\_rev}} $$

- **目标位置 (脉冲)** = 用户指令 $\times$ Scale
- **实际位置 (用户单位)** = 驱动器反馈脉冲 / Scale

---

## 完整配置示例

以下示例展示了一个包含 3 个从站的配置：
1. **Slave 0**: 单轴驱动器，带 10:1 减速机，按“度”控制 (Axis 0)。
2. **Slave 1**: 单轴驱动器，直驱，按“度”控制 (Axis 1)。
3. **Slave 2**: IO 模块或特殊驱动器，配置为 Axis 2。

```json
{
  "network": {
    "eni_path": "doc/HCFAX3E.xml",
    "cycle_us": 4000
  },
  "slaves": [
    {
      "id": 0,
      "type": "single_axis",
      "axes": [
        { 
          "axis_id": 0, 
          "encoder_res": 131072, 
          "gear_ratio": 10.0, 
          "unit_per_rev": 360.0 
        }
      ]
    },
    {
      "id": 1,
      "type": "single_axis",
      "axes": [
        { 
          "axis_id": 1, 
          "encoder_res": 131072, 
          "gear_ratio": 1.0, 
          "unit_per_rev": 360.0 
        }
      ]
    },
    {
      "id": 2,
      "type": "single_axis",
      "axes": [
        { 
          "axis_id": 2, 
          "encoder_res": 131072, 
          "gear_ratio": 1.0, 
          "unit_per_rev": 360.0 
        }
      ]
    }
  ]
}
```
