# unilidar-driver — Unitree L1 激光雷达驱动

自行实现的 [Unitree L1](https://www.unitree.com/products/unilidar)（宇树 4D LiDAR）
ROS2 驱动：收 UDP 报文、解成标准 `PointCloud2` 与 `Imu` 话题。
**不链接官方静态库 `libunitree_lidar_sdk.a`**，报文解析与几何换算全部自己实现。

[驱动编写思路](docs/driver-design.md)

## 为什么单独一个仓库

雷达驱动是独立可复用的资产：凡是宇树 L1 的项目都用得上它，与主动悬架没有必然关系。
把它从 [active-suspension-lidar-preview](https://github.com/qi-neo/active-suspension-lidar-preview)
里拆出来，两边各自演进：

- 本仓库专注"把字节流变成可信数据"；
- 悬架仓库专注"用这些数据做预瞄与控制"，只依赖本仓库发布的话题。

> 两个仓库目前各有一份驱动代码。改动请以本仓库为准，再同步过去；
> 两边 README 均已互相注明关系。

## 特性

- 协议解析核心**不依赖 ROS**，可在任意有 clang 的机器上编译并跑断言
- 报文定界只用「帧尾标识 + CRC32」，不硬编码整包长度
- 配套 `udp_probe` 实测定界探针，5 分钟确认真实包长与 CRC
- 63 项断言的离线自检，做过变异测试
- QoS 与常见下游对齐（`SensorDataQoS`），避免"明明在发却收不到"

## 快速开始

```bash
# 放到 ROS2 工作空间的 src/ 下
cd ~/ws/src
git clone https://github.com/qi-neo/unilidar-driver
cd ~/ws
colcon build --packages-select unilidar_driver
source install/setup.bash

# 接真实雷达：先跑探针确认能收到、真实包长是多少
ros2 run unilidar_driver udp_probe --port 6201 --seconds 5 --hex

# 跑驱动
ros2 launch unilidar_driver unilidar_driver.launch.py
```

默认话题名与官方节点一致，可直接顶替：

| 话题 | 消息类型 |
|---|---|
| `/unilidar/cloud` | `sensor_msgs/PointCloud2`（字段 x, y, z, intensity） |
| `/unilidar/imu` | `sensor_msgs/Imu` |

脱离 ROS 的快速自检：

```bash
clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1 -I src/unilidar_driver/include \
  src/unilidar_driver/examples/decoder_selftest.cpp \
  src/unilidar_driver/src/protocol.cpp src/unilidar_driver/src/packet_parser.cpp \
  src/unilidar_driver/src/frame_decoder.cpp src/unilidar_driver/src/udp_receiver.cpp \
  -o /tmp/decoder_selftest && /tmp/decoder_selftest
```

## 上手第一步：先确认包长

官方协议头文件里的字节数注释与它自己的字段定义并不一致：

| 项 | 注释写的 | 按字段实算 |
|---|---|---|
| 3D 点云整包 | 1036 B | 1044 B |
| IMU 整包 | 156 B | 80 B |

所以本驱动**不拿整包长度当判据**，只用「帧尾标识 + CRC32」定界
（CRC32 是 32 位校验，随机数据对齐的概率约 2⁻³²，足够可靠）。
长度只用来做诊断提示。接上雷达先用 `udp_probe` 核对实际情况，
若真实长度不是 1044，把探针结果反馈回来核对固件版本即可。

## 两处需要实机确认

**1. IMU 四元数分量顺序。** 固件没有公开文档约定 `quaternion[4]` 的排列，
而官方 ROS2 包自身就是矛盾的：它的 Imu 消息按 `x,y,z,w` 填，TF 广播却按
`w,x,y,z` 填。填错会让姿态整体错乱。本驱动做成参数 `quaternion_order`
（`wxyz` 或 `xyzw`）。判定方法：把雷达静止水平放置，正确的顺序下
`w` 应接近 ±1，其余三个分量接近 0。

**2. 网络地址。** 宇树 L1 出厂通常配为 `192.168.1.62`，主机侧 `192.168.1.2`，
端口雷达 6101、主机 6201。默认参数是"监听所有网卡 + 不过滤来源"，
先这样确认能收到，再填 `local_ip` 与 `source_ip` 收紧。

## 仓库结构

```
unilidar-driver/
├── docs/
│   └── driver-design.md      《如何编写一个激光雷达驱动》思路文档
├── src/unilidar_driver/       ROS2 包
│   ├── include/unilidar_driver/   协议、解析、解码、收包（不依赖 ROS）
│   ├── src/                       实现 + unilidar_driver_node.cpp（ROS 薄节点）
│   ├── examples/
│   │   ├── decoder_selftest.cpp   63 项断言的离线自检
│   │   └── udp_probe.cpp          实测定界探针
│   ├── launch/ config/
│   └── README.md                 包的详细说明
└── LICENSE
```

`docs/driver-design.md` 讲的是**方法论**：驱动的职责边界、四层架构、
帧定界的几种做法与推荐组合、各层的具体坑、如何在不接硬件的情况下验证。
文中每个坑都来自本项目的真实开发过程，不是泛泛的最佳实践。

## 已验证 / 未验证

本机没有 ROS，也不在车旁，验证范围如下，不夸大：

| 项目 | 状态 |
|---|---|
| CRC32 算法 | 已对标准测试向量 `123456789` 校验（0xCBF43926） |
| 报文定界、重同步、CRC 篡改、帧尾篡改、长度越界 | 离线自检 63 项断言全过 |
| 点云几何换算 | 退化标定下与解析解逐点比对（仰角 30° 时 z 应为 0.5） |
| 自检自身有效性 | 变异测试：3 处人为破坏全部被对应断言抓住 |
| `udp_receiver` + 解析器 + 探针 | 真实 UDP 回环：7 包收到、6 帧解出、1 帧 CRC 损坏被拒 |
| 解析核心编译 | `-Wall -Wextra -Wpedantic` 零警告 |
| **ROS 节点编译运行** | **未验证**，需在装 ROS 的机器 `colcon build` |
| **接真实雷达** | **未验证**，需按探针流程核对 |

## 协议来源

字段布局与点云几何公式参考官方开源的 `unitree_lidar_protocol.h` 与
`unitree_lidar_utilities.h`（均属 `unitree_lidar_sdk2`，BSD-3-Clause），
本仓库按其公开内容自行重述，**运行时不链接、不调用、不 include 官方任何文件**。

需要说清的是，这两处确实来自官方而非我原创：

| 内容 | 来源 |
|---|---|
| 报文结构体字段布局（`protocol.hpp`） | 按官方公开字段重述，改为 1 字节打包 |
| 点云 x/y/z 几何公式（`frame_decoder.cpp`） | 取自官方 `parseFromPacketToPointCloud` |

除此之外全部是自研实现：socket 封装、帧定界与重同步、越界保护、统计诊断、
ROS 适配层、63 项自检、`udp_probe` 探针。协议字段布局和双棱镜几何属于
"设备通信格式的规格"而非"官方的实现"——同一种硬件只有一种正确解释方式，
BSD-3-Clause 也允许这样使用。

相对官方实现的关键差异是**帧定界策略**：本驱动不拿整包长度当判据，
因为官方头文件里的字节数注释与其自身字段并不一致（详见上文）。

## 许可证

MIT
