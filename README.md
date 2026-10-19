# unilidar-driver — Unitree L1 激光雷达驱动

自行实现的 [Unitree L1](https://www.unitree.com/products/unilidar)（宇树 4D LiDAR）
ROS2 驱动：收 UDP 报文、解成标准 `PointCloud2` 与 `Imu` 话题。
**不链接官方静态库 `libunitree_lidar_sdk.a`**，报文解析逻辑全部自己实现；
协议字段布局与点云几何公式依据官方开源头文件（BSD-3-Clause）重述，
详见文末[协议来源](#协议来源)。

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
- 91 项断言的离线自检，做过变异测试
- 按报文类型分别跟踪序号：重复包直接丢弃并计数，乱序与丢包分开统计
- QoS 与常见下游对齐（`SensorDataQoS`），避免"明明在发却收不到"

## 快速开始

```bash
# 放到 ROS2 工作空间的 src/ 下
cd ~/ws/src
git clone https://github.com/qi-neo/unilidar-driver
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y   # 按 package.xml 装齐依赖
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

## 三处需要实机确认

**1. IMU 四元数分量顺序。** 固件没有公开文档约定 `quaternion[4]` 的排列，
而官方 ROS2 包自身就是矛盾的：它的 Imu 消息按 `x,y,z,w` 填，TF 广播却按
`w,x,y,z` 填。填错会让姿态整体错乱。本驱动做成参数 `quaternion_order`
（默认 `xyzw`，与官方 Imu 话题一致）。判定方法：把雷达静止水平放置，正确的
顺序下 `w` 应接近 ±1，其余三个分量接近 0。

**2. CRC32 的覆盖范围。** 本实现按**官方实现**（而不是官方注释）取"只覆盖数据段"，
即 `[12, packet_size-12)`：从官方 `libunilidar_sdk2.a` 反汇编
`LidarPacketParser::parseByte`，CRC 循环起点是 `frame+12`、终点是
`frame+packet_size-12`；而官方协议头文件里
`uint32_t crc32; // crc check of head and data` 这句注释与实现相矛盾。
接上真机后第一件要看的指标就是诊断日志里的 `CRC` 计数：**若它持续在涨而
帧尾错误很少，说明该固件的 CRC 规则与官方实现不同**，把 `udp_probe --hex`
的原始包发回来即可定位。

**3. 网络地址。** 宇树 L1 出厂通常配为 `192.168.1.62`，主机侧 `192.168.1.2`，
端口雷达 6101、主机 6201。默认参数是"监听所有网卡 + 不过滤来源"，
先这样确认能收到，再填 `local_ip` 与 `source_ip` 收紧。

另外注意 UDP 接收缓冲：默认请求 8 MiB（`recv_buf_bytes`，内置常量
`kDefaultRecvBufBytes`）。Linux 内核会把请求值翻倍并受 `net.core.rmem_max`
裁剪，**诊断日志打印的是 `getsockopt` 读回的实际生效值**，不看请求值。
若诊断里的 `序号缺口` 持续增长，先把 `recv_buf_bytes` 调到 16 MiB / 32 MiB
再观察；`duplicates` 持续增长则说明链路上有重复包（回环、镜像口或重传），
驱动会直接丢弃重复包，不会重复计入点云。

## 仓库结构

```
unilidar-driver/
├── .github/workflows/
│   └── ci.yml                    CI：双编译器 × sanitizer + UDP 回环 + ROS 编译
├── docs/
│   └── driver-design.md          《如何编写一个激光雷达驱动》思路文档
├── tests/
│   ├── udp_loopback_test.sh      UDP 回环集成测试（udp_receiver 的唯一覆盖）
│   └── send_synthetic_packets.py 回环测试的合成报文发送端
├── src/unilidar_driver/           ROS2 包
│   ├── include/unilidar_driver/     协议、解析、解码、收包（不依赖 ROS）
│   ├── src/                       实现 + unilidar_driver_node.cpp（ROS 薄节点）
│   ├── examples/
│   │   ├── decoder_selftest.cpp   91 项断言的离线自检
│   │   └── udp_probe.cpp          实测定界探针
│   ├── launch/ config/
│   └── README.md                 包的详细说明
└── LICENSE
```

`docs/driver-design.md` 讲的是**方法论**：驱动的职责边界、四层架构、
帧定界的几种做法与推荐组合、各层的具体坑、如何在不接硬件的情况下验证。
文中每个坑都来自本项目的真实开发过程，不是泛泛的最佳实践。

## 持续集成

[![CI](https://github.com/qi-neo/unilidar-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/qi-neo/unilidar-driver/actions/workflows/ci.yml)

`.github/workflows/ci.yml`，四个 job：

| Job | 内容 |
|---|---|
| `deps` | 依赖声明一致性：核对 `CMakeLists.txt` 的 `find_package` 与 `package.xml` 的依赖声明，漏声明即失败。见 `tools/check_deps.py` |
| `core` | clang++ 与 g++ 两种编译器 × 普通 / ASan+UBSan 两种构建，共 4 个组合。`-Werror` 强制零警告，跑 91 项自检，另编译探针确认可用 |
| `udp-loopback` | 真实 UDP 回环集成测试，再叠 ASan/UBSan 跑一遍；并验证探针在收不到包时返回非 0 |
| `ros` | 在官方 `ros:humble-ros-base` 容器里 `rosdep install --from-paths src --ignore-src -r -y` 后 `colcon build --packages-select unilidar_driver`，并确认节点/自检/探针产物可执行；再用越界 `local_port` 启动节点，确认参数校验会 FATAL 并以非 0 退出 |

前三个 job 不需要 ROS：解析核心源码层面不出现 ROS 头文件，一条 `clang++` 命令加一次执行就能覆盖协议、解析、解码三层。`ros` job 专门补上「ROS 节点从未被编译验证」这块缺口——之前只有外部仓库的 CI 顺带编译过同一份代码，本仓库自己是不编译节点的。

回环测试是 `udp_receiver.cpp` 唯一的自动化覆盖：91 项自检直接喂字节给解析器，不经过 socket，而缓冲、超时、来源地址只有真收发才能验证。测试资产在 `tests/`：

```bash
bash tests/udp_loopback_test.sh                                    # 普通构建
EXTRA_CXXFLAGS="-fsanitize=address,undefined -g" \
  bash tests/udp_loopback_test.sh                                  # 叠加 sanitizer
```

**CI 仍未覆盖**：ROS 节点接真实雷达的实机运行（需要硬件与网络配置），
以及四元数顺序、CRC 规则这类只能对着真机确认的项。

## 已验证 / 未验证

本机没有 ROS，也不在车旁。能在 CI 里验证的都由 CI 持续跑，其余如实标注：

| 项目 | 状态 |
|---|---|
| CRC32 算法 | 已对标准测试向量 `123456789` 校验（0xCBF43926） |
| 报文定界、重同步、CRC 篡改、帧尾篡改、长度越界、重复/乱序序号 | 离线自检 91 项断言全过 |
| CRC32 覆盖范围 | 按**官方实现**核对：反汇编 `libunilidar_sdk2.a` 的 `LidarPacketParser::parseByte`，区间为 `[frame+12, frame+packet_size-12)`；并加了"CRC 含帧头必须被拒"的回归断言。官方协议头注释与实现不一致，**真机仍待确认** |
| 点云几何换算 | 退化标定下与解析解逐点比对（仰角 30° 时 z 应为 0.5） |
| 自检自身有效性 | 变异测试：3 处人为破坏全部被对应断言抓住 |
| `udp_receiver` + 解析器 + 探针 | 真实 UDP 回环：7 包收到、6 帧解出、1 帧 CRC 损坏被拒 |
| 解析核心编译 | `-Wall -Wextra -Wpedantic -Werror` 零警告，clang 与 g++ 均过 |
| 内存与未定义行为 | ASan + UBSan 下自检与回环测试均无报告 |
| ROS 节点编译 | 本仓库 CI 新增的 `ros` job 在 `ros:humble-ros-base` 容器里 `rosdep install` + `colcon build`（此前由 [悬架仓 CI](https://github.com/qi-neo/active-suspension-lidar-preview/actions) 顺带编译同一份代码；本仓库这一 job 为新增，首次运行结果待 CI 确认） |
| CMakeLists 与 package.xml 依赖一致 | CI 每次运行核对，漏声明即失败 |
| **ROS 节点实机运行** | **未验证**，需接雷达后按探针流程核对 |
| **接真实雷达** | **未验证**，需按探针流程核对 |

## 协议来源与许可证

字段布局与点云几何公式来自官方开源的 `unitree_lidar_protocol.h` 与
`unitree_lidar_utilities.h`（均属 `unitree_lidar_sdk2`，BSD-3-Clause）。
本仓库按其公开内容自行实现，**运行时不链接、不调用、不 include 官方任何文件**。

需要说清的是，以下两处确实派生自官方而非原创：

| 文件 | 派生自 | 内容 |
|---|---|---|
| `protocol.hpp` | `unitree_lidar_protocol.h` | 报文结构体字段与顺序、报文类型常量、帧头帧尾标识字节 |
| `frame_decoder.cpp` | `unitree_lidar_utilities.h` 的 `parseFromPacketToPointCloud` | 测距值换算、角度迭代、无效点与距离门限判定、双棱镜机构到 XYZ 的投影公式 |

除此之外全部是自研实现：socket 封装、帧定界与重同步、三道越界保护、统计诊断、
ROS 适配层、91 项自检、`udp_probe` 探针、回环测试、方法论文档。
其中 CRC32 是标准 CRC-32/ISO-HDLC，属公开标准算法，不专属任何厂商。

两个派生文件的头部均已按 BSD-3-Clause 第 1 条**保留原始版权声明**，并逐项列出
相对原始实现的改动，可自行比对。完整的许可证文本、派生范围清单与原创部分
清单见 [`NOTICE`](NOTICE)。

许可证分工：本项目原创部分为 MIT（见 [`LICENSE`](LICENSE)），上述两个文件的
派生部分仍受 Unitree Robotics 的 BSD-3-Clause 约束。

相对官方实现的关键差异是**帧定界策略**：本驱动不拿整包长度当判据，
因为官方头文件里的字节数注释与其自身字段并不一致（详见上文）。

## 许可证

原创部分 MIT（见 `LICENSE`），派生部分的第三方义务见 `NOTICE`。

版本变更记录在 [`CHANGELOG.md`](CHANGELOG.md)。
