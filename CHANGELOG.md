# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。

## [未发布]

对应 1.1.x。主线是"把协议里照注释抄来的假设换成对着官方实现核对过的结论"。

### 修复

- **CRC32 覆盖范围错误。** 原实现按 `FrameTail.crc32` 的注释
  （"crc check of head and data"）把 12 字节帧头也算进 CRC，算出
  `crc32(frame, packet_size-12)`。官方实现的真实范围是**只有数据段**
  `[12, packet_size-12)`：反汇编官方 `libunilidar_sdk2.a` 的
  `LidarPacketParser::parseByte`，CRC 循环起点 `frame+12`、终点
  `frame+packet_size-12`，长度 `packet_size-24`。两者不一致，
  真机接上后每一帧都会被判 `bad_crc` 丢弃，点云一帧都发不出来。
  已改为数据段覆盖，并对 `packet_size == 24`（数据段长度为 0）保持官方
  行为：跳过校验。
- **`quaternion_order` 默认值改为 `xyzw`。** 官方 ROS2 节点填 Imu 话题时
  按 `x,y,z,w` 取（`orientation.x = quaternion[0]`, `...w = quaternion[3]`），
  原默认 `wxyz` 会让发布的姿态整体错乱。官方同一文件广播 TF 时又按
  `w,x,y,z` 取，两处互相矛盾，因此该参数保留可配，默认对齐 Imu 话题。

### 新增

- 离线自检 69 项（原 63 项），新增"CRC 覆盖范围"回归：
  分别构造"只覆盖数据段"与"含帧头"的两种帧，断言前者被接受、后者被拒绝，
  并验证 24 字节空包不走 CRC。
- 节点诊断日志的告警文案补充 CRC 排查提示：当"一帧都没解出"且 `bad_crc`
  持续增长时，优先怀疑 CRC 覆盖范围与固件不一致。
- README 的"需要实机确认"从两处扩为三处，把 CRC 覆盖范围写进已知风险清单。

### 验证

- `decoder_selftest` 69/69 通过（clang，`-Wall -Wextra -Wpedantic -Werror`）。
- 变异检验：把解析器换回旧实现后，新自检从 69/69 变成 36 项失败，
  证明新增断言确实能抓住这个 bug。

### 仍待实机确认

- CRC 规则（若某固件确实把帧头算进去，诊断日志会显示 `bad_crc` 持续增长）。
- 四元数顺序（静止水平时 `|w|≈1` 的那个分量即 w）。
- 型号/协议对应关系：本实现的 `55 AA 05 0A` + 300 点/片来自官方
  `unilidar_sdk2`，而该 SDK 官方说明是 **L2** 的 SDK；项目名与文档目前写 L1。
  接真机第一件事应是用 `udp_probe --hex` 核对帧头与包长，再统一命名。

## [1.0.1] - 2026-10-14

依赖修复版本。**若你用的是 1.0.0，请直接换到本版本**——1.0.0 的
`package.xml` 漏声明 `geometry_msgs`，按该 tag 拉下来的包 rosdep 装不全环境。

### 修复

- `package.xml` 补 `geometry_msgs` 依赖。节点用
  `geometry_msgs::msg::TransformStamped` 做 TF 广播，CMakeLists 早已
  `find_package(geometry_msgs REQUIRED)`，但 package.xml 一直没声明。
  **1.0.0 就有这个问题**，本版本修复。开发机上不易发现，因为别的包或
  全局环境往往已经把依赖带上了。

### 新增

- `tools/check_deps.py` 与对应的 CI job：核对 CMakeLists 的 `find_package`
  与 package.xml 的依赖声明是否一致，漏声明即失败。纯文本检查，不需要 ROS。
  就是为防上面那类脱节复发而加的。

### 修改

- 包内 README 的方法论文档链接由相对路径 `../../../docs/driver-design.md` 改为
  GitHub 绝对链接。原路径只在驱动仓库成立，而悬架仓库保留了一份驱动副本，
  照搬过去会变成死链；改后两边都能打开，且两份副本可逐字节一致。
- README 的验证范围表：ROS 节点编译一项由「未验证」改为已通过——
  悬架仓的 ROS CI 在 Humble 干净环境下编译了同一份代码。节点**实机运行**
  与接真实雷达仍标注为未验证。

## [1.0.0] - 2026-10-13

首个正式版本。自行实现 Unitree L1 的 UDP 报文解析与几何换算，发布标准
`PointCloud2` 与 `Imu` 话题，不链接官方静态库 `libunitree_lidar_sdk.a`，
可作为官方 `unitree_lidar_ros2` 节点的 drop-in 替换。

### 新增

- 协议解析核心：`protocol`、`packet_parser`、`frame_decoder`、`udp_receiver`
  四个文件，源码层面不出现任何 ROS 头文件，可在无 ROS 的机器编译与自检。
- ROS 薄节点 `unilidar_driver_node`：参数、收包、发布、TF、诊断，
  按 `SensorDataQoS` 发布以匹配下游订阅方。
- `decoder_selftest`：69 项断言的离线自检，覆盖 CRC32 标准向量、报文定界、
  重同步、各类损坏、流式分片、序号连续性、点云几何、IMU 解码、缓冲上限。
- `udp_probe`：实测定界探针，回答"有没有收到包、真实整包多长、CRC 是否成立"。
- `tests/udp_loopback_test.sh` 与 `tests/send_synthetic_packets.py`：
  真实 UDP 回环集成测试，是 `udp_receiver.cpp` 唯一的自动化覆盖。
- `docs/driver-design.md`：驱动编写方法论。
- CI：clang++ 与 g++ × 普通 / ASan+UBSan 共 4 组合，加 UDP 回环测试。

### 与安全/正确性相关的设计决定

- 帧定界只用「帧尾标识 + CRC32」，不拿整包长度当判据。官方协议头文件的
  字节数注释与其自身字段不一致（3D 点云整包注释 1036 B、按字段实算 1044 B；
  IMU 整包注释 156 B、按字段实算 80 B），硬编码长度会把新固件的包整批判坏。
- IMU 四元数分量顺序做成参数 `quaternion_order`。官方未定死分量排列，
  且官方 ROS2 包内部 Imu 消息按 `xyzw`、TF 广播按 `wxyz`，自相矛盾。
- 点云解码加三道越界保护，并在包长不足时先整体清零再拷入实际长度的字节，
  避免读到上一次调用的栈残留。

### 文档

- `NOTICE`：按 BSD-3-Clause 第 1 条保留官方版权声明，列出派生范围与原创部分清单。
  协议字段布局与点云几何公式派生自 Unitree Robotics 的 BSD-3-Clause 开源头文件。

### 已验证 / 未验证

已验证：69 项自检、双编译器零警告、ASan+UBSan 无报告、真实 UDP 回环。
**未验证**：ROS 节点编译运行、接真实雷达。详见 README 的验证范围表。
