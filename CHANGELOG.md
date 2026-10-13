# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。

## [未发布]

### 修改

- 包内 README 的方法论文档链接由相对路径 `../../../docs/driver-design.md` 改为
  GitHub 绝对链接。原路径只在驱动仓库成立，而悬架仓库保留了一份驱动副本，
  照搬过去会变成死链；改后两边都能打开，且两份副本可逐字节一致。

## [1.0.0] - 2026-10-13

首个正式版本。自行实现 Unitree L1 的 UDP 报文解析与几何换算，发布标准
`PointCloud2` 与 `Imu` 话题，不链接官方静态库 `libunitree_lidar_sdk.a`，
可作为官方 `unitree_lidar_ros2` 节点的 drop-in 替换。

### 新增

- 协议解析核心：`protocol`、`packet_parser`、`frame_decoder`、`udp_receiver`
  四个文件，源码层面不出现任何 ROS 头文件，可在无 ROS 的机器编译与自检。
- ROS 薄节点 `unilidar_driver_node`：参数、收包、发布、TF、诊断，
  按 `SensorDataQoS` 发布以匹配下游订阅方。
- `decoder_selftest`：63 项断言的离线自检，覆盖 CRC32 标准向量、报文定界、
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

已验证：63 项自检、双编译器零警告、ASan+UBSan 无报告、真实 UDP 回环。
**未验证**：ROS 节点编译运行、接真实雷达。详见 README 的验证范围表。
