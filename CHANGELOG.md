# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。

## [未发布]

对应 1.1.x。主线是"把协议里照注释抄来的假设换成对着官方实现核对过的结论"，
并把收包路径的健壮性补齐：缓冲可配、序号异常可分辨、参数非法直接失败。

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
- **收包缓冲沿用内核默认值，高码率下会静默丢包。** 雷达约 180 KB/s，而
  Linux 内核默认 `rmem` 只有约 200 KB，`poll_period_ms=2` 的轮询一旦被调度
  延迟（系统忙、下游阻塞）就会在 `recvfrom` 之前溢出，表现为序号缺口持续
  增长但没有任何显式错误。现在默认请求 8 MiB 的 `SO_RCVBUF`，并打印
  `getsockopt` 读回的实际值（内核会翻倍、并受 `net.core.rmem_max` 裁剪，
  请求值不等于生效值）。
- **非法参数被静默回退。** 原先 `cloud_scan_num < 1`、`poll_period_ms < 1`
  只 WARN 后改成 1 继续跑；`quaternion_order` 只要不是 `xyzw` 就当 `wxyz`；
  `range_min >= range_max`、端口越界也照常启动。配置与实际行为不符，排障时
  极难发现。现在统一 `RCLCPP_FATAL` + 抛异常，`main` 捕获后以非 0 退出，
  覆盖 `local_port`(1..65535)、`quaternion_order`(wxyz/xyzw)、
  `cloud_scan_num`(>=1)、`poll_period_ms`(>=1)、`range_min < range_max`、
  `recv_buf_bytes`(>=0)。

### 新增

- **CI 增加 ROS 节点编译。** 新增 `ros` job：在官方 `ros:humble-ros-base`
  容器里安装 `python3-colcon-common-extensions` 与 `rosdep`，
  `rosdep install --from-paths src --ignore-src -r -y` 后
  `colcon build --packages-select unilidar_driver`。此前 CI 只编译不依赖 ROS
  的核心库，节点从未被本仓库编译过。该 job 还确认三个产物可执行，并用越界
  `local_port` 启动节点，断言参数校验会打印"参数非法"且进程非 0 退出。
- **`recv_buf_bytes` 参数。** `UdpReceiver::open()` 增加该参数并
  `setsockopt(SO_RCVBUF)`；`recvBufBytes()` 返回 `getsockopt` 实际生效值
  （未设置/读取失败为 0）。节点参数默认 8 MiB，`config/unilidar_params.yaml`
  已同步；`udp_probe` 启动时也打印实际值。0 表示沿用系统默认。
- **重复/乱序包处理。** `ParseStats` 新增 `duplicates` 与 `out_of_order`，
  解析器改为**按 `packet_type` 分别跟踪 `last_seq`**（点云 102 与 IMU 104
  的序号序列互不相干）：`seq == last` 判定为重复包，CRC/帧尾校验通过后
  整包丢弃、不交付、不计入 `frames_ok`；`seq < last` 判定为乱序，计入
  `out_of_order` 但仍交付；`seq > last + 1` 维持原有 `seq_gaps` 语义。
  原 `seq_discontinuity` 字段由 `out_of_order` 取代（含义更明确，且不再把
  重复包混进去）。
- **节点诊断增强。** 每轮日志打印 `SO_RCVBUF` 实际生效值、重复包、乱序包；
  序号缺口只按"本周期新增"告警一次，并在告警文案里建议调大 `recv_buf_bytes`。
- **离线自检 69 → 91 项。** 新增 22 项断言：重复包被丢弃且不增加
  `frames_ok`、重复包之后的下一帧仍能正常解出（跳过重复包不能错位）、
  乱序包仍交付、`seq_gaps`/`duplicates`/`out_of_order` 三者互不重叠、
  点云与 IMU 按类型分别跟踪序号、uint32 序号回绕
  （`0xFFFFFFFF -> 0`）不被误判为丢包或乱序。
- **回环测试补充两项检查。** 探针在收不到任何 UDP 数据时必须返回非 0
  （用一个没人发包的端口跑 1 秒）；汇总里的"重复包/乱序包"必须为 0。

### 修改

- `UdpReceiver::open()` 签名增加 `recv_buf_bytes`，`udp_probe` 与节点同步更新；
  `udp_probe` 汇总新增"重复包/乱序包"两项。
- `docs/driver-design.md` 的断言数量同步为 91。
- CI `core` job 的自检数量、README / 包 README 的说明同步更新。

### 验证

- `decoder_selftest` 91/91 通过（clang++，`-Wall -Wextra -Wpedantic -Werror`）。
- 真实 UDP 回环 PASSED：7 包收到、6 帧解出、1 帧 CRC 损坏被拒、
  重复包/乱序包均为 0；探针在无数据时返回非 0。
- `python3 tools/check_deps.py` 通过（1 个包依赖声明一致）。
- ASan/UBSan（`-fsanitize=address,undefined`）下自检与回环测试均无报告。
- 变异检验：把解析器换回旧实现后，CRC 批次新增断言从 69/69 变成 36 项失败，
  证明新增断言确实能抓住该 bug。
- **未在本地执行**：ROS 节点的 `colcon build` 与参数校验冒烟（本机无 ROS），
  已写进 CI 的 `ros` job，待首次 CI 运行确认。

### 仍待实机确认

- CRC 规则（若某固件确实把帧头算进去，诊断日志会显示 `bad_crc` 持续增长）。
- 四元数顺序（静止水平时 `|w|≈1` 的那个分量即 w）。
- 型号/协议对应关系：本实现的 `55 AA 05 0A` + 300 点/片来自官方
  `unilidar_sdk2`，而该 SDK 官方说明是 **L2** 的 SDK；项目名与文档目前写 L1。
  接真机第一件事应是用 `udp_probe --hex` 核对帧头与包长，再统一命名。
- 收包缓冲的实际收益：默认 8 MiB 是按码率与常见调度延迟估算的，
  真机上应该观察 `序号缺口` 是否归零后再决定是否继续调大。

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
