# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。

## [未发布]

对应 1.1.x。主线是"把协议里照注释抄来的假设换成对着官方实现核对过的结论"，
并把收包路径的健壮性补齐：缓冲可配、序号异常可分辨、参数非法直接失败；
再补上逐点 `time` / `ring`，为下游运动去畸变打基础；最后补上实物默认的
串口接收路径（此前只有 UDP）。

### 测试（C1 覆盖率补齐）

对着 gcov 报告补盲区，而不是凭感觉补用例：`g++ --coverage` 编出全部自检 →
跑完 → `gcov` 出行级报告 → 每条未覆盖行映射回"什么外部输入能走到这里" →
补行为可观测的断言。新增自检段落 `[11] 覆盖率补齐`（51 项断言，
报文自检 151 → **202**），补掉 `packet_parser.cpp` 与 `frame_decoder.cpp`
共 17 行盲区，两个文件行覆盖到 100%：

| 盲区 | 对应源行 | 现场 |
|---|---|---|
| `PacketParser::reset()` 整个方法无调用方 | packet_parser 13-17 | 复位漏清序号状态时，同序号帧会被误判重复包而整包丢弃 |
| `append()` 累积超限（单批不超、累加超） | packet_parser 28-33 | 长期解不出合法帧时缓冲只涨不清；且新批次不应被连坐丢弃 |
| 半截帧头 + 前置垃圾的重同步 | packet_parser 67-70 | 帧头标识已到、12 字节头没凑齐时，垃圾不清会干扰后续拼接 |
| 点数组越出报文尾部 | frame_decoder 133-135 | `point_num` 合法但报文被截断，不拦会读到零填充的假点 |
| 报文自带的距离门限 | frame_decoder 184-186 | 帧内 `range_min/max` 与用户门限是两回事，统计必须分开 |
| IMU 帧短于最小读取长度 | frame_decoder 254-256 | 差 1 字节读不到 `linear_acceleration`，不拦会解出零值采样 |

六个分支全是"外部数据异常"路径——报文被截断、帧头被拆散、缓冲被顶爆。
正常数据永远走不到，而真机上恰恰是这些路径决定驱动会不会越界读、
会不会把内存吃光。

**变异验证（六个变异体全部被新断言抓住，且只抓目标断言、不误伤）：**

| 变异体（改坏什么） | 被抓断言数 |
|---|---|
| M1 `reset()` 不调 `seq_state_.clear()` | 3 |
| M2 `append()` 删掉累积超限分支 | 2 |
| M3 `pull()` 删掉半帧头前的垃圾清理 | 2 |
| M4 `decodePointCloud` 删掉点数组边界检查 | 3 |
| M5 `decodePointCloud` 删掉帧内距离门限 | 4 |
| M6 `decodeImu` 删掉最小长度检查 | 3 |

### 测试（A2 节点实时性追加验证）

同一套 gcov 流程用在主循环上：`driver_loop.cpp` 查出 21 行未覆盖，全部是
异常路径——读错误后的恢复、单轮读满上限、整帧被帧内门限滤空后的 pending
复位、`dropPendingScans()` 的统计复位语义、重复包的端到端交付。新增自检
段落 `[4] 追加验证`（37 项断言，节点主循环自检 28 → **65**），独立仓总数
308 → **345**：

| 场景 | 构造 | 坐实的行为 |
|---|---|---|
| 读错误注入 | blobA（20 帧）→ 一次 `read() = -1`（模拟设备拔出）→ blobB（20 帧） | 读错误计数 1 次；已入缓冲的 20 帧照常解出；恢复后不重复计数 |
| 深积压读满 | `recv_chunk=4096`、`max_recv_per_poll=32`，130 帧一次备妥 | 一轮正好读 131072 B；`read_capped_polls==1`；130 帧零丢失 |
| 空云线 | 18 条帧内门限 [10,20] m 外的线，再续 18 条好线 | 不发 0 点帧；`dropped_frame_range==5400`；只发 1 帧 5400 点（pending 复位） |
| 半圈丢弃 | 5 条旧线（stamp 500）→ drop → 18 条新线（stamp 1000） | 只发 1 帧 5400 点；stamp 取 1000 而非陈废的 500；ring 1..18 各 300；幂等 |
| 统计复位 | 跑两轮后 `resetStats()` | 循环统计与分位数归零；**解析器状态不动**（语义锁定）；之后从 0 重计 |
| 重复线端到端 | seq 1..18 + 紧邻重复 seq 18 + 19..36 | `duplicates==1`；36 帧解出；2 帧各 5400 点；无 ring 越界 |

补完未覆盖只剩 1 行：`pollOnce()` 收尾大括号（return 后清理区域的归属，
非可执行代码）。PTY 从端队列自限流，深积压在 PTY 路径下构造不出来，改用
「小 chunk + 大存量」的内存源模拟内核串口缓冲深、录包回放等真实深积压。

**变异验证（六个变异体全部被新断言抓住，且只抓目标断言、不误伤）：**

| 变异体（改坏什么） | 被抓断言数 |
|---|---|
| M1 读错误不计数 | 2 |
| M2 读满不上报 `read_capped_polls` | 2 |
| M3 空云线也发布 | 2 |
| M4 `dropPendingScans()` 变空操作 | 2 |
| M5 `resetStats()` 顺手清解析器 | 1 |
| M6 重复包也交付 | 1 |

**60 s 长时程实测**（2 Mbps 满码率）：fed=15830 out=15830 零丢失、缓冲
峰值 1022 B、p99 0.078 ms（与 2 s 短跑一致，无退化）、RSS 3.6 MB。
双编译器 × 普通/ASan+UBSan 四组合 65 项全过。

### 测试（C2 传输层覆盖率补齐）

C1 补完解析层（`packet_parser`/`frame_decoder` 行覆盖 100%）、A2 补完主循环
之后，`gcov` 查下来驱动核心里只剩两个传输层文件没过覆盖率分析：
`serial_receiver.cpp` 81.69%、`udp_receiver.cpp` 67.14%。未覆盖行逐条归因后
分三类：用户态构造不出的失败（`tcgetattr`/`tcsetattr` 失败）、要耗尽 fd 才
够得着的硬错误（`socket()`/`recvfrom()`）、以及**平台语义**——PTY 无数据时
read 返回 0 而非 EAGAIN（`VMIN=0/VTIME=0` 下的固有限定，`readSome` 的 EAGAIN
分支是防御代码）；设备拔出的语义则被首轮 CI 实测推翻，见下。

- **串口接收自检 64 → 81**（macOS 63 → 80）。新增 `[9] 打开契约与设备拔出`
  17 项：空设备名直接失败且 err 提示「空」；EACCES（`chmod 0` 构造）提示
  「Permission denied（把当前用户加入 dialout 组后重新登录）」，root 下自动
  跳过（权限位被绕过）；设备拔出断言 EOF 语义；fd 失效后 `readSome` 走错误
  分支（返回 -1、err 点名设备与「失败」）——错误分支覆盖靠「从外部关掉驱动
  持有的 fd」模拟句柄变野，为此 `SerialReceiver` 新增 `fd()` 访问器（select/
  poll 型调用方同样用得上，非测试后门）；拔出后 `isOpen` 保持 true、
  `device()` 不变，close 后句柄归零、再读返回 -1。
- **UDP 接收自检新建 38 项**（第五套离线自检）。此前 `udp_receiver.cpp` 只有
  `udp-loopback` job 的真实回环脚本覆盖，而脚本只走 happy path；新建自检补上
  全部分支：打开契约与 `SO_RCVBUF`/`SO_RCVTIMEO` 回读（打印的是 `getsockopt`
  生效值不是请求值）、非法 IP（`300.1.2.3`/`1.2.3`）早退、bind 非本机地址
  （`198.51.100.7`，EADDRNOTAVAIL）报「绑定」失败且点明是哪个 IP、未打开即收
  报「未打开」、超时返回 0 且不污染 err、close
  幂等（`recvBufBytes()` 归零、再收仍 -1）。分工写进头注释：脚本=集成测试
  只走 happy path，自检=单元测试覆盖每条分支含失败路径。

| 文件 | 覆盖前 | 覆盖后 | 剩余未覆盖行归因 |
|---|---|---|---|
| `serial_receiver.cpp` | 81.69% | **88.73%**（macOS 本机） | `tcgetattr`/`tcsetattr` 失败分支（用户态不可构造）；EAGAIN 分支（`VMIN=0/VTIME=0` 下 read 无数据返回 0，防御代码） |
| `udp_receiver.cpp` | 67.14% | **92.86%** | `socket()`/`recvfrom()` 硬错误分支（需耗尽 fd 构造） |

**两轮 CI 翻车与修正（平台语义教训，本轮最重要的经验）**：

1. **设备拔出的 PTY 语义**：原按「Linux 主端关闭后从端 read 返回 -1/EIO」
   断言并写进文档，首轮 CI 在 Ubuntu 24.04 runner（内核 6.8）上两平台实测
   都是 **EOF（返回 0、err 为空）**——4 个核心库组合与 ROS job 当场挂掉。
   修正：删掉 `#ifdef __APPLE__` 分平台断言，统一按 EOF 断言（readSome
   无法区分 EOF 与暂时无数据，都是 0，POSIX read 在 `VMIN=0` 下的固有限定；
   真机 USB 串口拔出两平台都是错误返回，EOF 是 PTY 独有的仿真语义），错误
   分支的覆盖改用 fd 失效场景补上。
2. **同端口二次 bind**：原按「SO_REUSEADDR 下二次 bind 报 EADDRINUSE」断言，
   二轮 CI 在 Linux runner 上挂掉——**Linux 上两个都设了 SO_REUSEADDR 的
   UDP socket 允许绑同一端口**（多进程负载均衡语义），macOS 才报 EADDRINUSE。
   修正：换成 bind 非本机地址（`198.51.100.7` → EADDRNOTAVAIL）——两平台
   行为一致，且「local_ip 填了不在这台机器上的 IP」真机上比端口被占更常见。

两次翻车是同一个教训：**平台语义不能凭文献/记忆断言，要以目标 CI 环境的
实测为准；实测与预期不符时，改的是断言和文档，不是事实。**踩坑记录已写进
自检注释、设计文档第 10 条与本条目。

**变异验证（十一个变异体全部被新断言抓住，且只抓目标断言、不误伤）：**

| 变异体（改坏什么） | 被抓断言数 |
|---|---|
| S1 `open()` 删掉空设备名校验 | 1 |
| S2 `open()` 删掉 EACCES 的 dialout 提示 | 1 |
| S3 `readSome()` 删掉「未打开」的 err 填充 | 1 |
| U1 `open()` 删掉非法 IP 的早退分支 | 5 |
| U2 `open()` bind 失败仍返回 true | 1 |
| U3 bind 失败提示里的「绑定」改名 | 2 |
| U4 `receive()` 删掉 src_ip 回填 | 1 |
| U5 `receive()` 超时改为返回 -1 | 1 |
| U6 `close()` 不清 `recv_buf_bytes_` | 1 |
| U7 `readSome()` 错误分支删掉 err 填充 | 1（fd 失效场景抓住） |
| U8 `open()` bind 失败的 err 退化成无 IP 的通用文案 | 1（非本机地址场景抓住） |

**60 s 长时程复跑**（2 Mbps 灌入 120 MB）：126,637 帧全部解出零丢失、
`readSome` 调用 136,026 次、缓冲峰值 2,064 B、RSS 峰值 1.7 MB、
`bad_size 59`（= 注入的伪帧头数，全部按预期拒斥）。
双编译器 × 普通/ASan+UBSan 四组合全过（UDP 自检连跑 5 次零抖动——端口竞态
是这类测试的常见假红灯，默认端口 46311 与 udp-loopback job 的 46201 不冲突，
另可用 `UDP_SELFTEST_PORT` 覆盖）。CI 新增「编译并运行 38 项 UDP 接收自检」
步骤，串口步骤名随 64 → 81 更新。

独立仓总数 345 → **400**（报文 202 + 串口 81 + UDP 38 + 压力 14 + 主循环 65）。

### 修复（本轮新增）

- **逐点 `time` 用 float32 存绝对 Unix 时间，运动去畸变完全失效（P0，静默）。**
  `PointXYZIT::time` 是 FLOAT32（配合 ROS 的 FLOAT32 字段），但填的是
  **绝对 Unix 秒**（帧时间戳 + `j × time_increment`）。float32 在 1.7e9 量级
  的量化步长是 **128 秒**——而代码注释里写的"约 256 ns"其实是 **double**
  在同一量级的步长，两者被搞混了。后果：一条扫描线 300 个点（总跨度
  299 × 7.7 µs ≈ 2.3 ms）全部塌成同一个值，逐点时间等于没有，下游去畸变
  会算成"整条线同一时刻"，且**不报错、不告警、自检全绿**。实测确认：
  修复前 300 个点的 `time` 只有 1 个可区分值，跨度实测 0.000000 s。
  改为**相对帧起始时刻的偏移秒数**：取值落在 0..0.2 s，float32 量化步长
  约 15 ns，相对 7.7 µs 的点间步长有约 500 倍余量；这也是 PCL / Velodyne /
  Ouster / Livox 的通行约定（PointCloud2 的 `t` 本就是相对 `header.stamp`
  的偏移）。绝对时刻 = `header.stamp + time`。节点侧相应改动：拼帧时按
  "本线时间戳 − 首线时间戳" 把每条线平移到相对整圈起始，保证跨线仍连续
  递增。偏移直接用 `j × time_step` 相减计算，不经"绝对时间相减"，避免
  double 在 1.7e9 量级 2.4e-7 s 的 ulp 污染 7.7 µs 的步长。
- **`packet_parser` 数据段短于 `DataInfo` 时越界读（P0，内存安全）。**
  `packet_size` 的合法下界是 `kHeaderSize + kTailSize = 24`，此时数据段长度
  为 0；而读序号用的 `DataInfo` 有 16 字节。`pull()` 在 CRC/帧尾校验通过后
  无条件 `memcpy(&info, frame + 12, 16)`，**24..39 字节的包会越过报文末尾
  读 4..16 个字节**（ASan 报 container-overflow）。数据来自 UDP/串口，这是
  可被外部输入触发的越界读，且这类短包是协议允许的（官方对数据段长度为 0
  的包连 CRC 都不校验），不能靠 `bad_size` 拒掉。修复：读序号前先确认
  数据段 ≥ `sizeof(DataInfo)`，不足则**跳过序号跟踪但照常交付该帧**。
- **`append()` 的缓冲上限在单批数据过大时形同虚设。** 原逻辑在
  `buf_.size() + len > max_buffered_` 时 `clear()` 后仍然原样 `insert`，
  若 `len` 本身就超过上限，清完再插一样超限。现在单批超限直接整批丢弃
  （保留任何一部分都凑不出完整帧）并计入 `buffer_overflow`。
- **`PullStatus::kNoFrame` 是从未返回的死枚举。** `pull()` 在任何路径上都不会
  返回它（缓冲耗尽统一返回 `kNeedMore`）。留着会让调用方误以为要处理第三种
  状态，已删除。

### 测试（本轮新增）

- 新增自检段落 `[10] P0 缺陷回归`（16 项断言，报文自检 135 → 151）。这两个
  bug 的共同点是**原有 135 项自检全绿却一个都没抓到**：测试只覆盖了"典型
  长度、典型数量级"，缺陷恰好藏在边界上——极短报文、真实 Unix 时间量级。
  所以补的是边界与真实量级，而不是再补几个正常用例。
- 越界读用例特意**不依赖 ASan**：libc++ 的 container-overflow 标注在 `-O1`
  及以上会被优化掉（实测 `-O0` 能报、`-O1`/`-O2` 不报），而 CI 的 sanitizer
  job 正是 `-O1`。改用可观测行为判定——24 字节包没有数据段，`DataInfo::seq`
  （偏移 12..15）恰好落在帧尾的 `crc32` 字段上，把 `crc32` 写成 42 再紧跟
  一个 `seq = 7` 的正常包，越界读会把它记成 `last = 42` 从而误报
  `out_of_order`。修复前该断言失败、修复后为 0，任何优化级别下都成立。
- 已做变异验证：把新自检跑在改动前的代码上，151 项中有 10 项失败，正是上述
  两个缺陷对应的断言；改后代码 151 项全绿。

### 修复

- **`packet_parser` 坏帧后丢整帧。** `pull()` 的扫描游标 `pos` 原在 for 循环
  外声明，`bad_size` / `bad_tail` / `bad_crc` 三个分支在 `erase(pos + 1)`
  丢弃坏字节后直接 `continue`，没有把 `pos` 重置为 0（只有 `duplicate`
  分支重置过）。下一轮 `seek_header` 从**旧位置**开始扫描，而缓冲区内容已
  因 erase 整体前移——扫描起点落在真帧头之前，会把真正的帧头当普通字节
  跳过，表现为**每次遇到坏帧就再丢一整帧**。触发条件正是真实链路最常见的
  形态：一帧好帧前面混着噪声字节。压力自检用「16 B 垃圾 + 12 B 伪帧头 +
  好帧」的组合最先抓到该 bug（此前 135 项报文自检与 64 项串口自检都没覆盖
  到：它们的坏帧用例都是单发坏帧，坏帧后面**不接**好帧）。修复为每轮从 0
  开扫，并删除 duplicate 分支里冗余的重置。回归：修复后 135 + 64 + 14 项
  全绿，四种注入模式（坏帧长度 / 坏帧尾 / 坏 CRC / 坏帧后接好帧）逐一验过。
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
  `cloud_scan_num`(1..65535，点云 ring 字段为 UINT16)、`poll_period_ms`(>=1)、
  `range_min < range_max`、`recv_buf_bytes`(>=0)。

### 新增

- **串口接收：实物默认路径。** L1 的原生通讯接口是 TTL UART，官方 SDK 默认
  经 CH340 类模块以 2,000,000 bps 转 USB 接入（`/dev/ttyUSB0`，需 `dialout`
  权限）。新增 `serial_receiver`（POSIX termios：`O_NONBLOCK` + `cfmakeraw`
  + `CLOCAL` + 关硬件流控 + `VMIN=0/VTIME=0`；打开前 `isatty` 确认是终端
  设备，打开后 `tcflush` 丢掉积压旧字节；读契约与 `udp_receiver` 一致：
  0 = 当前无数据，-1 = 出错）。报文定界完全复用 `packet_parser`——串口是
  字节流，解析器本就按无边界字节流设计，可在任意位置开始接收。波特率是否
  可配置由 `serialBaudSupported()` 提前问出（平台 termios 常量表只到
  38400 是 POSIX 保证，2 Mbps 是各平台扩展）。节点新增 `transport` 参数
  （`serial` 默认 / `udp` 备选）与 `serial_device`、`serial_baud`；诊断日志
  与启动告警按传输层分别给出排障建议（串口路径查设备枚举 / `dialout` /
  波特率 / 线序共地，UDP 路径查 IP 端口与 `SO_RCVBUF`）。
- **`serial_receiver_selftest`：64 项串口自检（Linux；macOS 本机 63 项）。** 用 PTY 主从对
  （`posix_openpt`/`grantpt`/`unlockpt`/`ptsname`）造虚拟串口，测试进程写
  主端、`SerialReceiver` 读从端，走与 `/dev/ttyUSB0` 完全相同的 termios
  代码路径，不需要真雷达也不需要真串口。覆盖：波特率支持表、打开失败
  路径（设备不存在 / 非终端 / 坏波特率 / 未打开即读）、整帧直达、跨写
  拆包、垃圾与伪帧头重同步、无数据契约、一次写入多帧、IMU 帧。CI 的
  `core` job 增加该自检（双编译器 × ASan 共 4 组合），`ros` job 增加
  容器内运行与非法 `transport` 的参数校验断言。数字按 CI 平台（Linux）
  计：波特率表里的 2 Mbps 断言依赖 `B2000000`，它是 Linux 扩展常量，
  macOS 的 termios 没有、跳过该项，故本机跑出来是 63 项。
- **`serial_stress_selftest`：14 项串口压力自检。** PTY 主端按 2 Mbps 向
  `SerialReceiver` 灌入合成报文，掺三种干扰：整秒边界注入 16 B 垃圾 +
  12 B 伪帧头（坏帧后紧跟好帧，专打坏帧恢复路径）、每 2 秒一次 100 ms
  读暂停（制造瞬态背压）、帧必然跨写（PTY 队列远小于一帧）。写端按
  `budget = rate × elapsed` 节流，只提前不落后。断言覆盖：无读写错误、
  读出字节 == 写入 + 注入、灌入帧数 == 解出帧数、点云/IMU 分类一致、
  `bad_crc`/`bad_tail`/`seq_gaps`/`duplicates` 全 0、`bad_size` == 注入的
  伪帧头数、重同步字节 >= 注入垃圾量、结束时 `buffered() == 0`、
  缓冲峰值有界、实际速率 >= 目标 80%、RSS < 64 MB。CI 的 `core` job 与
  `ros` job 均以 `--seconds 3` 运行；本机 60 秒满量版实测：灌入 126,636
  帧全部解出（点云 113,972 / IMU 12,664），实际速率 2.00 Mbps，缓冲峰值
  2,064 B，RSS 峰值 1.7 MB。至此「CH340 需实测」缩小到只剩物理层误码率。
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
- **逐点时间与环号（运动去畸变的前置）。** 核心库新增 `PointXYZIT`
  （x/y/z/intensity/time/ring）与带 `ring` 参数的 `decodePointCloud` 重载，
  旧签名（只出 xyz + intensity）保持不变，旧调用路径与统计语义不动。
  语义逐条对着官方 `unitree_lidar_utilities.h` 的 `parseFromPacketToPointCloud`
  核对：第 j 个点的相对偏移 = `j × time_increment`（`time_relative` 在点循环里
  每点累加一次，被剔除的无效点也占一步）；帧时间戳沿用原有两种模式；
  `ring` 保留官方"1 基、一条扫描线一个环"的约定，由节点按拼帧顺序写入
  （官方 SDK 2.0.2 实际发布二进制里每点 `ring` 写死为 1，只有累加器的
  `ringNum` 计数——本驱动补上了真实线号，拼帧后 `ring ∈ [1, cloud_scan_num]`）。
  ROS 节点 PointCloud2 增加 `time`(FLOAT32，偏移 16) 与 `ring`(UINT16，偏移 20)，
  `point_step` 由 16 调整为 24，偏移 22 起 2 字节 padding 用于保持 4 字节对齐；
  节点诊断新增"逐点时间步长异常"计数与一次性告警。
- **离线自检 91 → 135 项。** 新增 44 项断言：逐点时间严格单调且相邻差等于
  `time_increment`（含 `time_increment = scan_period/point_num` 的对齐情形）、
  被剔除的无效点不压缩时间轴、系统/硬件两种帧时间戳模式下 `time` 基准正确、
  `ring` 显式传入与默认 1（与官方单包逐点值一致）、拼满 18 条扫描线后同一线内
  点序号的 `ring` 分布为 1..18、多线拼接后时间不倒退、
  `time_increment` 为 0/NaN 时退化并计数、新旧接口解出的 xyz/intensity 逐点一致。
- **回环测试补充两项检查。** 探针在收不到任何 UDP 数据时必须返回非 0
  （用一个没人发包的端口跑 1 秒）；汇总里的"重复包/乱序包"必须为 0。

### 新增

- **`driver_loop`：主循环抽出核心库，节点退化成薄壳（28 项自检）。** 节点里
  唯一"有行为"的部分是「收包 → 定界 → 解码 → 拼帧」这一段，原先埋在 ROS 节点
  里，只能靠真机或 ROS 容器观测，实时性与背压永远测不了。现在抽成不依赖 ROS
  的 `DriverLoop`（发布作为回调注入），节点只提供"从哪读"与"怎么发布"。
  于是这段可以在没有 ROS、没有雷达的机器上用 PTY 灌满码率实测，且节点与自检
  跑的是同一份循环代码，不存在两套逻辑各自漂移。配套 `node_loop_selftest`
  28 项断言，三个场景：
  - 正常节奏（2 Mbps + 2 ms 节拍）：零丢失、缓冲有界、没有一轮读满上限、
    拼帧完整、单轮均值/p99 耗时度量。10 秒实测：2,639 帧灌入全部解出，
    均值 0.026 ms、p99 0.065 ms、最大 0.289 ms，缓冲峰值 1,022 B；
  - 慢节拍（8 ms，模拟调度不及时）：仍然零丢失、缓冲有界。**这里有个必须
    说清的事实**：PTY 从端队列（macOS 约 512 B / Linux 约 4 KB）才是唯一的
    积压点且自带上限，单线程生产者写不进去只能等下一拍，所以"消费慢于生产"
    并不会让定界缓冲无限增长——真机上这条队列是内核串口缓冲 + USB 缓冲，
    同样有界只是大得多。所以断言的是"节拍慢 4 倍仍不静默丢帧"，而不是
    去制造一个不会发生的溢出；
  - 超载与恢复：单批 64 KB 灌进 16 KiB 上限的循环，`append()` 整批丢弃并
    计入 `buffer_overflow`（丢帧可见、可诊断，不是静默少几帧），随后逐帧
    继续收，后续 60 帧全部解出、拼帧完整。
- **节点诊断新增实时性与背压一行**：吞吐 Mbps、帧数、轮询次数、单轮均值/
  p99/最大耗时、定界缓冲当前值与峰值、整批丢弃次数、读满单轮上限次数、
  读错误次数。缓冲持续贴近上限即消费跟不上生产，是本节点唯一会静默丢帧的
  路径——打印出来才有得判读。

### 修改

- **测试基建抽取为 `selftest_fixture.hpp`。** `serial_receiver_selftest.cpp`
  里的 PTY 主从对、帧构造、边写边读（`pushAndPump`）、断言计数搬进
  `examples/selftest_fixture.hpp`，与 `serial_stress_selftest.cpp` 共用。
  重构后接收自检断言逐条核对，一条不差（Linux 64 / macOS 63 不变）。
  抽取时发现并利用了 PTY 的一个平台差异：macOS 从端输入队列约 512 B、
  Linux 约 4 KB，都远小于一帧点云 1,044 B——主端必须非阻塞边写边读，
  阻塞写遇队列满会死锁（压力自检开发中实际踩到过）。
- `UdpReceiver::open()` 签名增加 `recv_buf_bytes`，`udp_probe` 与节点同步更新；
  `udp_probe` 汇总新增"重复包/乱序包"两项。
- 点云发布路径改用带 time/ring 的解码重载：`PointCloud2` 字段 6 个、
  `point_step = 24`，`cloud_scan_num` 校验补上 uint16 上限。
- `docs/driver-design.md` 的断言数量、README / 包 README / CI 的说明同步为 135。
- 包 README 的 `quaternion_order` 默认值说明同步为 `xyzw`（与节点、yaml 一致，
  此前文档笔误写成 `wxyz`）。

### 验证

- `decoder_selftest` 135/135 通过（clang++ 与 g++，`-Wall -Wextra -Wpedantic -Werror`）。
- 逐点 time/ring 语义对照官方源码核对：`parseFromPacketToPointCloud` 里
  `time_relative` 每点累加、`point3d.ring = 1`；官方 README 示例
  `cloud size = 5205, ringNum = 18 ... ring = 1`；并反汇编
  `UnitreeLidarReaderImp::runParse` / `appendOnePointCloud` 确认官方单包内
  ring 写死为 1、真实线序只在累加器的 `ringNum` 计数里。
- 真实 UDP 回环 PASSED：7 包收到、6 帧解出、1 帧 CRC 损坏被拒、
  重复包/乱序包均为 0；探针在无数据时返回非 0。
- `python3 tools/check_deps.py` 通过（1 个包依赖声明一致）。
- 压力自检本机 60 秒满量版：灌入 126,636 帧 = 解出 126,636 帧（点云
  113,972 / IMU 12,664），实际速率 2.00 Mbps，`bad_crc 0 · bad_tail 0 ·
  seq_gaps 0 · duplicates 0`，`bad_size 59`（= 59 次伪帧头注入），
  重同步 1,652 B（= 59 × 28 B），缓冲峰值 2,064 B，RSS 峰值 1.7 MB；
  14 项断言全过。g++ / clang++ `-Werror` 与 ASan+UBSan 三种构建形态均过。
- ASan/UBSan（`-fsanitize=address,undefined`）下自检与回环测试均无报告。
- 变异检验：把解析器换回旧实现后，CRC 批次新增断言从 69/69 变成 36 项失败，
  证明新增断言确实能抓住该 bug。
- **未在本地执行**：ROS 节点的 `colcon build` 与参数校验冒烟（本机无 ROS），
  已写进 CI 的 `ros` job，待首次 CI 运行确认；`PointCloud2` 的 time/ring
  字段发布路径只做了字节级手算，并用最小 ROS 桩头文件做了语法/模板实例化检查
  （clang 与 g++ 均通过），未在真实 ROS 环境编译/运行过。

### 仍待实机确认

- CRC 规则（若某固件确实把帧头算进去，诊断日志会显示 `bad_crc` 持续增长）。
- 四元数顺序（静止水平时 `|w|≈1` 的那个分量即 w）。
- **逐点 time/ring**：`time_increment` 与 `scan_period` 的真实取值、`ring`
  的线序是否与拼帧顺序一致、系统时间戳模式下跨线时间抖动，都要接真机用
  rosbag 或 `ros2 topic echo` 核对；若固件 `time_increment` 为 0，诊断日志会
  打印"逐点时间步长异常"。
- 型号/协议对应关系：本实现的 `55 AA 05 0A` + 300 点/片来自官方
  `unilidar_sdk2`，而该 SDK 官方说明是 **L2** 的 SDK；项目名与文档目前写 L1。
  接真机第一件事应是用 `udp_probe --hex` 核对帧头与包长，再统一命名。
- 收包缓冲的实际收益：默认 8 MiB 是按码率与常见调度延迟估算的，
  真机上应该观察 `序号缺口` 是否归零后再决定是否继续调大。
- **CH340 串口物理层误码率。** 协议层已由压力自检全覆盖（2 Mbps 灌入、
  垃圾注入、读背压均零丢失），实物接上后只剩 USB 转串口模块与线材的物理
  误码待实测：诊断日志里 `CRC` 计数持续为 0 即通过；若持续增长，先换
  线/换模块/缩短线长，再怀疑固件 CRC 规则（后者已有对应的排障流程）。

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
