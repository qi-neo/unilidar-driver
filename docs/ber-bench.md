# CH340 物理层误码率与长稳 bench（B1）

本文回答一个问题：

> **L1 经 CH340 以 2 Mbps 接到香橙派后，真实物理链路在长时间运行下帧错误率是多少？
> 内存与文件描述符会不会悄悄上涨？**

它补的是 A1 串口压力自检的**物理层**。A1 已证明「驱动在 2 Mbps 字节流下零丢失、
零误判、内存有界」，但那是 PTY 虚拟串口、字节流是测试自己生成的、没有一位是错的。
物理层有没有误码、长跑会不会漏 fd 或涨内存，只有接真雷达才看得到。

> 状态：**未验证（待硬件）**。CI 只跑 `--selftest`（合成流，7 项断言），
> 证明「解析链路计数正确、报告生成路径可用」，不证明物理链路质量。

## 为什么用帧错误率而不是逐位 BER

雷达自发真实点云，没有已知测试图案，无法逐位比对。因此用帧错误率作为物理层质量的
**上界代理**：

```
frame_error_rate = bad_crc / frames_ok
```

一帧里任意一位翻转几乎必然导致 CRC 失败，所以这个比率是逐位 BER 的**上界**，
既不是下界也不是精确值。要拿到真正的逐位 BER，需要**回环已知图案测试**
（见文末）。

## 前置条件

| # | 条件 | 不满足时的后果 |
|---|---|---|
| 1 | Unitree L1 已上电，TTL UART 经 CH340 类模块接主机 | 打不开设备或读不到数据 |
| 2 | 波特率 2,000,000（L1 出厂固件） | 全是 `resync_bytes`，一帧都解不出 |
| 3 | 用户在 `dialout` 组（Linux）或有对应 udev 权限 | `open()` 直接 EACCES |
| 4 | 线材尽量短、双绞、共地；USB 口不与其他大电流设备共用 | 误码率被线材放大，结论不归因于驱动 |
| 5 | 主机不在省电降频状态（香橙派关 cpufreq 省电） | `seq_gaps` 增长会被误判为链路丢帧 |

选设备：`ls -l /dev/serial/by-id/` 比 `/dev/ttyUSB0` 稳定——后者会随插拔顺序漂移。

## 三种跑法

```bash
# 1) 真机长稳（推荐 2 小时起步，越长越能暴露 fd/内存泄漏）
hardware_ber_bench --device /dev/ttyUSB0 --baud 2000000 \
  --duration 7200 --report ber_report.md --csv ber_report.csv

# 2) 回放已有抓包（没有雷达时复现问题）
#    cat /dev/ttyUSB0 > capture.bin    # 先录一段
hardware_ber_bench --input capture.bin --report ber_report.md

# 3) 离线自检（CI 每次跑，不碰硬件）
hardware_ber_bench --selftest --quiet
```

`--quiet` 只抑制屏幕输出，报告文件照常写。

## 报告字段速查

CSV 是两列 `metric,value`，字段与代码一一对应：

| 字段 | 含义 | 健康值 |
|---|---|---|
| `elapsed_s` / `bytes_in` | 时长与总字节 | 与 `--duration`、2 Mbps 相符 |
| `frames_ok` | CRC 通过的帧数 | 点云 + IMU 交错，随时长线性增长 |
| `bad_crc` | CRC 失败帧数 | **关键指标**，见下方判据 |
| `bad_tail` | 帧尾不合法 | 应接近 0；持续增长说明波特率或线材问题 |
| `bad_size` | 长度字段越界 | 应为 0；非 0 说明帧头被误判 |
| `resync_bytes` | 重同步丢弃的字节 | 与 `bad_crc` 同量级属正常 |
| `seq_gaps` | 序号缺口 | 与 `bad_crc` 同量级属正常；远大于 `bad_crc` 说明主机侧丢读 |
| `duplicates` | 重复序号 | 应为 0；非 0 说明链路有回环或镜像 |
| `clouds_decoded` / `imus_decoded` | 成功解出的点云 / IMU 帧 | 与 `frames_ok` 按帧型比例吻合 |
| `decode_fail` | 解码失败 | 应为 0 |
| `rss_peak_mb` | 进程 RSS 峰值 | 长跑应稳定不单调上涨 |
| `cpu_pct` | 进程 CPU 占比 | 2 Mbps 下应远低于单核 |
| `fd_peak` | fd 峰值 | 长跑应恒定；单调上涨即泄漏 |
| `frame_error_rate` | `bad_crc / frames_ok` | 见判据 |

## 判据

- `bad_crc == 0` → 物理层在本次时长内**未观测到**可检测误码，`frame_error_rate = 0`。
- `bad_crc > 0` 且 `seq_gaps ≈ bad_crc` → 是链路误码，照实记录，不算驱动问题；
  先换短线、换 USB 口、加共地复测，把误码归因清楚再谈驱动。
- `seq_gaps >> bad_crc` 而 `bad_crc ≈ 0` → 不是误码，是**主机侧读不及**（背压）。
  查是否被调度抢占或 USB 带宽共享。
- `fd_peak` 随时间单调上涨，或 `rss_peak_mb` 持续爬升 → **判定为缺陷**，必须查。
- 最终结论必须写明：时长、帧数、`bad_crc`、`frame_error_rate`、以及三项资源指标，
  缺任一项都不算跑完。

## 记录模板

| 项目 | 实测 |
|---|---|
| 设备路径 / 模块型号 | |
| 线材长度 / 是否双绞 | |
| 时长 s | |
| frames_ok | |
| bad_crc | |
| frame_error_rate | |
| seq_gaps / duplicates | |
| rss_peak_mb / fd_peak | |
| 结论（通过 / 归因说明） | |

## 边界：bench 不能证明什么

1. **不给逐位 BER。** 见上。
2. **不覆盖 L1 固件的 CRC 覆盖范围是否与官方实现一致。** 那要在诊断日志里看 `CRC`
   计数随 `bad_tail` 的比值，属另一项待确认（见主 README「三处需要实机确认」）。
3. **不覆盖长时间热漂移。** 2 小时只是一次采样；若怀疑温漂，需跨昼夜重复。

## 需要真 BER 时的回环方案（预留，未实现）

把 CH340 的 TX 与 RX 短接（或用两块模块背靠背），发送**已知伪随机图案**，
接收端逐位比对——这样才有真正的逐位 BER 与误码位置分布。此方案不依赖 L1，
属独立工具，尚未实现；上述 bench 已把「报告生成 + 资源采样」这套骨架做好，
补一个图案发生器与比对器即可复用。
