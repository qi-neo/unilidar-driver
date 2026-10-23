# 硬件集成 / Hardware Integration

`unilidar-driver` 本身是**纯软件仓库**（宇树 L1 激光雷达的 ROS2 驱动，自研 UDP 协议解析，不链接官方静态库）。这里只放与驱动配套的**物理集成件**说明——L1 的转接与减振安装。

> 状态：设计文档。更完整的硬件系统设计（执行层、电源、急停）见 [`active-suspension-lidar-preview` 的 hardware/](https://github.com/qi-neo/active-suspension-lidar-preview/tree/main/hardware)，本仓库不重复放控制/电源类硬件。

## UniLiDAR L1 转接 / 安装板

把 L1 可靠地装到样机（如主动悬架小车）并稳定供电、出数。

### 1. 供电

- L1 典型工作电压 **12 V**，建议由样机电源板稳压输出（见主动悬架 `hardware/power-emc-board/`）。
- 入口加**反接保护 + TVS 钳位 + 保险丝**；电源与数据地单点连接，避免地环路。

### 2. 数据接口

- L1 通过**以太网**输出点云（本驱动走 UDP 解析）。板子做 RJ45 或端子 breakout，直连 香橙派/计算平台网口；若 L1 用非标准连接器，做小转接板。
- 网络线用屏蔽双绞，屏蔽层在板端单点接屏蔽地。

### 3. 机械安装（关键）

- 雷达相对车体的位姿是**预瞄几何换算的基准**：安装基准面、螺孔位必须在板子上标注清楚，并与驱动里 `mount_*` 安装量参数一致（README 提示默认全 0 为占位，须按实车实测填写）。
- **减振刚性支架**：雷达抖动会直接污染点云与路面高程提取；建议硅胶柱/阻尼垫 + 刚性连接，避免柔性支架引入额外相位。

### 4. 设计要点

- 标注所有接插件 pinout 与线序，避免错插。
- 预留 12 V 电源测试点与网口指示灯，便于联调。

## 引用

- 驱动用法、包长探针：`unilidar-driver` 仓库根 README 与 `docs/driver-design.md`。
- 完整硬件系统（控制/电源/急停/传感器）：[`active-suspension-lidar-preview/hardware`](https://github.com/qi-neo/active-suspension-lidar-preview/tree/main/hardware)。
