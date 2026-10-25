// 四元数分量顺序（xyzw / wxyz）一键判定的核心逻辑。只依赖 std 与
// frame_decoder.hpp 的 QuatOrder 枚举，不碰 ROS，可离线自检。
//
// 背景：L1 固件四元数分量的排列没有公开文档，官方 ROS2 包自身也不一致
// （Imu 消息按 x,y,z,w 填，同一文件广播 TF 时又按 w,x,y,z 填）。本驱动把
// 它做成 quaternion_order 参数，但默认值只是按官方 Imu 消息的填法猜的
// （见 frame_decoder.hpp 的 DecodeParams 注释）——猜错的后果是发布的姿态
// 整体错位，而且不报错，只有接上雷达摆正了才看得出来。
//
// 判定原理（唯一可靠的判据）：雷达静止且水平放置时，姿态四元数是单位
// 四元数。驱动发布的话题一定是"整理后"的 x,y,z,w 顺序（resolveQuaternion
// 保证，见 frame_decoder.cpp）：
//   - 当前参数设对了  -> 发布值 ~= (0, 0, 0, +-1)，主导分量在第 4 位（w 位）；
//   - 固件是 wxyz 而参数是 xyzw -> 发布值 ~= (+-1, 0, 0, 0)，主导分量在第 1 位；
//   - 固件是 xyzw 而参数是 wxyz -> 发布值 ~= (0, 0, +-1, 0)，主导分量在第 3 位。
// q 与 -q 表示同一旋转（双覆盖），所以主导分量 ~= +1 或 ~= -1 都算。
//
// 只统计低动态样本（|角速度| 低于门限）：运动中的四元数一直在变，均值没有
// 意义。每个样本先归一化再累计——驱动不保证归一化（固件行为未知，decodeImu
// 只做重排不归一化），判据要用相对大小才稳。
//
// 一致性交叉校验：主导分量位置必须能被「当前参数 + 某种固件顺序」解释，
// 解释不了（例如参数是 wxyz 而主导分量落在第 1 位）说明雷达没有水平静止
// 放置，此时给不出结论——打印原始统计量让人工判读，而不是硬给一个答案。
// 判定类工具最害人的行为就是条件不满足时还输出一个看似确定的结论。

#pragma once

#include <cmath>
#include <cstddef>
#include <string>

#include "unilidar_driver/frame_decoder.hpp"  // QuatOrder

namespace unilidar_driver {

enum class QuatVerdict { kXyzw, kWxyz, kUndetermined };

struct QuatVerdictConfig {
  double omega_max{0.05};        // rad/s，低动态门限（静止时应远小于此）
  std::size_t min_samples{30};   // 给出结论所需的最少低动态样本
  double dominant_min{0.7};      // 主导分量 |均值| 下限（单位四元数应 ~=1）
  double other_max{0.3};         // 非主导分量 |均值| 上限
  double zero_tol{0.1};          // 所有分量 |均值| 低于此值按「全零」处理
  double tie_tol{1e-3};          // 两个分量 |均值| 差小于此值算并列，拒绝判定
};

class QuatVerdictAnalyzer {
public:
  QuatVerdictAnalyzer(QuatOrder current_order, const QuatVerdictConfig& cfg = {});

  // 喂入一个样本：q 为驱动发布顺序（x, y, z, w），omega_norm 为角速度模
  // （rad/s）。返回本样本是否计入低动态统计。
  bool feed(const float q[4], float omega_norm);

  std::size_t total() const { return total_; }
  std::size_t lowDynamic() const { return low_; }
  // 通过了动态门限、但四元数全零/非法（归一化失败）的样本数——
  // 固件未初始化时会整窗都是这种样本，需要和「一直在动」区分开
  std::size_t invalid() const { return invalid_; }
  // 低动态样本的各分量均值与 |值| 最大偏离（打印统计量用）
  double mean(int i) const {
    return low_ > 0 ? sum_[i] / static_cast<double>(low_) : 0.0;
  }
  double maxAbs(int i) const { return maxabs_[i]; }

  // |均值| 最大的分量下标（0 基）；并列（差 < tie_tol）返回 -1
  int dominantIndex() const;

  QuatVerdict verdict() const;
  const char* verdictText() const;  // "xyzw" / "wxyz" / "无法判定"
  // 判定成功时是建议动作；失败时是人话原因（调用方原样打印）
  std::string reason() const;

private:
  QuatOrder current_;
  QuatVerdictConfig cfg_;
  std::size_t total_{0};
  std::size_t low_{0};
  std::size_t invalid_{0};
  double sum_[4]{0.0, 0.0, 0.0, 0.0};
  double maxabs_[4]{0.0, 0.0, 0.0, 0.0};
};

// ---------------------------------------------------------------------------
// 实现（头文件实现：逻辑小且纯函数式，自检与工具各自 include 即可，无 ODR 问题）
// ---------------------------------------------------------------------------

inline QuatVerdictAnalyzer::QuatVerdictAnalyzer(QuatOrder current_order,
                                                 const QuatVerdictConfig& cfg)
    : current_(current_order), cfg_(cfg) {}

inline bool QuatVerdictAnalyzer::feed(const float q[4], float omega_norm) {
  ++total_;
  if (!(omega_norm <= cfg_.omega_max)) return false;  // NaN 也走这里（比较为假）

  // 逐个样本归一化：固件是否归一化未知，判据用相对大小
  const double n = std::sqrt(static_cast<double>(q[0]) * q[0] +
                             static_cast<double>(q[1]) * q[1] +
                             static_cast<double>(q[2]) * q[2] +
                             static_cast<double>(q[3]) * q[3]);
  if (!(n > 0.0) || !std::isfinite(n)) {
    ++invalid_;  // 全零/非法四元数：固件未初始化的典型表现
    return false;
  }
  ++low_;
  for (int i = 0; i < 4; ++i) {
    const double v = static_cast<double>(q[i]) / n;
    sum_[i] += v;
    const double a = std::fabs(v);
    if (a > maxabs_[i]) maxabs_[i] = a;
  }
  return true;
}

inline int QuatVerdictAnalyzer::dominantIndex() const {
  if (low_ == 0) return -1;
  double best = 0.0;
  int bi = -1;
  bool tie = false;
  for (int i = 0; i < 4; ++i) {
    const double a = std::fabs(mean(i));
    if (bi < 0 || a > best + cfg_.tie_tol) {
      best = a;
      bi = i;
      tie = false;
    } else if (a > best - cfg_.tie_tol) {
      tie = true;  // 与当前最优差距在容差内，算并列
    }
  }
  return tie ? -1 : bi;
}

inline QuatVerdict QuatVerdictAnalyzer::verdict() const {
  if (low_ < cfg_.min_samples) return QuatVerdict::kUndetermined;
  const int d = dominantIndex();
  if (d < 0) return QuatVerdict::kUndetermined;
  if (std::fabs(mean(d)) < cfg_.dominant_min) return QuatVerdict::kUndetermined;
  for (int i = 0; i < 4; ++i) {
    if (i != d && std::fabs(mean(i)) > cfg_.other_max) return QuatVerdict::kUndetermined;
  }
  // 映射表（推导见文件头注释）：
  //   d==3 -> 发布姿态就是单位四元数，当前参数设对了，固件顺序 == 当前参数；
  //   d==0 且当前 xyzw -> 固件的 w 被原样发布成了 x，固件是 wxyz；
  //   d==2 且当前 wxyz -> 固件的 w 被重排到了 z 位，固件是 xyzw；
  //   其余组合（d==1，或位置与当前设置矛盾）两种顺序都解释不了，拒判。
  if (d == 3) {
    return current_ == QuatOrder::kXyzw ? QuatVerdict::kXyzw : QuatVerdict::kWxyz;
  }
  if (d == 0 && current_ == QuatOrder::kXyzw) return QuatVerdict::kWxyz;
  if (d == 2 && current_ == QuatOrder::kWxyz) return QuatVerdict::kXyzw;
  return QuatVerdict::kUndetermined;
}

inline const char* QuatVerdictAnalyzer::verdictText() const {
  switch (verdict()) {
    case QuatVerdict::kXyzw: return "xyzw";
    case QuatVerdict::kWxyz: return "wxyz";
    default: return "无法判定";
  }
}

inline std::string QuatVerdictAnalyzer::reason() const {
  const QuatVerdict v = verdict();
  if (v == QuatVerdict::kXyzw || v == QuatVerdict::kWxyz) {
    const bool same = (v == QuatVerdict::kXyzw) == (current_ == QuatOrder::kXyzw);
    const std::string want = (v == QuatVerdict::kXyzw) ? "xyzw" : "wxyz";
    if (same) {
      return "当前 quaternion_order 设置正确，固件原始顺序为 " + want + "，无需改动";
    }
    return "固件原始顺序为 " + want +
           "，与当前设置不符：请把 quaternion_order 改为 " + want +
           "（改 config/unilidar_params.yaml 后重启节点，或 "
           "ros2 param set /unilidar_driver_node quaternion_order " + want + "）";
  }

  if (total_ == 0) {
    return "未收到任何 IMU 样本：驱动节点在运行？订阅的话题是 /unilidar/imu？";
  }
  if (low_ < cfg_.min_samples) {
    if (invalid_ == total_) {
      return "全部 " + std::to_string(total_) +
             " 个样本的四元数都是全零/非法值：固件可能尚未初始化"
             "（全零四元数不是有效姿态）";
    }
    return "低动态样本不足（" + std::to_string(low_) + " < " +
           std::to_string(cfg_.min_samples) + "，共 " + std::to_string(total_) +
           " 个样本）：窗口内雷达是否一直在运动？可加长窗口或让雷达静止";
  }
  double max_mean = 0.0;
  for (int i = 0; i < 4; ++i) max_mean = std::max(max_mean, std::fabs(mean(i)));
  if (max_mean < cfg_.zero_tol) {
    return "四元数各分量均值都接近 0：固件可能尚未初始化（全零四元数不是有效姿态）";
  }
  const int d = dominantIndex();
  if (d < 0) {
    return "各分量均值并列，无法区分（并列通常意味着雷达在动或放置面倾斜）";
  }
  if (std::fabs(mean(d)) < cfg_.dominant_min) {
    return "没有单一主导分量（最大 |均值| = " + std::to_string(max_mean) +
           "，需要 >= " + std::to_string(cfg_.dominant_min) +
           "）：雷达是否水平静止放置？单位四元数判据要求有一个分量 ~= +-1";
  }
  for (int i = 0; i < 4; ++i) {
    if (i != d && std::fabs(mean(i)) > cfg_.other_max) {
      return "非主导分量均值过大（第 " + std::to_string(i + 1) + " 个分量 |均值| = " +
             std::to_string(std::fabs(mean(i))) + "）：雷达是否倾斜放置？";
    }
  }
  return "主导分量在第 " + std::to_string(d + 1) +
         " 位，与当前 quaternion_order 设置矛盾（两种顺序都解释不了该姿态）："
         "雷达未水平静止放置，或固件四元数异常。统计量：x=" +
         std::to_string(mean(0)) + " y=" + std::to_string(mean(1)) +
         " z=" + std::to_string(mean(2)) + " w=" + std::to_string(mean(3));
}

}  // namespace unilidar_driver
