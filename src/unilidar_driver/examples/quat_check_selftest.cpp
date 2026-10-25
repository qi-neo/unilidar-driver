// 四元数分量顺序判定核心（quat_verdict.hpp）的离线自检。
// 不需要 ROS、不需要雷达、不需要网络——IMU 样本全部合成。
//
// 编译与运行（路径相对于仓库根目录）：
//   clang++ -std=c++17 -Wall -Wextra -Wpedantic -O1
//     -I src/unilidar_driver/include
//     src/unilidar_driver/examples/quat_check_selftest.cpp
//     src/unilidar_driver/src/frame_decoder.cpp
//     src/unilidar_driver/src/protocol.cpp
//     -o /tmp/quat_check_selftest
//   /tmp/quat_check_selftest
//
// 场景与 quat_verdict.hpp 头注释里的映射表一一对应：
//   [1] 参数设对：两种固件顺序各自判「无需改动」
//   [2] 参数设错：判出固件原始顺序并给出改参指令
//   [3] q 与 -q 双覆盖：w ~= -1 也是单位四元数
//   [4] 低动态门限：运动样本不计入；边界值语义
//   [5] 样本数边界：恰好 min_samples 判得出来，少一个拒判
//   [6] 拒判场景：倾斜、全零、并列、与当前设置矛盾、主导分量落 y 位
//   [7] 稳健性：噪声、非归一化输入（固件是否归一化未知）
//   [8] 统计量与 API 契约：计数、mean/maxAbs、feed 返回值、空分析器
//
// 样本用 resolveQuaternion 从「固件原始值 + 当前参数」合成发布值，
// 与真链路的形状一致（解码层只重排不归一化，见 frame_decoder.cpp）。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

#include "unilidar_driver/frame_decoder.hpp"
#include "unilidar_driver/quat_verdict.hpp"

using namespace unilidar_driver;

static int g_checks = 0;
static int g_failed = 0;

static void check(bool ok, const std::string& what) {
  ++g_checks;
  if (!ok) {
    ++g_failed;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

// 以「固件原始顺序」的四元数喂 n 个样本：先按当前参数解析成发布顺序
// （与 decodeImu -> resolveQuaternion 同一路径），再喂给分析器。
static void feedRaw(QuatVerdictAnalyzer& a, const float raw[4], QuatOrder current,
                    std::size_t n, float omega = 0.001f) {
  float pub[4];
  resolveQuaternion(raw, current, pub);
  for (std::size_t i = 0; i < n; ++i) {
    a.feed(pub, omega);
  }
}

// 直接以发布顺序喂样本（拒判场景用：构造发布值反常的输入）
static void feedPub(QuatVerdictAnalyzer& a, const float pub[4], std::size_t n,
                    float omega = 0.001f) {
  for (std::size_t i = 0; i < n; ++i) {
    a.feed(pub, omega);
  }
}

static const float kIdentityXyzw[4] = {0.f, 0.f, 0.f, 1.f};  // (x,y,z,w)
static const float kIdentityWxyz[4] = {1.f, 0.f, 0.f, 0.f};  // (w,x,y,z)

// ---------------------------------------------------------------------------
// [1] 参数设对：主导分量在 w 位（第 4 位），判「当前设置正确」
// ---------------------------------------------------------------------------
static void test_setting_correct() {
  std::printf("[1] 参数设对：两种固件顺序各自判无需改动\n");

  {
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kXyzw, "固件 xyzw + 参数 xyzw -> 判 xyzw");
    check(std::string(a.verdictText()) == "xyzw", "verdictText 为 xyzw");
    check(a.reason().find("无需改动") != std::string::npos,
          "原因说明确为无需改动");
    check(a.dominantIndex() == 3, "主导分量在第 4 位（w 位）");
  }
  {
    QuatVerdictAnalyzer a(QuatOrder::kWxyz);
    feedRaw(a, kIdentityWxyz, QuatOrder::kWxyz, 100);
    check(a.verdict() == QuatVerdict::kWxyz, "固件 wxyz + 参数 wxyz -> 判 wxyz");
    check(std::string(a.verdictText()) == "wxyz", "verdictText 为 wxyz");
    check(a.reason().find("无需改动") != std::string::npos,
          "原因说明确为无需改动");
  }
}

// ---------------------------------------------------------------------------
// [2] 参数设错：判出固件原始顺序，并给出改参指令
// ---------------------------------------------------------------------------
static void test_setting_wrong() {
  std::printf("[2] 参数设错：判出固件顺序并给出改参指令\n");

  {
    // 固件 wxyz、参数 xyzw：w 被原样发布成 x，主导分量落到第 1 位
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, kIdentityWxyz, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kWxyz, "固件 wxyz + 参数 xyzw -> 判 wxyz");
    check(a.dominantIndex() == 0, "主导分量在第 1 位");
    check(a.reason().find("改为 wxyz") != std::string::npos,
          "原因里给出改为 wxyz 的指令");
    check(a.reason().find("quaternion_order") != std::string::npos,
          "指令点明参数名 quaternion_order");
  }
  {
    // 固件 xyzw、参数 wxyz：w 被重排到 z 位，主导分量落到第 3 位
    QuatVerdictAnalyzer a(QuatOrder::kWxyz);
    feedRaw(a, kIdentityXyzw, QuatOrder::kWxyz, 100);
    check(a.verdict() == QuatVerdict::kXyzw, "固件 xyzw + 参数 wxyz -> 判 xyzw");
    check(a.dominantIndex() == 2, "主导分量在第 3 位");
    check(a.reason().find("改为 xyzw") != std::string::npos,
          "原因里给出改为 xyzw 的指令");
  }
}

// ---------------------------------------------------------------------------
// [3] q 与 -q 双覆盖：w ~= -1 也是单位四元数（同一旋转的两种表示）
// ---------------------------------------------------------------------------
static void test_sign_dual_cover() {
  std::printf("[3] q 与 -q 双覆盖\n");

  {
    const float raw[4] = {0.f, 0.f, 0.f, -1.f};  // -identity，xyzw
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, raw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kXyzw, "w ~= -1 仍判 xyzw");
    check(a.dominantIndex() == 3, "负 identity 主导分量仍在 w 位");
  }
  {
    const float raw[4] = {-1.f, 0.f, 0.f, 0.f};  // -identity，wxyz
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, raw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kWxyz, "wxyz 负 identity 仍判 wxyz");
  }
}

// ---------------------------------------------------------------------------
// [4] 低动态门限：运动样本不计入统计；边界值按 <= 计入
// ---------------------------------------------------------------------------
static void test_low_dynamic_gate() {
  std::printf("[4] 低动态门限\n");

  {
    // 全程运动：低动态样本为 0，拒判且原因点明样本不足
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 100, 0.5f);
    check(a.total() == 100, "运动样本计入 total");
    check(a.lowDynamic() == 0, "运动样本不计入低动态");
    check(a.verdict() == QuatVerdict::kUndetermined, "全程运动 -> 无法判定");
    check(a.reason().find("低动态样本不足") != std::string::npos,
          "原因点明低动态样本不足");
  }
  {
    // 混合：100 个运动样本夹在 100 个静止样本之间，只用静止样本判定
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 100, 0.001f);
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 100, 0.5f);
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 100, 0.001f);
    check(a.total() == 300, "混合场景 total 为 300");
    check(a.lowDynamic() == 200, "混合场景低动态样本为 200");
    check(a.verdict() == QuatVerdict::kXyzw, "混合场景仍判 xyzw（只用静止样本）");
  }
  {
    // 边界值语义：omega_max = 1.0（1.0f 提升成 double 后精确可表示，
    // 边界判定不受浮点提升影响）。恰好等于门限计入，超出不计入。
    QuatVerdictConfig cfg;
    cfg.omega_max = 1.0;
    QuatVerdictAnalyzer a(QuatOrder::kXyzw, cfg);
    const float q[4] = {0.f, 0.f, 0.f, 1.f};
    check(a.feed(q, 1.0f), "omega 恰好等于门限应计入");
    check(!a.feed(q, 1.001f), "omega 超出门限不应计入");
    check(a.lowDynamic() == 1, "边界场景低动态计数为 1");
  }
}

// ---------------------------------------------------------------------------
// [5] 样本数边界：恰好 min_samples 判得出来，少一个拒判
// ---------------------------------------------------------------------------
static void test_min_samples_boundary() {
  std::printf("[5] 样本数边界\n");

  {
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);  // 默认 min_samples = 30
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 30);
    check(a.verdict() == QuatVerdict::kXyzw, "恰好 30 个低动态样本 -> 判得出");
  }
  {
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, kIdentityXyzw, QuatOrder::kXyzw, 29);
    check(a.verdict() == QuatVerdict::kUndetermined, "29 个低动态样本 -> 拒判");
    check(a.reason().find("低动态样本不足") != std::string::npos,
          "原因点明样本数不足");
  }
}

// ---------------------------------------------------------------------------
// [6] 拒判场景：条件不满足时不给硬结论，原因说清缺什么
// ---------------------------------------------------------------------------
static void test_refuse_to_judge() {
  std::printf("[6] 拒判场景\n");

  {
    // 倾斜 45°：姿态不是单位四元数，非主导分量超限
    const float raw[4] = {0.3826834f, 0.f, 0.f, 0.9238795f};
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, raw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kUndetermined, "倾斜 45 度 -> 拒判");
    check(a.reason().find("倾斜") != std::string::npos, "原因提示雷达可能倾斜");
  }
  {
    // 全零四元数：固件未初始化的典型表现（归一化失败，整窗无有效样本）
    const float raw[4] = {0.f, 0.f, 0.f, 0.f};
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, raw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kUndetermined, "全零四元数 -> 拒判");
    check(a.invalid() == 100, "全零样本计入 invalid 而非 lowDynamic");
    check(a.reason().find("全零") != std::string::npos,
          "原因点明全零/非法（固件可能未初始化）");
  }
  {
    // 并列：两个分量均值相等，dominantIndex 返回 -1
    const float pub[4] = {0.5f, 0.5f, 0.f, 0.f};
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedPub(a, pub, 100);
    check(a.dominantIndex() == -1, "并列时 dominantIndex 为 -1");
    check(a.verdict() == QuatVerdict::kUndetermined, "并列 -> 拒判");
    check(a.reason().find("并列") != std::string::npos, "原因点明并列");
  }
  {
    // 与当前设置矛盾：声称参数是 wxyz，却看到 x 位主导——两种顺序都解释不了
    const float pub[4] = {1.f, 0.f, 0.f, 0.f};
    QuatVerdictAnalyzer a(QuatOrder::kWxyz);
    feedPub(a, pub, 100);
    check(a.dominantIndex() == 0, "矛盾场景主导分量在第 1 位");
    check(a.verdict() == QuatVerdict::kUndetermined, "位置与设置矛盾 -> 拒判");
    check(a.reason().find("矛盾") != std::string::npos, "原因点明与设置矛盾");
  }
  {
    // 主导分量落在 y 位（第 2 位）：180° 绕 y，两种顺序都不对应单位四元数
    const float pub[4] = {0.f, 1.f, 0.f, 0.f};
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedPub(a, pub, 100);
    check(a.verdict() == QuatVerdict::kUndetermined, "y 位主导 -> 拒判");
    check(a.reason().find("矛盾") != std::string::npos,
          "y 位主导同样按矛盾拒判");
  }
}

// ---------------------------------------------------------------------------
// [7] 稳健性：噪声不改变结论；非归一化输入先归一化再统计
// ---------------------------------------------------------------------------
static void test_robustness() {
  std::printf("[7] 稳健性：噪声与非归一化\n");

  {
    // 噪声：x 分量正负交替（均值 0），w 在 1 附近抖动
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    float pub[4];
    resolveQuaternion(kIdentityXyzw, QuatOrder::kXyzw, pub);
    for (int i = 0; i < 100; ++i) {
      pub[0] = (i % 2 == 0) ? 0.02f : -0.02f;
      pub[3] = (i % 2 == 0) ? 0.99f : 1.01f;
      a.feed(pub, 0.001f);
    }
    check(a.verdict() == QuatVerdict::kXyzw, "带噪声的单位四元数仍判 xyzw");
    check(std::fabs(a.mean(3) - 1.0) < 0.01, "噪声下 w 均值仍 ~= 1");
    check(a.maxAbs(0) >= 0.019f, "maxAbs 记录到噪声峰值");
  }
  {
    // 非归一化：模为 2 的 identity，归一化后判据不变
    const float raw[4] = {0.f, 0.f, 0.f, 2.f};
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, raw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kXyzw, "模 2 的 xyzw identity 仍判 xyzw");
    check(std::fabs(a.mean(3) - 1.0) < 1e-9, "归一化后 w 均值精确为 1");
  }
  {
    // 非归一化 wxyz：模为 2，参数 xyzw（错）-> 仍判出 wxyz
    const float raw[4] = {2.f, 0.f, 0.f, 0.f};
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    feedRaw(a, raw, QuatOrder::kXyzw, 100);
    check(a.verdict() == QuatVerdict::kWxyz, "模 2 的 wxyz identity 仍判 wxyz");
  }
}

// ---------------------------------------------------------------------------
// [8] 统计量与 API 契约
// ---------------------------------------------------------------------------
static void test_stats_and_api() {
  std::printf("[8] 统计量与 API 契约\n");

  {
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    check(a.total() == 0 && a.lowDynamic() == 0, "空分析器计数为 0");
    check(a.dominantIndex() == -1, "空分析器无主导分量");
    check(a.verdict() == QuatVerdict::kUndetermined, "空分析器拒判");
    check(a.reason().find("未收到任何 IMU 样本") != std::string::npos,
          "空分析器原因点明未收到样本");
    check(std::fabs(a.mean(0)) < 1e-12, "空分析器 mean 为 0（不除零）");
  }
  {
    // feed 返回值语义：低动态 true、运动 false、全零 false
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    const float q[4] = {0.f, 0.f, 0.f, 1.f};
    const float zero[4] = {0.f, 0.f, 0.f, 0.f};
    check(a.feed(q, 0.001f), "低动态有效样本 feed 返回 true");
    check(!a.feed(q, 0.5f), "运动样本 feed 返回 false");
    check(!a.feed(zero, 0.001f), "全零四元数 feed 返回 false");
    check(a.total() == 3, "total 统计全部喂入的样本");
    check(a.lowDynamic() == 1, "lowDynamic 只统计计入的样本");
  }
  {
    // mean/maxAbs 语义：喂两个 |q|=1 的样本交替（归一化是恒等变换），
    // 均值应为算术平均、maxAbs 应为 |值| 最大。注意分析器逐个样本归一化，
    // 所以这里必须用已归一化的输入，否则测到的是归一化后的值。
    QuatVerdictAnalyzer a(QuatOrder::kXyzw);
    const float qa[4] = {0.6f, 0.8f, 0.f, 0.f};  // |q| = 1
    const float qb[4] = {0.8f, 0.6f, 0.f, 0.f};  // |q| = 1
    const float qc[4] = {0.f, 0.f, 1.f, 0.f};    // |q| = 1
    for (int i = 0; i < 10; ++i) a.feed(i % 2 == 0 ? qa : qb, 0.001f);
    check(std::fabs(a.mean(0) - 0.7) < 1e-6, "mean 为算术平均");
    check(std::fabs(a.mean(1) - 0.7) < 1e-6, "mean 逐分量独立");
    check(std::fabs(a.maxAbs(0) - 0.8f) < 1e-6, "maxAbs 为 |值| 最大");
    check(a.maxAbs(2) == 0.0, "未出现的分量 maxAbs 为 0");
    a.feed(qc, 0.001f);
    check(a.maxAbs(2) == 1.0, "单次大值即抬升 maxAbs");
    check(a.lowDynamic() == 11, "lowDynamic 计数含后续样本");
  }
}

int main() {
  std::printf("unilidar_driver 四元数判定自检\n\n");
  test_setting_correct();
  test_setting_wrong();
  test_sign_dual_cover();
  test_low_dynamic_gate();
  test_min_samples_boundary();
  test_refuse_to_judge();
  test_robustness();
  test_stats_and_api();

  std::printf("\n检查项 %d，失败 %d -> %s\n", g_checks, g_failed,
              g_failed == 0 ? "全部通过" : "存在失败");
  return g_failed == 0 ? 0 : 1;
}
