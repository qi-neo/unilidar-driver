// 四元数分量顺序一键判定工具。接真实 L1、驱动节点在跑时用：
//   ros2 run unilidar_driver quat_check
//   ros2 run unilidar_driver quat_check --ros-args -p window_sec:=30 -p current_order:=wxyz
//
// 判据、结论怎么用见 quat_verdict.hpp 头注释；判定核心的离线自检是
// quat_check_selftest（两种顺序、q/-q 双覆盖、拒判场景、噪声与非归一化）。
//
// 分工与驱动节点一致：本文件只做「订阅 /unilidar/imu + 喂样本 + 打印」的
// 接线，判定逻辑全部在 quat_verdict.hpp（不依赖 ROS，可离线自检）。
// 输出用裸 printf：这是个 CLI 工具，stdout 就是交付物，不加日志前缀。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "unilidar_driver/quat_verdict.hpp"

namespace {

constexpr const char* kDefaultImuTopic = "/unilidar/imu";
constexpr double kDefaultWindowSec = 60.0;
constexpr double kProgressPeriodSec = 10.0;

}  // namespace

class QuatCheckNode : public rclcpp::Node {
public:
  QuatCheckNode() : rclcpp::Node("quat_check"),
                    analyzer_(unilidar_driver::QuatOrder::kXyzw) {
    window_sec_ = declare_parameter<double>("window_sec", kDefaultWindowSec);
    const std::string order = declare_parameter<std::string>("current_order", "xyzw");
    const std::string topic = declare_parameter<std::string>("imu_topic", kDefaultImuTopic);

    if (order != "xyzw" && order != "wxyz") {
      std::fprintf(stderr, "current_order=\"%s\" 非法，只接受 xyzw 或 wxyz\n",
                   order.c_str());
      ok_ = false;
      return;
    }
    if (!(window_sec_ > 0.0)) {
      std::fprintf(stderr, "window_sec=%f 非法，必须为正数\n", window_sec_);
      ok_ = false;
      return;
    }
    current_ = (order == "xyzw") ? unilidar_driver::QuatOrder::kXyzw
                                 : unilidar_driver::QuatOrder::kWxyz;
    analyzer_ = unilidar_driver::QuatVerdictAnalyzer(current_);
    topic_ = topic;
    // QoS 必须与驱动节点的 imu_pub_ 一致（SensorDataQoS / BEST_EFFORT），
    // 否则订阅端默认 RELIABLE 匹配不上，一个样本都收不到
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&QuatCheckNode::onImu, this, std::placeholders::_1));
  }

  bool started() const { return ok_; }

  // 跑满统计窗口，打印进度与最终结论。返回进程退出码：
  // 0 = 判定成功，1 = 无法判定（原因见输出），2 = 启动失败
  int run() {
    const auto t0 = std::chrono::steady_clock::now();
    auto last_report = t0;
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - t0).count();
      if (elapsed >= window_sec_) break;
      if (std::chrono::duration<double>(now - last_report).count() >= kProgressPeriodSec) {
        printProgress(elapsed);
        last_report = now;
      }
      // 有节拍地 spin：不忙等，也不漏消息（未处理的在 executor 里排队）
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    printFinal();
    return analyzer_.verdict() == unilidar_driver::QuatVerdict::kUndetermined ? 1 : 0;
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const float q[4] = {static_cast<float>(msg->orientation.x),
                        static_cast<float>(msg->orientation.y),
                        static_cast<float>(msg->orientation.z),
                        static_cast<float>(msg->orientation.w)};
    const float omega = std::sqrt(static_cast<float>(
        msg->angular_velocity.x * msg->angular_velocity.x +
        msg->angular_velocity.y * msg->angular_velocity.y +
        msg->angular_velocity.z * msg->angular_velocity.z));
    const float acc = std::sqrt(static_cast<float>(
        msg->linear_acceleration.x * msg->linear_acceleration.x +
        msg->linear_acceleration.y * msg->linear_acceleration.y +
        msg->linear_acceleration.z * msg->linear_acceleration.z));
    analyzer_.feed(q, omega);
    acc_sum_ += acc;
    ++acc_n_;
  }

  void printProgress(double elapsed) const {
    std::printf("[quat_check] %.1fs / %.1fs | 样本 %zu（低动态 %zu，无效 %zu）\n",
                elapsed, window_sec_, analyzer_.total(), analyzer_.lowDynamic(),
                analyzer_.invalid());
  }

  void printFinal() const {
    std::printf("\n[quat_check] 窗口 %.1f s | 订阅 %s | 当前 quaternion_order=%s\n",
                window_sec_, topic_.c_str(),
                current_ == unilidar_driver::QuatOrder::kXyzw ? "xyzw" : "wxyz");
    std::printf("[quat_check] 样本 %zu（低动态 %zu，无效 %zu）\n", analyzer_.total(),
                analyzer_.lowDynamic(), analyzer_.invalid());
    std::printf("[quat_check] 低动态样本四元数均值：x=%.4f y=%.4f z=%.4f w=%.4f\n",
                analyzer_.mean(0), analyzer_.mean(1), analyzer_.mean(2),
                analyzer_.mean(3));
    std::printf("[quat_check] 各分量 |值| 最大：x=%.4f y=%.4f z=%.4f w=%.4f\n",
                analyzer_.maxAbs(0), analyzer_.maxAbs(1), analyzer_.maxAbs(2),
                analyzer_.maxAbs(3));
    if (acc_n_ > 0) {
      std::printf("[quat_check] 线加速度模均值 %.2f m/s^2（静止水平应 ~= 9.8）\n",
                  acc_sum_ / static_cast<double>(acc_n_));
    }
    std::printf("判定结果：%s\n", analyzer_.verdictText());
    std::printf("%s\n", analyzer_.reason().c_str());
  }

  double window_sec_{kDefaultWindowSec};
  unilidar_driver::QuatOrder current_{unilidar_driver::QuatOrder::kXyzw};
  unilidar_driver::QuatVerdictAnalyzer analyzer_;
  std::string topic_{kDefaultImuTopic};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  double acc_sum_{0.0};
  std::size_t acc_n_{0};
  bool ok_{true};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QuatCheckNode>();
  if (!node->started()) {
    rclcpp::shutdown();
    return 2;
  }
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
