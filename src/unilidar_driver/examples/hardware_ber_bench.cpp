// 真机 CH340 @ 2 Mbps 误码率 / 长稳测试 bench。回答的问题是：
//
//   「L1 经 CH340 以 2 Mbps 接到香橙派后，真实物理链路在长时间下
//     帧错误率多少？内存/文件描述符会不会悄悄涨？」
//
// 这是 A1 串口压力自检（PTY 虚拟串口，只验解析稳定性）的「物理层」补齐：
// A1 已证明「驱动在 2 Mbps 字节流下零丢失、零误判、内存有界」，但那是虚拟
// 串口、字节流是测试自己生成的、没有一位是错的。物理层有没有误码、长时间
// 跑会不会漏 fd / 涨内存，只有接真雷达才看得到——本 bench 就是接真雷达时
// 一键出的那份报告。
//
// 三种用法（都不依赖 ROS）：
//   1) 真机：  hardware_ber_bench --device /dev/ttyUSB0 --baud 2000000
//                --duration 7200 --report ber_report.md --csv ber_report.csv
//   2) 回放：  hardware_ber_bench --input capture.bin
//              （capture.bin 由 `cat /dev/ttyUSB0 > capture.bin` 之类录得）
//   3) 自检：  hardware_ber_bench --selftest [--quiet]   # CI 离线回归，不碰硬件
//
// 为什么需要雷达：雷达自发真实点云，没有已知测试图案，无法逐位比对，因此
// 用「帧错误率」作为物理层质量的【上界】代理——
//     frame_error_rate = bad_crc / frames_ok
// 一帧里任意一位翻转几乎必然导致 CRC 失败，所以这个比率是逐位 BER 的上界，
// 不是下界也不是精确值。要逐位 BER，需要回环已知图案测试（见 docs/ber-bench.md）。
//
// 复用：SerialReceiver（真实设备打开）、PacketParser（定界+CRC 统计）、
//       FrameDecoder（解出点云/IMU 作为链路健康二级指标）。

#include <dirent.h>
#include <sys/resource.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "unilidar_driver/frame_decoder.hpp"
#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/protocol.hpp"
#include "unilidar_driver/serial_receiver.hpp"

// 自检用的合成帧构造（与 decoder_selftest 同一套 seal 逻辑）
#include "selftest_fixture.hpp"

using namespace unilidar_driver;
using namespace unilidar_selftest;

namespace {

struct Options {
  std::string device;
  int baud{2000000};
  double duration_s{7200.0};
  std::string report_md;
  std::string report_csv;
  std::string input_file;
  bool selftest{false};
  bool quiet{false};
};

bool parseArgs(int argc, char** argv, Options& opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "参数 %s 缺值\n", name);
        return nullptr;
      }
      return argv[++i];
    };
    if (a == "--device") {
      const char* v = need_value("--device");
      if (!v) return false;
      opt.device = v;
    } else if (a == "--baud") {
      const char* v = need_value("--baud");
      if (!v) return false;
      opt.baud = std::atoi(v);
    } else if (a == "--duration") {
      const char* v = need_value("--duration");
      if (!v) return false;
      opt.duration_s = std::atof(v);
    } else if (a == "--report") {
      const char* v = need_value("--report");
      if (!v) return false;
      opt.report_md = v;
    } else if (a == "--csv") {
      const char* v = need_value("--csv");
      if (!v) return false;
      opt.report_csv = v;
    } else if (a == "--input") {
      const char* v = need_value("--input");
      if (!v) return false;
      opt.input_file = v;
    } else if (a == "--selftest") {
      opt.selftest = true;
    } else if (a == "--quiet") {
      opt.quiet = true;
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      return false;
    }
  }
  if (!opt.selftest && opt.device.empty() && opt.input_file.empty()) {
    std::fprintf(stderr, "必须给定 --device、--input 或 --selftest 之一\n");
    return false;
  }
  if (opt.duration_s <= 0.0 && !opt.selftest) {
    std::fprintf(stderr, "--duration 必须为正\n");
    return false;
  }
  return true;
}

double rssPeakMb() {
  struct rusage ru {};
  ::getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
  return static_cast<double>(ru.ru_maxrss) / (1024.0 * 1024.0);
#else
  return static_cast<double>(ru.ru_maxrss) / 1024.0;
#endif
}

// CPU 累计时间（用户+系统，秒）。两次采样之差 / 墙钟之差 = 本进程 CPU 占比。
double cpuSeconds() {
  struct rusage ru {};
  ::getrusage(RUSAGE_SELF, &ru);
  return static_cast<double>(ru.ru_utime.tv_sec) + ru.ru_utime.tv_usec / 1e6 +
         static_cast<double>(ru.ru_stime.tv_sec) + ru.ru_stime.tv_usec / 1e6;
}

// 当前打开的 fd 数（读 /proc/self/fd）。非 Linux（无 /proc）时返回 -1 表示跳过。
long fdCount() {
  DIR* d = ::opendir("/proc/self/fd");
  if (d == nullptr) return -1;
  long n = 0;
  while (::readdir(d) != nullptr) ++n;
  ::closedir(d);
  return n;  // 含 . 与 .. 两个目录项，仅作相对监视，不影响趋势判断
}

// 把一帧喂给解码器，统计点云/IMU 解出数与解码失败数（解码失败 = 数据段虽过
// CRC 但字段越界，是链路健康的二级指标，应与 bad_crc 解耦看待）。
struct DecodeTally {
  std::uint64_t clouds{0};
  std::uint64_t imus{0};
  std::uint64_t decode_fail{0};
};

DecodeTally decodeFrameTally(const std::vector<std::uint8_t>& frame,
                             std::uint32_t type, DecodeStats& stats) {
  DecodeTally t{};
  DecodeParams params;
  params.quat_order = QuatOrder::kXyzw;
  if (type == kTypePointCloud3D) {
    std::vector<PointXYZI> pts;
    double stamp = 0.0;
    if (decodePointCloud(frame, params, 0.0, pts, stamp, stats)) ++t.clouds;
    else ++t.decode_fail;
  } else if (type == kTypeImu) {
    ImuSample imu{};
    if (decodeImu(frame, params, 0.0, imu, stats)) ++t.imus;
    else ++t.decode_fail;
  }
  return t;
}

// 生成一段合成字节流用于 --selftest：N 个好帧（点云+IMU 交错）+ 1 个被篡改
// 的好帧（翻转数据段一个字节 -> 必触发 bad_crc）。返回字节流与预期计数。
struct SyntheticExpect {
  std::size_t good_cloud{0};
  std::size_t good_imu{0};
  std::size_t bad_crc{1};  // 故意注入 1 个坏帧
};

void buildSynthetic(std::vector<std::uint8_t>& out, SyntheticExpect& exp) {
  out.clear();
  std::uint32_t cloud_seq = 1, imu_seq = 1;
  for (int i = 0; i < 100; ++i) {
    if (i % 10 == 5) {
      auto f = makeImuFrame(imu_seq++);
      out.insert(out.end(), f.begin(), f.end());
      ++exp.good_imu;
    } else {
      auto f = makeCloudFrame(cloud_seq++, 8);
      out.insert(out.end(), f.begin(), f.end());
      ++exp.good_cloud;
    }
  }
  // 篡改一帧：复制一个好点云帧，翻转折射数据段最后一字节（CRC 覆盖范围）。
  // 故意用「与上一条好点云相同的 seq」——CRC 失败的帧不会被计入序号状态，
  // 因此末尾补的好帧相对最后一条成功点云的 seq 只 +1，不会凭空制造 seq_gaps。
  auto bad = makeCloudFrame(90, 8);
  const std::size_t last = bad.size() - kTailSize - 1;
  bad[last] ^= 0xFF;
  out.insert(out.end(), bad.begin(), bad.end());
  // 末尾再补一个好帧：验证解析器在遇到坏帧重同步后能干净排空（buffered==0），
  // 而不是把坏帧残余字节永远留在缓冲里。
  auto tail = makeCloudFrame(cloud_seq++, 8);
  out.insert(out.end(), tail.begin(), tail.end());
  ++exp.good_cloud;
}

void runStream(const std::uint8_t* data, std::size_t len, PacketParser& parser,
               DecodeStats& dstats, DecodeTally& dtally) {
  parser.append(data, len);
  std::vector<std::uint8_t> out;
  std::uint32_t type = 0;
  for (;;) {
    const PullStatus st = parser.pull(out, type);
    if (st != PullStatus::kFrameReady) break;
    const DecodeTally t = decodeFrameTally(out, type, dstats);
    dtally.clouds += t.clouds;
    dtally.imus += t.imus;
    dtally.decode_fail += t.decode_fail;
  }
}

void emitReport(const Options& opt, double elapsed_s, double bytes_in,
                const ParseStats& ps, const DecodeStats& ds, const DecodeTally& dt,
                double rss_peak, double cpu_pct, long fd_peak, bool real_hw) {
  const std::uint64_t frames_ok = ps.frames_ok;
  const double fer = frames_ok > 0
                         ? static_cast<double>(ps.bad_crc) / static_cast<double>(frames_ok)
                         : 0.0;
  const double mbps = elapsed_s > 0 ? (bytes_in / 1e6) / elapsed_s : 0.0;

  std::stringstream md;
  md << "# CH340 @ " << (real_hw ? "2 Mbps 真机" : "回放/自检") << " 误码率 / 长稳报告\n\n";
  md << "- 时长: " << elapsed_s << " s\n";
  md << "- 字节入: " << bytes_in << " (" << mbps << " MB/s)\n";
  md << "- 模式: " << (real_hw ? opt.device : (opt.input_file.empty() ? "selftest" : opt.input_file)) << "\n\n";
  md << "## 帧统计（PacketParser）\n";
  md << "- frames_ok: " << frames_ok << "\n";
  md << "- bad_crc: " << ps.bad_crc << "\n";
  md << "- bad_tail: " << ps.bad_tail << "\n";
  md << "- bad_size: " << ps.bad_size << "\n";
  md << "- resync_bytes: " << ps.resync_bytes << "\n";
  md << "- seq_gaps: " << ps.seq_gaps << "\n";
  md << "- duplicates: " << ps.duplicates << "\n";
  md << "- out_of_order: " << ps.out_of_order << "\n";
  md << "- buffer_overflow: " << ps.buffer_overflow << "\n\n";
  md << "## 解码统计（FrameDecoder）\n";
  md << "- clouds_decoded: " << dt.clouds << "\n";
  md << "- imus_decoded: " << dt.imus << "\n";
  md << "- decode_fail: " << dt.decode_fail << "\n";
  md << "- dropped_user_range: " << ds.dropped_user_range << "\n\n";
  md << "## 资源采样\n";
  md << "- RSS 峰值: " << rss_peak << " MB\n";
  md << "- CPU 占比(进程): " << cpu_pct << " %\n";
  md << "- fd 峰值: " << (fd_peak < 0 ? std::string("N/A") : std::to_string(fd_peak)) << "\n\n";
  md << "## 误码率代理\n";
  md << "- frame_error_rate = bad_crc / frames_ok = " << fer << "\n";
  md << "- 注: 雷达自发真实点云、无已知测试图案，无法逐位比对；帧错误率是逐位 BER 的【上界】代理。\n";
  md << "- 逐位 BER 需回环已知图案测试（见 docs/ber-bench.md）。\n";

  if (!opt.report_md.empty()) {
    std::ofstream f(opt.report_md);
    f << md.str();
  } else if (!opt.quiet) {
    std::printf("%s", md.str().c_str());
  }

  if (!opt.report_csv.empty()) {
    std::ofstream f(opt.report_csv);
    f << "metric,value\n";
    f << "elapsed_s," << elapsed_s << "\n";
    f << "bytes_in," << bytes_in << "\n";
    f << "frames_ok," << frames_ok << "\n";
    f << "bad_crc," << ps.bad_crc << "\n";
    f << "bad_tail," << ps.bad_tail << "\n";
    f << "bad_size," << ps.bad_size << "\n";
    f << "resync_bytes," << ps.resync_bytes << "\n";
    f << "seq_gaps," << ps.seq_gaps << "\n";
    f << "duplicates," << ps.duplicates << "\n";
    f << "clouds_decoded," << dt.clouds << "\n";
    f << "imus_decoded," << dt.imus << "\n";
    f << "decode_fail," << dt.decode_fail << "\n";
    f << "rss_peak_mb," << rss_peak << "\n";
    f << "cpu_pct," << cpu_pct << "\n";
    f << "fd_peak," << fd_peak << "\n";
    f << "frame_error_rate," << fer << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) return 2;

  SelftestLog log;

  if (opt.selftest) {
    // 离线自检：合成流喂同一条解析链路，核对计数与 --selftest 的预期一致。
    SyntheticExpect exp{};
    std::vector<std::uint8_t> stream;
    buildSynthetic(stream, exp);

    PacketParser parser;
    DecodeStats ds{};
    DecodeTally dt{};
    runStream(stream.data(), stream.size(), parser, ds, dt);
    const ParseStats& ps = parser.stats();

    if (!opt.quiet) {
      std::printf("hardware_ber_bench 离线自检（合成流，不碰硬件）\n");
      std::printf("流字节 %zu · 解出帧 %zu · bad_crc %zu\n\n", stream.size(),
                  static_cast<std::size_t>(ps.frames_ok),
                  static_cast<std::size_t>(ps.bad_crc));
    }

    log.check(ps.frames_ok == exp.good_cloud + exp.good_imu, "解出帧数应等于注入的好帧数");
    log.check(ps.bad_crc == exp.bad_crc, "篡改帧应且仅计一次 bad_crc");
    log.check(dt.clouds == exp.good_cloud, "点云解出数应与注入一致");
    log.check(dt.imus == exp.good_imu, "IMU 解出数应与注入一致");
    log.check(ps.seq_gaps == 0, "连续递增序号不得出现缺口");
    log.check(parser.buffered() == 0, "排空后 parser 不应残留字节");
    log.check(ds.dropped_user_range == 0, "默认配置下好帧不应被用户距离门限丢弃");
    // 模拟「报告」输出，确认报告生成路径不崩溃
    emitReport(opt, 1.0, stream.size(), ps, ds, dt, 5.0, 12.3, 7, false);
    if (!opt.quiet) {
      std::printf("\n结论: 解析链路计数正确，报告生成路径可用；"
                  "接真雷达时改用 --device 即出物理层报告。\n");
    }
    return log.report() == 0 ? 0 : 1;
  }

  // ---- 真机 / 回放 模式 ----
  PacketParser parser;
  DecodeStats ds{};
  DecodeTally dt{};

  std::vector<std::uint8_t> rbuf(65536);
  std::string err;
  std::uint64_t bytes_in = 0;
  bool real_hw = !opt.device.empty();

  if (real_hw) {
    SerialReceiver rx;
    if (!rx.open(opt.device, opt.baud, 0, err)) {
      std::fprintf(stderr, "打开设备 %s 失败: %s\n", opt.device.c_str(), err.c_str());
      return 2;
    }
    if (!opt.quiet) {
      std::printf("真机 bench: %s @ %d bps，时长 %.0f s\n\n", opt.device.c_str(),
                  opt.baud, opt.duration_s);
    }
    const auto t0 = std::chrono::steady_clock::now();
    double next_sample = 1.0;
    double cpu0 = cpuSeconds();
    double rss_peak = rssPeakMb();
    long fd_peak = fdCount();
    while (true) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed >= opt.duration_s) break;
      const int n = rx.readSome(rbuf.data(), rbuf.size(), err);
      if (n > 0) {
        bytes_in += static_cast<std::uint64_t>(n);
        runStream(rbuf.data(), static_cast<std::size_t>(n), parser, ds, dt);
        const double rss = rssPeakMb();
        if (rss > rss_peak) rss_peak = rss;
        const long fd = fdCount();
        if (fd > fd_peak) fd_peak = fd;
      } else if (n < 0) {
        std::fprintf(stderr, "readSome 出错: %s\n", err.c_str());
        return 2;
      } else {
        ::usleep(500);
      }
      if (elapsed >= next_sample) {
        next_sample += 1.0;  // 每秒采样一次（已在上面取峰值）
      }
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const double cpu1 = cpuSeconds();
    const double cpu_pct = elapsed > 0 ? (cpu1 - cpu0) / elapsed * 100.0 : 0.0;
    emitReport(opt, elapsed, bytes_in, parser.stats(), ds, dt, rss_peak, cpu_pct,
               fd_peak, true);
    if (!opt.quiet) {
      std::printf("真机 bench 完成：报告已写出（%s%s）\n",
                  opt.report_md.empty() ? "" : (opt.report_md + " ").c_str(),
                  opt.report_csv.empty() ? "" : opt.report_csv.c_str());
    }
    return 0;
  }

  // 回放模式：把整文件读进来喂解析链路（文件不大时一次性；大文件按块读）
  if (!opt.input_file.empty()) {
    std::ifstream f(opt.input_file, std::ios::binary);
    if (!f) {
      std::fprintf(stderr, "无法打开输入文件: %s\n", opt.input_file.c_str());
      return 2;
    }
    if (!opt.quiet) std::printf("回放模式: %s\n\n", opt.input_file.c_str());
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::uint8_t> chunk(1 << 20);
    double rss_peak = rssPeakMb();
    long fd_peak = fdCount();
    while (f) {
      f.read(reinterpret_cast<char*>(chunk.data()), chunk.size());
      const std::streamsize got = f.gcount();
      if (got <= 0) break;
      bytes_in += static_cast<std::uint64_t>(got);
      runStream(chunk.data(), static_cast<std::size_t>(got), parser, ds, dt);
      const double rss = rssPeakMb();
      if (rss > rss_peak) rss_peak = rss;
      const long fd = fdCount();
      if (fd > fd_peak) fd_peak = fd;
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    emitReport(opt, elapsed, bytes_in, parser.stats(), ds, dt, rss_peak, 0.0, fd_peak,
               false);
    return 0;
  }

  return 2;
}
