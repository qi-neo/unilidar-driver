#!/usr/bin/env bash
# UDP loopback integration test: build the probe -> listen in background ->
# send synthetic packets -> verify the summary.
#
# This is the only automated coverage for udp_receiver.cpp. The 63 assertions in
# decoder_selftest feed bytes straight into the parser and never touch a socket;
# in reality packets first go through recvfrom, and buffering, timeouts and
# source addresses can only be exercised by a real send/receive.
#
# Usage: bash tests/udp_loopback_test.sh
# Exit code 0 means pass.
#
# Note: keep this script ASCII-only. Under a C locale, bash treats the trailing
# bytes of a full-width comma as part of a variable name, so "$CXX," becomes an
# unset variable and `set -u` aborts the script.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PORT="${UDP_LOOPBACK_PORT:-46201}"
HOST="127.0.0.1"
LISTEN_SECONDS="${UDP_LOOPBACK_SECONDS:-4}"
CXX="${CXX:-clang++}"
# 额外编译选项，CI 用它叠加 sanitizer：
#   EXTRA_CXXFLAGS="-fsanitize=address,undefined -g" bash tests/udp_loopback_test.sh
EXTRA_CXXFLAGS="${EXTRA_CXXFLAGS:-}"
PROBE_BIN="${REPO_ROOT}/udp_probe"
PROBE_OUT="/tmp/udp_probe_loopback.txt"

echo "=== Building probe (${CXX} ${EXTRA_CXXFLAGS}, warnings as errors) ==="
# shellcheck disable=SC2086  # EXTRA_CXXFLAGS 需要分词展开
"${CXX}" -std=c++17 -Wall -Wextra -Wpedantic -Werror -O1 ${EXTRA_CXXFLAGS} \
  -I src/unilidar_driver/include \
  src/unilidar_driver/examples/udp_probe.cpp \
  src/unilidar_driver/src/protocol.cpp \
  src/unilidar_driver/src/packet_parser.cpp \
  src/unilidar_driver/src/udp_receiver.cpp \
  -o "$PROBE_BIN"

echo "=== Starting probe on ${HOST}:${PORT} for ${LISTEN_SECONDS}s ==="
"$PROBE_BIN" --ip "$HOST" --port "$PORT" --seconds "$LISTEN_SECONDS" \
  > "$PROBE_OUT" 2>&1 &
PROBE_PID=$!

# Give the probe time to bind before sending, otherwise the first packets are lost
sleep 1

echo "=== Sending synthetic packets ==="
python3 tests/send_synthetic_packets.py --host "$HOST" --port "$PORT"

# The probe exits on its own after the configured duration
wait "$PROBE_PID" 2>/dev/null || true

echo
echo "=== Probe output ==="
cat "$PROBE_OUT"
echo

echo "=== Verifying summary ==="
python3 - "$PROBE_OUT" <<'PY'
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()


def num(label):
    m = re.search(label + r"\s+(\d+)", text)
    if not m:
        print(f"  [FAIL] label not found in summary: {label}")
        return None
    return int(m.group(1))


expected = {
    "UDP 包数": 7,          # 6 valid point clouds + 1 CRC-corrupted
    "解出报文帧数": 6,
    "校验失败 CRC": 1,      # the corrupted one must be rejected
    "校验失败 帧尾": 0,
    "长度字段越界": 0,
    "开头非帧头的包": 0,
}

failed = 0
for label, want in expected.items():
    got = num(label)
    if got is None:
        failed += 1
        continue
    ok = got == want
    if not ok:
        failed += 1
    print(f"  [{'OK' if ok else 'FAIL'}] {label}: expected {want}, got {got}")

# Sequence numbers must be continuous, otherwise packets were lost or framing slipped
gaps = num("序号缺口")
if gaps is None:
    failed += 1
elif gaps != 0:
    failed += 1
    print(f"  [FAIL] sequence gaps: expected 0, got {gaps}")
else:
    print("  [OK] sequence gaps: 0")

# Total packet length must match what the driver expects
if "1044 字节 x 6" not in text:
    failed += 1
    print("  [FAIL] packet-length distribution does not contain '1044 bytes x 6'")
else:
    print("  [OK] packet-length distribution: 1044 bytes x 6")

print()
if failed:
    print(f"loopback test FAILED ({failed} item(s))")
    sys.exit(1)
print("loopback test PASSED")
PY

echo
echo "=== Cleanup ==="
rm -f "$PROBE_BIN"
echo "done"
