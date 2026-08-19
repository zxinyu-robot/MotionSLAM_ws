#!/usr/bin/env bash
# 修正系统时间 (Jetson RTC 未校准时常回到 1970, 会导致 apt/colcon/ROS 异常)
#
# 优先：经 wlan0 对公网 NTP（eth0 默认路由常无外网）
#   ./scripts/ops/fix_system_time.sh
#   ./scripts/ops/fix_system_time.sh --no-ntp "2026-08-13 15:00:00"
#
# 容器内 (--privileged + --pid host):
#   docker exec motionslam bash -lc '/ws/scripts/ops/fix_system_time.sh'
#
# 宿主机 (需 sudo 密码):
#   sudo timedatectl set-ntp false
#   sudo timedatectl set-time "2026-08-13 15:00:00"
#   sudo timedatectl set-ntp true
set -euo pipefail

SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET=""
USE_NTP=true
NTP_HOST="${NTP_HOST:-120.25.115.20}"
IFACE="${TIME_SYNC_IFACE:-wlan0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-ntp)
      USE_NTP=false
      shift
      ;;
    --iface)
      IFACE="${2:?}"
      shift 2
      ;;
    --ntp-host)
      NTP_HOST="${2:?}"
      shift 2
      ;;
    *)
      if [[ -z "$TARGET" ]]; then
        TARGET="$1"
      fi
      shift
      ;;
  esac
done

TARGET="${TARGET:-2026-08-13 15:00:00}"
YEAR="$(date +%Y 2>/dev/null || echo 1970)"

ok_time() {
  [[ "$YEAR" -ge 2020 ]] && timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes
}

if ok_time; then
  echo "OK: 系统时间已同步 ($(date -Iseconds))"
  exit 0
fi

echo "WARN: 系统时间异常或未 NTP 同步 ($(date -Iseconds))"

_sntp_sync() {
  local sntp_bin=""
  for c in sntp ntpdate; do
    if command -v "$c" >/dev/null 2>&1; then
      sntp_bin="$c"
      break
    fi
  done
  if [[ -z "$sntp_bin" ]]; then
    return 1
  fi
  echo "  尝试 NTP: $NTP_HOST via $IFACE ($sntp_bin)..."
  if [[ "$sntp_bin" == "sntp" ]]; then
    if ip link show "$IFACE" >/dev/null 2>&1; then
      sntp -S -s -i "$IFACE" "$NTP_HOST"
    else
      sntp -S -s "$NTP_HOST"
    fi
  else
    ntpdate -u "$NTP_HOST"
  fi
}

if $USE_NTP; then
  if _sntp_sync 2>/dev/null; then
    YEAR="$(date +%Y)"
    if hwclock -w 2>/dev/null; then
      echo "  RTC 已写入"
    fi
    echo "OK: NTP 对时成功 ($(date -Iseconds))"
    exit 0
  fi
  echo "  NTP 对时失败，回退手动设时..."
fi

if [[ -f /.dockerenv ]] || { ! command -v timedatectl >/dev/null 2>&1; }; then
  echo "  容器内修正: date -s \"$TARGET\""
  date -s "$TARGET"
  hwclock -w 2>/dev/null || true
  echo "OK: $(date -Iseconds)"
  echo "NOTE: 断电重启可能再次回到 1970; 建议在宿主机配置 wlan0 NTP"
  exit 0
fi

if command -v timedatectl >/dev/null 2>&1; then
  echo "  宿主机修正 (需 sudo)..."
  sudo timedatectl set-ntp false || true
  sudo timedatectl set-time "$TARGET"
  sudo timedatectl set-ntp true || true
  echo "OK: $(timedatectl status | head -5)"
  exit 0
fi

echo "ERROR: 请手动: sudo date -s \"$TARGET\"" >&2
exit 1
