#!/usr/bin/env bash
# 停用宇树 unitree_slam 栈, 释放 mid360 (UDP 独占, 自研栈运行前必须执行)
# NOTE: 只停进程不删文件
# 恢复官方栈: /unitree/module/unitree_slam/bin/unitree_slam eth0
set -e

for p in unitree_slam mid360_driver xt16_driver; do
    if pgrep -x "$p" >/dev/null 2>&1; then
        echo "停止进程: $p"
        sudo pkill -x "$p"
    fi
done

sleep 1
# 用 -x 按进程名检查; 勿用 pgrep -f 'unitree_slam' (会误匹配本脚本 stop_unitree_slam.sh)
remaining=""
for p in unitree_slam mid360_driver xt16_driver; do
    if pgrep -x "$p" >/dev/null 2>&1; then
        remaining="$remaining $p"
    fi
done
if [[ -n "$remaining" ]]; then
    echo "ERROR: 仍有进程存活:$remaining  请手动检查: pgrep -x unitree_slam; pgrep -x mid360_driver" >&2
    exit 1
fi
echo "OK: mid360 已空闲, 可启动自研栈"
