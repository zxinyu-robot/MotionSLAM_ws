#!/usr/bin/env python3
"""v1 边侧上行 TCP 客户端：JSON Lines 与 binary forward 共用 IO 线程."""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Union

JsonPayload = dict[str, Any]
BinaryPayload = tuple[dict[str, Any], bytes]
QueueItem = Union[JsonPayload, BinaryPayload]


@dataclass
class UplinkStats:
    sent_ok: int = 0
    sent_fail: int = 0
    dropped: int = 0
    reconnects: int = 0
    last_error: str = ""
    last_ok_mono: float = 0.0


class EdgeTcpUplinkClient:
    """JSON Lines over TCP；也支持 enqueue_binary(meta, raw_bytes)."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        queue_max: int = 8,
        connect_timeout_s: float = 2.0,
        send_timeout_s: float = 3.0,
        on_error: Optional[Callable[[str], None]] = None,
        binary_packer: Optional[Callable[[dict[str, Any], bytes], bytes]] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._connect_timeout = connect_timeout_s
        self._send_timeout = send_timeout_s
        self._binary_packer = binary_packer
        self._queue: queue.Queue[QueueItem] = queue.Queue(maxsize=max(1, queue_max))
        self._stats = UplinkStats()
        self._on_error = on_error
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread = threading.Thread(target=self._run, name=f"edge-uplink-{port}", daemon=True)
        self._thread.start()

    @property
    def stats(self) -> UplinkStats:
        return self._stats

    def stop(self) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait({})
        except queue.Full:
            pass
        self._thread.join(timeout=2.0)
        self._close_socket()

    def enqueue(self, payload: dict[str, Any]) -> bool:
        return self._put(payload)

    def enqueue_binary(self, meta: dict[str, Any], payload: bytes) -> bool:
        if not payload:
            return False
        return self._put((meta, payload))

    def _put(self, item: QueueItem) -> bool:
        if self._stop.is_set():
            return False
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            self._stats.dropped += 1
            self._emit_error("uplink queue full, drop frame")
            return False

    def _emit_error(self, msg: str) -> None:
        self._stats.last_error = msg
        if self._on_error:
            self._on_error(msg)

    def _connect(self) -> bool:
        self._close_socket()
        try:
            sock = socket.create_connection(
                (self._host, self._port), timeout=self._connect_timeout
            )
            sock.settimeout(self._send_timeout)
            self._sock = sock
            return True
        except OSError as exc:
            self._emit_error(f"connect {self._host}:{self._port} failed: {exc}")
            return False

    def _close_socket(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _send_one(self, item: QueueItem) -> bool:
        if not item:
            return False
        if isinstance(item, tuple):
            meta, payload = item
            if self._binary_packer is None:
                self._emit_error("binary_packer not configured")
                return False
            data = self._binary_packer(meta, payload)
        else:
            line = json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n"
            data = line.encode("utf-8")
        if self._sock is None and not self._connect():
            return False
        assert self._sock is not None
        try:
            self._sock.sendall(data)
            return True
        except OSError as exc:
            self._emit_error(f"send failed: {exc}")
            self._close_socket()
            self._stats.reconnects += 1
            return False

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if self._stop.is_set() or not item:
                break
            if self._send_one(item):
                self._stats.sent_ok += 1
                self._stats.last_ok_mono = time.monotonic()
            else:
                self._stats.sent_fail += 1
            time.sleep(0.001)
