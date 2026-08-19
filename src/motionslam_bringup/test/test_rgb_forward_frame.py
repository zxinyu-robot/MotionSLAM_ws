"""rgb_forward_frame 单测."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from rgb_forward_frame import (  # noqa: E402
    RgbForwardMetaInput,
    build_rgb_forward_meta,
    pack_rgb_forward_frame,
    unpack_rgb_forward_frame,
)


def test_pack_unpack_roundtrip():
    payload = b"\x00\x00\x00\x01" + b"\xab" * 100
    meta = build_rgb_forward_meta(
        RgbForwardMetaInput(
            session_id="s1",
            stream_field="video360p",
            payload=payload,
            time_frame=99,
            seq=7,
        )
    )
    frame = pack_rgb_forward_frame(meta, payload)
    meta2, payload2, consumed = unpack_rgb_forward_frame(frame)
    assert consumed == len(frame)
    assert meta2["schema"] == "motionslam.rgb_forward.v1"
    assert meta2["seq"] == 7
    assert payload2 == payload
