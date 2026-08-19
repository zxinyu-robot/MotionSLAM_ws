#!/usr/bin/env python3
"""Export logic-layer and dual-mode architecture diagrams to PNG."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "docs" / "架构" / "assets"

# Cursor dark-ish palette
BG = (24, 24, 24)
BAND = (30, 30, 30)
NODE = (28, 28, 28)
STROKE = (90, 90, 90)
TEXT = (228, 228, 228)
SUBTEXT = (160, 160, 160)
ACCENT = (89, 156, 231)
DOMAIN_STROKE = (120, 120, 120)


@dataclass
class Node:
    id: str
    label: str
    x: float
    y: float
    w: float
    h: float
    sub: Optional[str] = None


@dataclass
class Edge:
    fr: tuple[float, float]
    to: tuple[float, float]
    label: str = ""
    accent: bool = False
    dashed: bool = False
    label_dy: float = -6


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    # Prefer CJK-capable faces; DejaVu has no Chinese glyphs (renders as tofu).
    # Noto Sans CJK TTC: 0=JP, 1=KR, 2=SC, 3=TC, 4=HK
    ttc = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
        if bold
        else "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    )
    if Path(ttc).exists():
        try:
            return ImageFont.truetype(ttc, size, index=2)
        except OSError:
            pass
    fallbacks = [
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ]
    for path in fallbacks:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def anchor(node: Node, side: str) -> tuple[float, float]:
    if side == "top":
        return (node.x + node.w / 2, node.y)
    if side == "bottom":
        return (node.x + node.w / 2, node.y + node.h)
    if side == "left":
        return (node.x, node.y + node.h / 2)
    if side == "right":
        return (node.x + node.w, node.y + node.h / 2)
    return (node.x + node.w / 2, node.y + node.h / 2)


def center_row(nodes: list[dict], band_w: float, gap: float = 28) -> list[Node]:
    rows: dict[float, list[dict]] = {}
    for spec in nodes:
        rows.setdefault(spec["y"], []).append(spec)
    out: list[Node] = []
    for _, row in sorted(rows.items()):
        total = sum(s["w"] for s in row) + gap * max(0, len(row) - 1)
        x = max(16, (band_w - total) / 2)
        for spec in row:
            out.append(
                Node(
                    id=spec["id"],
                    label=spec["label"],
                    sub=spec.get("sub"),
                    x=x,
                    y=spec["y"],
                    w=spec["w"],
                    h=spec["h"],
                )
            )
            x += spec["w"] + gap
    return out


def vshift(nodes: list[Node], band_h: float, label_reserve: float = 22) -> float:
    if not nodes:
        return 0
    top = min(n.y for n in nodes)
    bottom = max(n.y + n.h for n in nodes)
    block = bottom - top
    return label_reserve + max(0, (band_h - label_reserve - block) / 2) - top


def build_five_row(
    go2_x: float,
    go2_w: float,
    edge_x: float,
    edge_w: float,
    origin_y: float,
    row_gap: float,
    rows: list[dict],
    default_gap: float = 28,
) -> tuple[list[tuple], list[Node], list[Node]]:
    bands: list[tuple] = []
    go2_nodes: list[Node] = []
    edge_nodes: list[Node] = []
    y = origin_y
    for row in rows:
        h = row["h"]
        bands.append((go2_x, y, go2_w, h, row["label"]))
        centered = center_row(row["go2"], go2_w, row.get("node_gap", default_gap))
        shift = vshift(centered, h)
        for n in centered:
            go2_nodes.append(
                Node(n.id, n.label, go2_x + n.x, y + n.y + shift, n.w, n.h, n.sub)
            )
        for spec in row.get("edge", []):
            edge_nodes.append(
                Node(
                    spec["id"],
                    spec["label"],
                    edge_x + (edge_w - spec["w"]) / 2,
                    y + (h - spec["h"]) / 2,
                    spec["w"],
                    spec["h"],
                    spec.get("sub"),
                )
            )
        y += h + row_gap
    return bands, go2_nodes, edge_nodes


def draw_arrow(draw: ImageDraw.ImageDraw, p1: tuple[float, float], p2: tuple[float, float], color: tuple, dashed: bool) -> None:
    if dashed:
        draw.line([p1, p2], fill=color, width=2)
    else:
        draw.line([p1, p2], fill=color, width=2)
    import math

    ang = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    size = 8
    a1 = ang + math.pi * 0.85
    a2 = ang - math.pi * 0.85
    tip = (
        (p2[0], p2[1]),
        (p2[0] + size * math.cos(a1), p2[1] + size * math.sin(a1)),
        (p2[0] + size * math.cos(a2), p2[1] + size * math.sin(a2)),
    )
    draw.polygon(tip, fill=color)


def render(
    width: int,
    height: int,
    title: str,
    bands: list[tuple],
    nodes: list[Node],
    edges: list[Edge],
    domains: list[tuple],
) -> Image.Image:
    img = Image.new("RGB", (width, height + 40), BG)
    draw = ImageDraw.Draw(img)
    font_title = load_font(18, bold=True)
    font_band = load_font(11, bold=True)
    font_main = load_font(12, bold=True)
    font_sub = load_font(10)
    font_edge = load_font(10)

    draw.text((16, 8), title, fill=TEXT, font=font_title)

    for x, y, w, h, label in domains:
        draw.rectangle([x, y, x + w, y + h], outline=DOMAIN_STROKE, width=2)
        draw.text((x + 12, y + 6), label, fill=SUBTEXT, font=font_band)

    for x, y, w, h, label in bands:
        draw.rectangle([x, y, x + w, y + h], fill=BAND, outline=STROKE)
        draw.text((x + 10, y + 4), label, fill=SUBTEXT, font=font_band)

    for e in edges:
        color = ACCENT if e.accent else STROKE
        draw_arrow(draw, e.fr, e.to, color, e.dashed)
        if e.label:
            mx = (e.fr[0] + e.to[0]) / 2
            my = (e.fr[1] + e.to[1]) / 2 + e.label_dy
            draw.text((mx, my), e.label, fill=SUBTEXT, font=font_edge, anchor="mm")

    for n in nodes:
        draw.rectangle([n.x, n.y, n.x + n.w, n.y + n.h], fill=NODE, outline=STROKE)
        if n.sub:
            draw.text((n.x + n.w / 2, n.y + n.h / 2 - 8), n.label, fill=TEXT, font=font_main, anchor="mm")
            draw.text((n.x + n.w / 2, n.y + n.h / 2 + 10), n.sub, fill=SUBTEXT, font=font_sub, anchor="mm")
        else:
            draw.text((n.x + n.w / 2, n.y + n.h / 2), n.label, fill=TEXT, font=font_main, anchor="mm")

    return img


def logic_layer_png() -> Image.Image:
    go2_x, go2_y, go2_w = 24, 48, 880
    inner_x, inner_w = go2_x + 12, go2_w - 24
    layer_top, layer_gap = 30, 10
    edge_x = go2_x + go2_w + 36
    edge_pad, edge_inner_w = 20, 280 - 40

    rows = [
        {
            "label": "感知定位层",
            "h": 84,
            "go2": [
                {"id": "sensors", "label": "mid360 / L1", "y": 32, "w": 118, "h": 40},
                {"id": "lio", "label": "Super-LIO", "y": 32, "w": 124, "h": 40},
                {"id": "rgb", "label": "前向 RGB", "sub": "/frontvideostream", "y": 32, "w": 136, "h": 48},
            ],
        },
        {
            "label": "地图与语义层",
            "h": 118,
            "go2": [
                {"id": "bev", "label": "RGB→BEV", "sub": "端侧推理", "y": 32, "w": 112, "h": 48},
                {"id": "enu", "label": "ENU 配准", "sub": "pose + gen", "y": 32, "w": 124, "h": 48},
                {"id": "occ", "label": "占据增量地图", "y": 32, "w": 132, "h": 40},
                {"id": "sem", "label": "语义三维地图", "y": 32, "w": 132, "h": 40},
                {"id": "life", "label": "版本化与淘汰", "y": 86, "w": 124, "h": 36},
                {"id": "rgb-up", "label": "RGB 关键帧上行", "sub": ":9878", "y": 86, "w": 148, "h": 44},
                {"id": "token", "label": "Token 编码", "y": 86, "w": 124, "h": 36},
            ],
            "edge": [
                {
                    "id": "edge-planner",
                    "label": "边端任务与语义",
                    "sub": "Planner / VLM",
                    "w": edge_inner_w,
                    "h": 48,
                }
            ],
        },
        {
            "label": "编排层",
            "h": 96,
            "go2": [
                {"id": "ingress", "label": "边端任务入口", "sub": "Ingress", "y": 32, "w": 132, "h": 48},
                {"id": "frontier", "label": "Frontier 探索", "y": 32, "w": 124, "h": 48},
                {"id": "bt", "label": "BehaviorTree", "sub": "编排 / goal 仲裁", "y": 32, "w": 156, "h": 56},
                {"id": "policy", "label": "PolicyDB", "sub": "策略归档", "y": 32, "w": 112, "h": 48},
            ],
        },
        {
            "label": "规划层",
            "h": 84,
            "node_gap": 96,
            "go2": [
                {"id": "pct", "label": "PCT", "sub": "全局粗规划", "y": 32, "w": 132, "h": 48},
                {"id": "scan", "label": "SCAN", "sub": "局部 B-spline", "y": 32, "w": 132, "h": 48},
            ],
            "edge": [
                {
                    "id": "edge-model",
                    "label": "远端模型消费",
                    "sub": "Token / RGB / 策略",
                    "w": edge_inner_w,
                    "h": 40,
                }
            ],
        },
        {
            "label": "执行层",
            "h": 84,
            "go2": [
                {"id": "cl", "label": "闭环跟踪", "y": 32, "w": 112, "h": 40},
                {"id": "fwd", "label": "Sport 转发", "y": 32, "w": 112, "h": 40},
                {"id": "safe", "label": "限幅 / 急停", "y": 32, "w": 112, "h": 40},
            ],
        },
    ]

    bands, go2_nodes, edge_nodes = build_five_row(
        inner_x, inner_w, edge_x + edge_pad, edge_inner_w, go2_y + layer_top, layer_gap, rows
    )
    all_nodes = go2_nodes + edge_nodes
    g = {n.id: n for n in go2_nodes}
    e = {n.id: n for n in edge_nodes}

    edges = [
        Edge(anchor(g["sensors"], "right"), anchor(g["lio"], "left")),
        Edge(anchor(g["rgb"], "bottom"), anchor(g["bev"], "top"), "RGB 帧"),
        Edge(anchor(g["rgb"], "right"), anchor(g["rgb-up"], "top"), "关键帧", dashed=True),
        Edge(anchor(g["bev"], "right"), anchor(g["enu"], "left")),
        Edge(anchor(g["lio"], "bottom"), anchor(g["enu"], "top"), "位姿"),
        Edge(anchor(g["lio"], "bottom"), anchor(g["occ"], "top")),
        Edge(anchor(g["enu"], "right"), anchor(g["sem"], "left"), "BEV→ENU"),
        Edge(anchor(g["sem"], "bottom"), anchor(g["token"], "top")),
        Edge(anchor(g["rgb-up"], "right"), anchor(e["edge-model"], "left"), "RGB :9878", accent=True),
        Edge(anchor(g["token"], "right"), anchor(e["edge-model"], "left"), "Token :9876", accent=True),
        Edge(anchor(e["edge-planner"], "left"), anchor(g["ingress"], "right"), "控制面 JSON", accent=True),
        Edge(anchor(g["ingress"], "right"), anchor(g["bt"], "left")),
        Edge(anchor(g["frontier"], "right"), anchor(g["bt"], "left")),
        Edge(anchor(g["bt"], "bottom"), anchor(g["pct"], "top"), "NAV 触发", accent=True),
        Edge(anchor(g["pct"], "right"), anchor(g["scan"], "left"), "粗路径回传", label_dy=-14),
        Edge(anchor(g["bt"], "bottom"), anchor(g["scan"], "top"), "唯一执行 goal", accent=True),
        Edge(anchor(g["scan"], "bottom"), anchor(g["cl"], "top")),
        Edge(anchor(g["cl"], "right"), anchor(g["fwd"], "left")),
        Edge(anchor(g["fwd"], "right"), anchor(g["safe"], "left")),
        Edge(anchor(g["policy"], "right"), anchor(e["edge-model"], "left"), "策略归档"),
        Edge(anchor(g["bt"], "right"), anchor(e["edge-planner"], "left"), "feedback :9880", accent=True),
    ]

    go2_h = layer_top + sum(r["h"] for r in rows) + layer_gap * (len(rows) - 1) + 16
    domains = [
        (go2_x, go2_y, go2_w, go2_h, "端侧域 · GO2 + Orin NX（Humble 容器）"),
        (edge_x, go2_y, 280, go2_h, "边端域 · PC"),
    ]
    return render(1240, go2_y + go2_h + 24, "逻辑分层架构", bands, all_nodes, edges, domains)


def three_space_png() -> Image.Image:
    nodes = [
        Node("wm", "世界模型", 40, 48, 200, 56, "因果地图 / 仿真 / VLM"),
        Node("abi", "原子技能 ABI", 320, 40, 200, 52, "Navigate / Explore / ReturnHome"),
        Node("sched", "差分调度", 320, 108, 200, 44, "session / generation"),
        Node("kernel", "3D 行为规划引导内核", 600, 40, 220, 52, "可行通行 + 连续局部"),
        Node("body", "本体控制器", 600, 128, 220, 44, "限幅 / Sport"),
    ]
    n = {x.id: x for x in nodes}
    edges = [
        Edge(anchor(n["wm"], "right"), anchor(n["abi"], "left"), "OS2 决策翻译", accent=True, label_dy=-12),
        Edge(anchor(n["abi"], "right"), anchor(n["kernel"], "left"), "OS1 行为翻译", accent=True, label_dy=-12),
        Edge(anchor(n["kernel"], "left"), anchor(n["wm"], "right"), "OS3 版本化物料", dashed=True, label_dy=28),
        Edge(anchor(n["sched"], "top"), anchor(n["abi"], "bottom")),
        Edge(anchor(n["kernel"], "bottom"), anchor(n["body"], "top")),
    ]
    domains = [
        (24, 32, 232, 160, "世界空间"),
        (304, 32, 232, 160, "符号空间"),
        (584, 32, 252, 160, "度量空间"),
    ]
    return render(860, 220, "三空间交互中间件 OS", [], nodes, edges, domains)


def dual_mode_png() -> Image.Image:
    nodes = [
        Node("a", "模式 A", 40, 56, 150, 52, "Frontier 探索"),
        Node("b", "模式 B", 40, 156, 150, 52, "边端结构化任务"),
        Node("bt", "编排层 BT", 240, 100, 160, 64, "唯一写执行 goal"),
        Node("pct", "PCT 全局", 500, 56, 130, 48),
        Node("scan", "SCAN 局部", 500, 176, 130, 48),
        Node("exec", "执行层", 500, 256, 130, 48, "闭环 → Sport"),
    ]
    n = {x.id: x for x in nodes}
    edges = [
        Edge(anchor(n["a"], "right"), anchor(n["bt"], "left")),
        Edge(anchor(n["b"], "right"), anchor(n["bt"], "left")),
        Edge(anchor(n["bt"], "right"), anchor(n["pct"], "left"), "NAV 时触发", accent=True, label_dy=-14),
        Edge(anchor(n["pct"], "left"), anchor(n["bt"], "right"), "粗路径回传", label_dy=14),
        Edge(anchor(n["bt"], "right"), anchor(n["scan"], "left"), "执行 goal", accent=True, label_dy=-14),
        Edge(anchor(n["scan"], "bottom"), anchor(n["exec"], "top")),
    ]
    return render(820, 340, "双模式任务源", [], nodes, edges, [])


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    logic_layer_png().save(OUT_DIR / "架构_逻辑分层.png", "PNG")
    dual_mode_png().save(OUT_DIR / "架构_双模式.png", "PNG")
    three_space_png().save(OUT_DIR / "架构_三空间.png", "PNG")
    print(f"Wrote {OUT_DIR / '架构_逻辑分层.png'}")
    print(f"Wrote {OUT_DIR / '架构_双模式.png'}")
    print(f"Wrote {OUT_DIR / '架构_三空间.png'}")


if __name__ == "__main__":
    main()
