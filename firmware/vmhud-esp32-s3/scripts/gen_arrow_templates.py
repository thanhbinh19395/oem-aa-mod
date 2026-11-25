#!/usr/bin/env python3
"""Generate quantized 16x16 arrow templates for ESP32-S3 VietMap BLE HUD bridge.

Reads VietMap Live HUD direction PNGs (both `lib/assets/hud/directions/{dark,light}/`
and top-level `lib/assets/hud/{turn,roundabout_turn,arriver}_*.png`) from an extracted
IPA/APK, box-downsamples each to 16x16 grayscale using the exact integer BT.601 luma
formula used by the ESP32 TJpgDec callback, deduplicates pixel-identical bitmaps, and
writes `firmware/vmhud-esp32-s3/lib/vmhud/arrow_templates.h`.

Each template stores:
  - `icon`: Mazda HUD `MazdaIcon` code (1..60, matching `hud_send.cpp`)
  - `norm_sq`: precomputed sum of squares of the 256 int8 entries (avoids recomputing)
  - `vec[256]`: zero-mean quantized int8 feature descriptor in [-127, 127]

Usage (macOS with clang + ImageIO, or any OS with Pillow installed):
    python3 firmware/vmhud-esp32-s3/scripts/gen_arrow_templates.py \
        --hud-dir tmp/vietmap_ipa_extract/Payload/Runner.app/Frameworks/App.framework/flutter_assets/lib/assets/hud \
        --out firmware/vmhud-esp32-s3/lib/vmhud/arrow_templates.h
"""
from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path

ASSET_RE = re.compile(r"/directions/(dark|light)/(direction_[a-z_]+)\.png$")
TOP_RE = re.compile(r"^(arriver|arriver_left|arriver_right|turn_[a-z_]+|roundabout_turn_[a-z_]+)\.png$")

MODIFIERS = (
    "slight_left",
    "slight_right",
    "sharp_left",
    "sharp_right",
    "uturn",
    "straight",
    "left",
    "right",
)

# Exact Mazda HUD icon codes mirrored from mazda/patches/blmjcicarplay/hud/hud_send.cpp
HUD_STRAIGHT = 1
HUD_LEFT = 2
HUD_RIGHT = 3
HUD_SLIGHT_LEFT = 4
HUD_SLIGHT_RIGHT = 5
HUD_OFF_RAMP_RIGHT = 7
HUD_DESTINATION = 8
HUD_SHARP_RIGHT = 9
HUD_U_TURN_RIGHT = 10
HUD_SHARP_LEFT = 11
HUD_FLAG = 12
HUD_U_TURN_LEFT = 13
HUD_FORK_RIGHT = 14
HUD_FORK_LEFT = 15
HUD_MERGE_LEFT = 16
HUD_MERGE_RIGHT = 17
HUD_OFF_RAMP_LEFT = 30
HUD_DESTINATION_LEFT = 33
HUD_DESTINATION_RIGHT = 34
HUD_FLAG_LEFT = 35
HUD_FLAG_RIGHT = 36

MODIFIER_DEGREES = {
    "straight": 0,
    "slight_right": 45,
    "right": 90,
    "sharp_right": 135,
    "uturn": 180,
    "sharp_left": 225,
    "left": 270,
    "slight_left": 315,
}

TURN_LIKE = {
    "left": HUD_LEFT,
    "right": HUD_RIGHT,
    "slight_left": HUD_SLIGHT_LEFT,
    "slight_right": HUD_SLIGHT_RIGHT,
    "sharp_left": HUD_SHARP_LEFT,
    "sharp_right": HUD_SHARP_RIGHT,
    "uturn": HUD_U_TURN_LEFT,  # RHT (Vietnam): U-turn curves left
    "straight": HUD_STRAIGHT,
    "": HUD_STRAIGHT,
}

BY_MANEUVER = {
    "turn": TURN_LIKE,
    "end_of_road": TURN_LIKE,
    "on_ramp": TURN_LIKE,
    "continue": TURN_LIKE,
    "notification": TURN_LIKE,
    "invalid": TURN_LIKE,
    "new_name": TURN_LIKE,
    "merge": {
        "left": HUD_MERGE_LEFT,
        "slight_left": HUD_MERGE_LEFT,
        "right": HUD_MERGE_RIGHT,
        "slight_right": HUD_MERGE_RIGHT,
        "straight": HUD_STRAIGHT,
        "": HUD_STRAIGHT,
    },
    "fork": {
        "left": HUD_FORK_LEFT,
        "slight_left": HUD_FORK_LEFT,
        "right": HUD_FORK_RIGHT,
        "slight_right": HUD_FORK_RIGHT,
        "straight": HUD_STRAIGHT,
        "": HUD_STRAIGHT,
    },
    "off_ramp": {
        "left": HUD_OFF_RAMP_LEFT,
        "slight_left": HUD_OFF_RAMP_LEFT,
        "right": HUD_OFF_RAMP_RIGHT,
        "slight_right": HUD_OFF_RAMP_RIGHT,
        "straight": HUD_STRAIGHT,
        "": HUD_OFF_RAMP_RIGHT,
    },
    "arrive": {
        "left": HUD_DESTINATION_LEFT,
        "right": HUD_DESTINATION_RIGHT,
        "straight": HUD_DESTINATION,
        "": HUD_DESTINATION,
    },
    "depart": {
        "left": HUD_FLAG_LEFT,
        "right": HUD_FLAG_RIGHT,
        "straight": HUD_FLAG,
        "": HUD_FLAG,
    },
    "flag": {"": HUD_FLAG},
    "close": {"": HUD_FLAG},
    "updown": {"": HUD_STRAIGHT},
}


def roundabout_icon(degrees: int) -> int:
    """RHT roundabout glyph bank (37..48), exact mirror of hud_send.cpp."""
    d = (180 - degrees) % 360
    return ((d + 15) // 30) % 12 + 37


def parse_name(name: str) -> tuple[str, str]:
    body = name[len("direction_") :] if name.startswith("direction_") else name
    if body.endswith("_night"):
        body = body[:-6]
    if body.startswith("arriver"):
        body = "arrive" + body[7:]
    if body.startswith("roundabout_turn_"):
        body = "roundabout_" + body[len("roundabout_turn_") :]
    mod = ""
    for m in MODIFIERS:
        if body == m:
            return "", m
        if body.endswith("_" + m):
            mod = m
            body = body[: -len(m) - 1]
            break
    if body == "notificaiton":
        body = "notification"
    return body, mod


def icon_for(maneuver: str, modifier: str) -> int:
    if maneuver in ("roundabout", "rotary"):
        return roundabout_icon(MODIFIER_DEGREES.get(modifier, 0))
    table = BY_MANEUVER.get(maneuver, TURN_LIKE)
    return table.get(modifier, TURN_LIKE.get(modifier, table.get("", HUD_STRAIGHT)))


def to_vec(px: list[int]) -> list[int] | None:
    mean = sum(px) / 256.0
    centered = [x - mean for x in px]
    mabs = max(abs(x) for x in centered)
    if mabs < 1.0:
        return None
    return [max(-127, min(127, int(round(x * 127.0 / mabs)))) for x in centered]


def ncc(a: list[int], b: list[int]) -> float:
    dot = sum(x * y for x, y in zip(a, b))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(x * x for x in b))
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return dot / (na * nb)


MAC_HELPER_C = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ImageIO/ImageIO.h>
#include <CoreGraphics/CoreGraphics.h>
#include <CoreServices/CoreServices.h>

static void dump_box16(const uint8_t *rgba, int w, int h) {
    uint32_t sum[256] = {0};
    uint32_t cnt[256] = {0};
    for (int y = 0; y < h; y++) {
        int dy = (y * 16) / h;
        for (int x = 0; x < w; x++) {
            int dx = (x * 16) / w;
            int idx = (y * w + x) * 4;
            uint32_t luma = (77u * rgba[idx] + 150u * rgba[idx + 1] + 29u * rgba[idx + 2]) >> 8;
            sum[dy * 16 + dx] += luma;
            cnt[dy * 16 + dx] += 1;
        }
    }
    for (int i = 0; i < 256; i++) {
        uint32_t v = cnt[i] ? (sum[i] / cnt[i]) : 0;
        printf("%s%u", i ? "," : "", v);
    }
    printf("\n");
}

int main(int argc, char **argv) {
    if (argc < 2) return 1;
    CFURLRef url = CFURLCreateFromFileSystemRepresentation(NULL, (const UInt8*)argv[1], strlen(argv[1]), false);
    CGImageSourceRef src = CGImageSourceCreateWithURL(url, NULL);
    if (!src) return 2;
    CGImageRef img = CGImageSourceCreateImageAtIndex(src, 0, NULL);
    if (!img) return 3;
    int w = (int)CGImageGetWidth(img);
    int h = (int)CGImageGetHeight(img);
    CGColorSpaceRef rgb = CGColorSpaceCreateDeviceRGB();
    uint8_t *buf = calloc((size_t)w * (size_t)h * 4u, 1);
    CGContextRef c = CGBitmapContextCreate(buf, w, h, 8, w * 4, rgb, kCGImageAlphaNoneSkipLast);
    CGContextSetRGBFillColor(c, 0, 0, 0, 1);
    CGContextFillRect(c, CGRectMake(0, 0, w, h));
    CGContextDrawImage(c, CGRectMake(0, 0, w, h), img);
    dump_box16(buf, w, h);
    free(buf);
    CGContextRelease(c);
    CGColorSpaceRelease(rgb);
    CGImageRelease(img);
    CFRelease(src);
    CFRelease(url);
    return 0;
}
"""


def make_loader(tmpdir: str):
    try:
        from PIL import Image  # type: ignore

        def load_pil(path: str) -> list[int]:
            img = Image.open(path).convert("RGBA")
            flat = Image.alpha_composite(Image.new("RGBA", img.size, (0, 0, 0, 255)), img).convert("RGB")
            w, h = flat.size
            raw = flat.tobytes()
            sums = [0] * 256
            cnts = [0] * 256
            for y in range(h):
                dy = (y * 16) // h
                for x in range(w):
                    dx = (x * 16) // w
                    off = (y * w + x) * 3
                    luma = (77 * raw[off] + 150 * raw[off + 1] + 29 * raw[off + 2]) >> 8
                    sums[dy * 16 + dx] += luma
                    cnts[dy * 16 + dx] += 1
            return [sums[i] // cnts[i] if cnts[i] else 0 for i in range(256)]

        return load_pil
    except ImportError:
        cpath = os.path.join(tmpdir, "imgbox.c")
        bpath = os.path.join(tmpdir, "imgbox")
        Path(cpath).write_text(MAC_HELPER_C, encoding="utf-8")
        subprocess.check_call(
            [
                "clang",
                "-O2",
                cpath,
                "-framework",
                "ImageIO",
                "-framework",
                "CoreGraphics",
                "-framework",
                "CoreFoundation",
                "-o",
                bpath,
            ]
        )

        def load_mac(path: str) -> list[int]:
            out = subprocess.check_output([bpath, path]).decode().strip()
            return [int(x) for x in out.split(",")]

        return load_mac


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hud-dir", required=True, help="Path to flutter_assets/lib/assets/hud")
    parser.add_argument("--out", required=True, help="Output C++ header path")
    args = parser.parse_args()

    hud_dir = args.hud_dir
    with tempfile.TemporaryDirectory() as td:
        load_box = make_loader(td)
        entries: list[tuple[str, int, list[int]]] = []

        dir_root = os.path.join(hud_dir, "directions")
        for root, _, files in os.walk(dir_root):
            for fname in sorted(files):
                full = os.path.join(root, fname)
                m = ASSET_RE.search(full.replace("\\", "/"))
                if not m:
                    continue
                theme, name = m.group(1), m.group(2)
                man, mod = parse_name(name)
                ic = icon_for(man, mod)
                vec = to_vec(load_box(full))
                if vec is not None:
                    entries.append((f"{theme}/{name}", ic, vec))

        for fname in sorted(os.listdir(hud_dir)):
            m = TOP_RE.match(fname)
            if not m:
                continue
            full = os.path.join(hud_dir, fname)
            man, mod = parse_name(m.group(1))
            ic = icon_for(man, mod)
            # Top-level arriver.png / arriver_left.png / arriver_right.png are pixel-identical flags
            if m.group(1).startswith("arriver"):
                ic = HUD_DESTINATION
            vec = to_vec(load_box(full))
            if vec is not None:
                entries.append((f"hud/{m.group(1)}", ic, vec))

    # Deduplicate near-identical descriptors (same icon, NCC > 0.990)
    templates: list[tuple[int, str, list[int], int]] = []
    for label, ic, vec in entries:
        merged = False
        for t_ic, _t_lbl, t_vec, _t_nsq in templates:
            if t_ic == ic and ncc(vec, t_vec) > 0.990:
                merged = True
                break
        if not merged:
            nsq = sum(x * x for x in vec)
            templates.append((ic, label, vec, nsq))

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "// SPDX-License-Identifier: AGPL-3.0-or-later",
        "// Auto-generated by scripts/gen_arrow_templates.py — do not hand-edit.",
        "// Quantized 16x16 zero-mean direction descriptors mapping VietMap CMD 0x40",
        "// JPEG arrows to Mazda HUD MazdaIcon codes (1..60).",
        "#pragma once",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "namespace vmhud {",
        "",
        "struct ArrowTemplate {",
        "    uint8_t icon;",
        "    float   invNorm;",
        "    int8_t  vec[256];",
        "};",
        "",
        f"static constexpr size_t kArrowTemplateCount = {len(templates)};",
        "",
        "static const ArrowTemplate kArrowTemplates[kArrowTemplateCount] = {",
    ]
    for ic, label, vec, nsq in templates:
        inv_norm = 1.0 / math.sqrt(nsq)
        lines.append(f"    // icon={ic} ({label})")
        lines.append(f"    {{{ic}u, {inv_norm:.9g}f, {{")
        for row in range(0, 256, 16):
            chunk = ",".join(f"{v:4d}" for v in vec[row : row + 16])
            lines.append(f"        {chunk},")
        lines.append("    }},")
    lines += [
        "};",
        "",
        "} // namespace vmhud",
        "",
    ]
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Generated {len(templates)} deduplicated templates from {len(entries)} icons -> {out_path}")


if __name__ == "__main__":
    main()
