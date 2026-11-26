#!/usr/bin/env python3
"""Generate quantized 16x16 luma + 8x8 (R-B) chroma traffic-sign templates for ESP32.

Reads VietMap Live iOS HUD sign assets (`lib/assets/hud/*.{png,jpg}`) from the
extracted iOS IPA, box-downsamples each image onto:
  - 16x16 BT.601 luma grid (256 cells, exact match with ESP32 TJpgDec callback)
  - 8x8 signed (R - B) chroma grid (64 cells, separating red prohibition slashes
    from grey end-of-restriction signs and blue area/toll signs)

Produces a 320-element int8 zero-mean normalized feature vector per template,
verifies inter-class NCC margins (including JPEG re-encoding), and writes
`firmware/vmhud-esp32-s3/lib/vmhud/traffic_sign_templates.h`.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import tempfile
from pathlib import Path

# Wire enum codes matching VietMapSignCode in cph_telemetry.h and Swift VietMapSignKind.
SIGN_NONE = 0x00
SIGN_NO_LEFT_TURN = 0x01
SIGN_NO_RIGHT_TURN = 0x02
SIGN_NO_LEFT_RIGHT_TURN = 0x03
SIGN_NO_UTURN = 0x04
SIGN_NO_LEFT_UTURN = 0x05
SIGN_NO_RIGHT_UTURN = 0x06
SIGN_NO_ENTRY = 0x07
SIGN_NO_ENTRY_NO_LEFT = 0x08
SIGN_NO_ENTRY_NO_RIGHT = 0x09

SIGN_NO_PASSING_START = 0x10
SIGN_NO_PASSING_END = 0x11
SIGN_RESIDENTIAL_START = 0x12
SIGN_RESIDENTIAL_END = 0x13
SIGN_NO_PARKING = 0x14
SIGN_TOLL_STATION = 0x15
SIGN_RAILWAY = 0x16
SIGN_TUNNEL = 0x17

SIGN_SPEED_CAMERA = 0x20
SIGN_TRAFFIC_CAMERA = 0x21

ASSET_TO_SIGN: dict[str, int] = {
    "noLeftTurn.png": SIGN_NO_LEFT_TURN,
    "noRightTurn.png": SIGN_NO_RIGHT_TURN,
    "noLeftTurnNoRightTurn.png": SIGN_NO_LEFT_RIGHT_TURN,
    "noUTurnLeft.png": SIGN_NO_UTURN,
    "noUTurnRight.png": SIGN_NO_UTURN,
    "noLeftTurnUTurn.png": SIGN_NO_LEFT_UTURN,
    "noRightTurnUTurn.png": SIGN_NO_RIGHT_UTURN,
    "noEntry.png": SIGN_NO_ENTRY,
    "noEntryNoLeftTurn.png": SIGN_NO_ENTRY_NO_LEFT,
    "noEntryNoRightTurn.png": SIGN_NO_ENTRY_NO_RIGHT,
    "no_passing_in.png": SIGN_NO_PASSING_START,
    "no_passing_out.png": SIGN_NO_PASSING_END,
    "residential_area_in.png": SIGN_RESIDENTIAL_START,
    "residential_area_out.png": SIGN_RESIDENTIAL_END,
    "noParkNoStop.png": SIGN_NO_PARKING,
    "parkRestrictionFull.png": SIGN_NO_PARKING,
    "parkRestrictionEven.jpg": SIGN_NO_PARKING,
    "parkRestrictionOdd.jpg": SIGN_NO_PARKING,
    "toll_fee_etc.png": SIGN_TOLL_STATION,
    "toll_fee_non_etc.png": SIGN_TOLL_STATION,
    "railway.png": SIGN_RAILWAY,
    "tunnel.png": SIGN_TUNNEL,
    "900.png": SIGN_TUNNEL,
    "speed_limit_checkpoint.png": SIGN_SPEED_CAMERA,
}

NEGATIVE_PROBES: tuple[str, ...] = (
    "vietmap_logo.png",
    "vietmap_logo_night.png",
    "car_por.png",
    "car_landscape.png",
    "app_icon_120.png",
    "highway_rest_area.png",
    "stop_lane.png",
    "888010.png",
    "888010_night.png",
    "888020.png",
    "888020_night.png",
    "888030.png",
    "888030_night.png",
    "888040.png",
    "888040_night.png",
    "888050.png",
    "888050_night.png",
    "888060.png",
    "888060_night.png",
    "888070.png",
    "888070_night.png",
    "900000.png",
    "900010.png",
    "900020.png",
    "900030.png",
    "900040.png",
    "900050.png",
)

MAC_DEC_C = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ImageIO/ImageIO.h>
#include <CoreGraphics/CoreGraphics.h>
#include <CoreServices/CoreServices.h>

static void dump_luma16_rb8(const uint8_t *rgba, int w, int h) {
    uint32_t luma_sum[256] = {0};
    uint32_t luma_cnt[256] = {0};
    int32_t  rb_sum[64] = {0};
    uint32_t rb_cnt[64] = {0};

    for (int y = 0; y < h; y++) {
        int dy16 = (y * 16) / h;
        int dy8  = (y * 8) / h;
        for (int x = 0; x < w; x++) {
            int dx16 = (x * 16) / w;
            int dx8  = (x * 8) / w;
            int idx = (y * w + x) * 4;
            int r = rgba[idx];
            int g = rgba[idx + 1];
            int b = rgba[idx + 2];
            uint32_t luma = (uint32_t)((77 * r + 150 * g + 29 * b) >> 8);
            luma_sum[dy16 * 16 + dx16] += luma;
            luma_cnt[dy16 * 16 + dx16] += 1;
            rb_sum[dy8 * 8 + dx8] += (r - b);
            rb_cnt[dy8 * 8 + dx8] += 1;
        }
    }
    for (int i = 0; i < 256; i++) {
        uint32_t v = luma_cnt[i] ? (luma_sum[i] / luma_cnt[i]) : 0;
        printf("%s%u", i ? "," : "", v);
    }
    printf("|");
    for (int i = 0; i < 64; i++) {
        int32_t v = rb_cnt[i] ? (rb_sum[i] / (int32_t)rb_cnt[i]) : 0;
        // Clamp R-B / 2 into [-127, 127]
        int32_t half = v / 2;
        if (half < -127) half = -127;
        if (half > 127) half = 127;
        printf("%s%d", i ? "," : "", (int)half);
    }
    printf("\n");
}

int main(int argc, char **argv) {
    if (argc < 2) return 1;
    bool jpeg_roundtrip = (argc >= 3 && strcmp(argv[2], "--jpeg") == 0);
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

    if (jpeg_roundtrip) {
        CGImageRef flat = CGBitmapContextCreateImage(c);
        CFMutableDataRef jdata = CFDataCreateMutable(NULL, 0);
        CGImageDestinationRef dst = CGImageDestinationCreateWithData(jdata, CFSTR("public.jpeg"), 1, NULL);
        float q = 0.65f;
        CFNumberRef qnum = CFNumberCreate(NULL, kCFNumberFloatType, &q);
        const void *keys[1] = { kCGImageDestinationLossyCompressionQuality };
        const void *vals[1] = { qnum };
        CFDictionaryRef opts = CFDictionaryCreate(NULL, keys, vals, 1, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
        CGImageDestinationAddImage(dst, flat, opts);
        CGImageDestinationFinalize(dst);
        CGImageSourceRef jsrc = CGImageSourceCreateWithData(jdata, NULL);
        CGImageRef jimg = CGImageSourceCreateImageAtIndex(jsrc, 0, NULL);
        memset(buf, 0, (size_t)w * (size_t)h * 4u);
        CGContextDrawImage(c, CGRectMake(0, 0, w, h), jimg);
        CGImageRelease(jimg);
        CFRelease(jsrc);
        CFRelease(opts);
        CFRelease(qnum);
        CFRelease(dst);
        CFRelease(jdata);
        CGImageRelease(flat);
    }

    dump_luma16_rb8(buf, w, h);
    free(buf);
    CGContextRelease(c);
    CGColorSpaceRelease(rgb);
    CGImageRelease(img);
    CFRelease(src);
    CFRelease(url);
    return 0;
}
"""


def build_feature_vec(luma256: list[int], rb64: list[int]) -> tuple[list[int], float]:
    """Mirror exact integer/float quantization that runs on ESP32."""
    mean = sum(luma256) / 256.0
    centered_luma = [float(x) - mean for x in luma256]
    # Combine centered luma [-255..255] with 1.5x weighted R-B half-chroma [-127..127]
    raw: list[float] = centered_luma + [float(c) * 1.5 for c in rb64]
    mabs = max(abs(x) for x in raw)
    if mabs < 2.0:
        raise ValueError("Blank/uniform traffic sign image")
    scale = 127.0 / mabs
    vec = [max(-127, min(127, int(round(x * scale)))) for x in raw]
    nsq = sum(v * v for v in vec)
    return vec, 1.0 / math.sqrt(nsq)


def ncc(v1: list[int], inv1: float, v2: list[int], inv2: float) -> float:
    return sum(a * b for a, b in zip(v1, v2)) * inv1 * inv2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--hud-dir", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as td:
        c_path = os.path.join(td, "dec_sign.c")
        bin_path = os.path.join(td, "dec_sign")
        Path(c_path).write_text(MAC_DEC_C, encoding="utf-8")
        subprocess.check_call(
            [
                "clang",
                "-O2",
                c_path,
                "-framework",
                "ImageIO",
                "-framework",
                "CoreGraphics",
                "-framework",
                "CoreServices",
                "-framework",
                "CoreFoundation",
                "-o",
                bin_path,
            ]
        )

        def decode_asset(path: str, jpeg: bool = False) -> tuple[list[int], list[int]]:
            cmd = [bin_path, path] + (["--jpeg"] if jpeg else [])
            out = subprocess.check_output(cmd, text=True).strip()
            l_part, c_part = out.split("|")
            return [int(x) for x in l_part.split(",")], [int(x) for x in c_part.split(",")]

        raw_entries: list[tuple[int, str, list[int], float]] = []
        jpeg_probes: list[tuple[int, str, list[int], float]] = []
        for fname, code in sorted(ASSET_TO_SIGN.items()):
            p = os.path.join(args.hud_dir, fname)
            if not os.path.isfile(p):
                raise SystemExit(f"Missing expected HUD sign asset: {p}")
            luma, rb = decode_asset(p, jpeg=False)
            vec, inv = build_feature_vec(luma, rb)
            raw_entries.append((code, fname, vec, inv))

            jluma, jrb = decode_asset(p, jpeg=True)
            jvec, jinv = build_feature_vec(jluma, jrb)
            jpeg_probes.append((code, fname, jvec, jinv))

        # Also decode non-sign assets (weather, zone pins, logo, car) as negative probes!
        negatives: list[tuple[str, list[int], float]] = []
        for neg_name in NEGATIVE_PROBES:
            np = os.path.join(args.hud_dir, neg_name)
            if os.path.isfile(np):
                try:
                    nl, nrb = decode_asset(np, jpeg=True)
                    nv, ninv = build_feature_vec(nl, nrb)
                    negatives.append((neg_name, nv, ninv))
                except ValueError:
                    pass

    # Deduplicate near-identical templates sharing the same code (NCC > 0.992)
    templates: list[tuple[int, str, list[int], float]] = []
    for code, fname, vec, inv in raw_entries:
        if any(tc == code and ncc(vec, inv, tv, tinv) > 0.992 for tc, _, tv, tinv in templates):
            continue
        templates.append((code, fname, vec, inv))

    # Verify JPEG q=65 classification accuracy and worst inter-class margin!
    worst_true_score = 2.0
    worst_margin = 2.0
    worst_pair = ""
    for want_code, fname, jvec, jinv in jpeg_probes:
        class_best: dict[int, float] = {}
        for tc, _, tv, tinv in templates:
            s = ncc(jvec, jinv, tv, tinv)
            if s > class_best.get(tc, -2.0):
                class_best[tc] = s
        best_code = max(class_best, key=lambda k: class_best[k])
        best_score = class_best[best_code]
        runner_up = max((v for k, v in class_best.items() if k != best_code), default=-2.0)
        margin = best_score - runner_up
        if best_code != want_code:
            raise SystemExit(f"MISMATCH on JPEG {fname}: want 0x{want_code:02X}, got 0x{best_code:02X} (score={best_score:.4f})")
        if best_score < worst_true_score:
            worst_true_score = best_score
        if margin < worst_margin:
            worst_margin = margin
            worst_pair = f"{fname} (0x{want_code:02X}, score={best_score:.4f}, margin={margin:.4f})"

    max_neg_score = -2.0
    max_neg_name = ""
    for nname, nv, ninv in negatives:
        s = max(ncc(nv, ninv, tv, tinv) for _, _, tv, tinv in templates)
        if s > max_neg_score:
            max_neg_score = s
            max_neg_name = nname

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "// SPDX-License-Identifier: AGPL-3.0-or-later",
        "// Auto-generated by scripts/gen_traffic_sign_templates.py — DO NOT EDIT.",
        "// Quantized 320-byte (16x16 luma + 8x8 R-B chroma) VietMap iOS HUD traffic-sign",
        "// descriptors for CMD 0x08 / CMD 0x41 prohibition, area, road, and camera icons.",
        "#pragma once",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "namespace vmhud {",
        "",
        "static constexpr size_t kTrafficSignLumaLen   = 256;",
        "static constexpr size_t kTrafficSignChromaLen = 64;",
        "static constexpr size_t kTrafficSignFeatLen   = kTrafficSignLumaLen + kTrafficSignChromaLen;",
        "",
        "struct TrafficSignTemplate {",
        "  uint8_t signCode;",
        "  float   invNorm;",
        "  int8_t  vec[kTrafficSignFeatLen];",
        "};",
        "",
        f"static constexpr size_t kTrafficSignTemplateCount = {len(templates)};",
        "",
        "static const TrafficSignTemplate kTrafficSignTemplates[kTrafficSignTemplateCount] = {",
    ]
    for code, fname, vec, inv in templates:
        lines.append(f"  // 0x{code:02X} ({fname})")
        lines.append(f"  {{0x{code:02X}u, {inv:.9g}f, {{{','.join(str(v) for v in vec)}}}}},")
    lines.extend(["};", "", "}  // namespace vmhud", ""])
    out_path.write_text("\n".join(lines), encoding="utf-8")

    print(
        f"Generated {len(templates)} traffic-sign templates from {len(raw_entries)} assets -> {out_path}\n"
        f"  JPEG q=65 min true score : {worst_true_score:.4f}\n"
        f"  JPEG q=65 min margin     : {worst_margin:.4f} [{worst_pair}]\n"
        f"  Max negative non-sign NCC: {max_neg_score:.4f} [{max_neg_name}]"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
