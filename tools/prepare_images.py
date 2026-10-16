#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prepare_images.py -- 把任意常见格式的图片转成开发板可读的 BMP。

目标格式（与赛题要求一致）：
    640 x 480 / 24-bit RGB / 非压缩 / Windows BITMAPINFOHEADER

用法：
    python prepare_images.py 图片1 图片2 ... [-o 输出目录]
    python prepare_images.py 输入目录 [-o 输出目录]

转换后会打印每张图的实际头字段，便于核对：
    尺寸、位深、压缩方式、像素数据偏移、行是否补位
"""

import argparse
import os
import struct
import sys

from PIL import Image

TARGET_W = 640
TARGET_H = 480
SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp", ".webp", ".gif", ".tif", ".tiff")

# 输出目录默认落在工程根目录下，和 write_card.py 的默认值保持一致；
# 否则在别的目录里执行脚本，"转出来的图"和"写卡工具找的图"会对不上。
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def describe_bmp(path):
    """读出 BMP 头里的关键字段，用来确认真的符合要求。"""
    with open(path, "rb") as fh:
        head = fh.read(54)
    if len(head) < 54 or head[0:2] != b"BM":
        raise ValueError("输出不是合法的 BMP 文件")

    (file_size,) = struct.unpack_from("<I", head, 0x02)
    (data_offset,) = struct.unpack_from("<I", head, 0x0A)
    (dib_size,) = struct.unpack_from("<I", head, 0x0E)
    (width,) = struct.unpack_from("<i", head, 0x12)
    (height,) = struct.unpack_from("<i", head, 0x16)
    (planes,) = struct.unpack_from("<H", head, 0x1A)
    (bpp,) = struct.unpack_from("<H", head, 0x1C)
    (compression,) = struct.unpack_from("<I", head, 0x1E)

    row_bytes = ((width * bpp // 8) + 3) // 4 * 4
    return {
        "file_size": file_size,
        "data_offset": data_offset,
        "dib_size": dib_size,
        "width": width,
        "height": height,
        "planes": planes,
        "bpp": bpp,
        "compression": compression,
        "row_bytes": row_bytes,
        "row_padding": row_bytes - width * bpp // 8,
        "bottom_up": height > 0,
    }


def convert_one(src, dst):
    with Image.open(src) as im:
        # 统一成 24 位 RGB，丢掉 alpha / 调色板 / 16 位等变体
        rgb = im.convert("RGB")
        if rgb.size != (TARGET_W, TARGET_H):
            rgb = rgb.resize((TARGET_W, TARGET_H), Image.Resampling.LANCZOS)
        rgb.save(dst, "BMP")
    return describe_bmp(dst)


def collect_inputs(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            for name in sorted(os.listdir(p)):
                if name.lower().endswith(SUFFIXES):
                    files.append(os.path.join(p, name))
        elif os.path.isfile(p):
            files.append(p)
        else:
            print("[跳过] 找不到：%s" % p)
    return files


def convert_all(files, outdir):
    # 按输入顺序编号，保证 FPGA 扫描到的先后与这里的顺序一致
    os.makedirs(outdir, exist_ok=True)
    ok = 0
    for idx, src in enumerate(files):
        stem = os.path.splitext(os.path.basename(src))[0]
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in stem)
        dst = os.path.join(outdir, "%02d_%s.bmp" % (idx, safe))
        try:
            info = convert_one(src, dst)
        except Exception as exc:  # noqa: BLE001
            print("[失败] %s : %s" % (src, exc))
            continue

        warnings = []
        if info["bpp"] != 24:
            warnings.append("位深不是 24")
        if info["compression"] != 0:
            warnings.append("不是非压缩")
        if (info["width"], abs(info["height"])) != (TARGET_W, TARGET_H):
            warnings.append("尺寸不是 %dx%d" % (TARGET_W, TARGET_H))
        if info["data_offset"] != 54:
            warnings.append("像素数据偏移不是 54")

        flag = "OK " if not warnings else "!! "
        print("%s%02d  %s -> %s" % (flag, idx, os.path.basename(src), os.path.basename(dst)))
        print("      %dx%d  %d位  压缩=%d  数据偏移=%d  行字节=%d  行补位=%d  %s"
              % (info["width"], abs(info["height"]), info["bpp"], info["compression"],
                 info["data_offset"], info["row_bytes"], info["row_padding"],
                 "自下而上" if info["bottom_up"] else "自上而下"))
        if warnings:
            print("      [警告] " + "；".join(warnings))
        ok += 1

    print()
    print("共转换 %d 张，输出目录：%s" % (ok, os.path.abspath(outdir)))
    print("文件名已按顺序编号，FPGA 扫描到的顺序就是这里的编号顺序。")
    return ok


def pause():
    """双击运行时控制台会一闪而过，这里停一下让人能看清结果。"""
    try:
        input("\n按回车键退出…")
    except EOFError:
        pass


def double_click_mode():
    """没有任何命令行参数时的行为：用固定的素材目录，双击即可完成转换。"""
    src = os.path.join(ROOT, "素材", "图片")
    out = os.path.join(ROOT, "bmp_out")
    os.makedirs(src, exist_ok=True)

    print("=" * 64)
    print("图片转换：转成 640x480 / 24 位 / 非压缩 BMP")
    print("=" * 64)
    print("素材目录：%s" % src)
    print()

    files = collect_inputs([src])
    if not files:
        print("这个目录还是空的。")
        print("把要用的图片复制进去（JPG / PNG / WebP / GIF / BMP 都行），")
        print("然后**再双击一次本脚本**，就会自动全部转换。")
        print()
        print("播放顺序 = 文件名排序顺序，想调整就改文件名。")
        pause()
        return 0

    print("发现 %d 个文件，按文件名排序后依次转换……" % len(files))
    print()
    convert_all(files, out)
    pause()
    return 0


def main():
    ap = argparse.ArgumentParser(description="图片批量转为 640x480 24 位非压缩 BMP")
    ap.add_argument("inputs", nargs="*", help="图片文件或目录；不填则使用 素材/图片 目录")
    ap.add_argument("-o", "--outdir", default=os.path.join(ROOT, "bmp_out"),
                    help="输出目录（默认 <工程根>/bmp_out）")
    args = ap.parse_args()

    if not args.inputs:
        return double_click_mode()

    files = collect_inputs(args.inputs)
    if not files:
        print("没有找到可转换的图片。")
        return 1
    convert_all(files, args.outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())

