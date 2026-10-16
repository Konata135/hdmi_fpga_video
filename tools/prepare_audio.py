#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prepare_audio.py -- 把音乐转成开发板可读的音轨文件（*.aud）。

输出格式（自定容器，见 doc/TF卡文件格式.md）：
    512 字节头 + 原始 PCM（48 kHz / 16 bit / 立体声 / 小端）+ 补齐到 512 字节
    头里带场景编号，FPGA 靠它把音轨和图片配成一对。

为什么不用 WAV：
    WAV 头只有 44 字节，数据不是扇区对齐的，FPGA 连续流式读取时要做字节相位
    记账；本容器把数据固定放在 512 字节边界之后，流式读取不需要任何对齐处理。

解码方式：
    优先调用 ffmpeg（MP3 / FLAC / M4A 等都能吃），找不到 ffmpeg 时退回到
    Python 自带的 wave 模块，此时只支持 WAV 输入。

用法：
    python prepare_audio.py 歌曲.mp3 -s 0
    python prepare_audio.py 音乐目录 -o aud_out --start-scene 0
"""

import argparse
import os
import struct
import subprocess
import sys
import wave

import numpy as np

MAGIC = b"MMAU"
VERSION = 1
HEADER_BYTES = 512
SECTOR = 512

RATE = 48000
CHANNELS = 2
BITS = 16

AUDIO_SUFFIXES = (".wav", ".mp3", ".flac", ".m4a", ".aac", ".ogg", ".wma", ".aiff", ".aif")

# 与 prepare_images.py / write_card.py 保持同一套默认路径
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


# ---------------------------------------------------------------------------
# ffmpeg 探测
# ---------------------------------------------------------------------------
def find_ffmpeg(explicit):
    if explicit:
        if os.path.isfile(explicit):
            return explicit
        print("[警告] 指定的 ffmpeg 不存在：%s" % explicit)

    from shutil import which
    found = which("ffmpeg")
    if found:
        return found

    # 常见位置，按存在顺序试
    candidates = [
        r"D:\oopz\oopz\ffmpeg.exe",
        r"C:\ffmpeg\bin\ffmpeg.exe",
        r"C:\Program Files\ffmpeg\bin\ffmpeg.exe",
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def decode_with_ffmpeg(ffmpeg, src):
    cmd = [
        ffmpeg, "-v", "error", "-i", src,
        "-ac", str(CHANNELS), "-ar", str(RATE),
        "-f", "s16le", "-acodec", "pcm_s16le", "pipe:1",
    ]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode("utf-8", "ignore").strip()[:300])
    return np.frombuffer(proc.stdout, dtype="<i2")


def decode_with_wave(src):
    with wave.open(src, "rb") as wf:
        ch = wf.getnchannels()
        width = wf.getsampwidth()
        rate = wf.getframerate()
        raw = wf.readframes(wf.getnframes())

    if width == 1:                      # 8 位 WAV 是无符号
        data = (np.frombuffer(raw, dtype=np.uint8).astype(np.float32) - 128.0) / 128.0
    elif width == 2:
        data = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    elif width == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        val = (b[:, 0].astype(np.int32)
               | (b[:, 1].astype(np.int32) << 8)
               | (b[:, 2].astype(np.int32) << 16))
        val = np.where(val >= 0x800000, val - 0x1000000, val)
        data = val.astype(np.float32) / 8388608.0
    else:
        raise RuntimeError("只支持 8/16/24 位 WAV，或安装 ffmpeg 处理其它格式")

    data = data.reshape(-1, ch)

    # 声道：单声道复制成两路，多声道只取前两路
    if ch == 1:
        data = np.repeat(data, 2, axis=1)
    elif ch > 2:
        data = data[:, :2]

    # 采样率：线性插值重采样
    if rate != RATE:
        n_out = int(round(data.shape[0] * RATE / float(rate)))
        idx = np.linspace(0.0, data.shape[0] - 1.0, n_out, dtype=np.float64)
        lo = np.floor(idx).astype(np.int64)
        hi = np.minimum(lo + 1, data.shape[0] - 1)
        frac = (idx - lo).astype(np.float32)[:, None]
        data = data[lo] * (1.0 - frac) + data[hi] * frac

    # 转回 16 位，带限幅
    out = np.clip(data, -1.0, 1.0) * 32767.0
    return np.round(out).astype("<i2").reshape(-1)


def pack_container(samples, scene):
    frames = samples.size // CHANNELS
    payload = samples.astype("<i2").tobytes()
    padded = (len(payload) + SECTOR - 1) // SECTOR * SECTOR

    header = bytearray(HEADER_BYTES)
    # 0x00 4s magic | 0x04 H version | 0x06 H scene | 0x08 I rate
    # 0x0C H channels | 0x0E H bits | 0x10 I frames | 0x14 I data bytes
    struct.pack_into("<4sHHIHHII", header, 0,
                     MAGIC, VERSION, scene, RATE, CHANNELS, BITS, frames, len(payload))
    struct.pack_into("<I", header, 0x18, HEADER_BYTES)
    return bytes(header) + payload + b"\x00" * (padded - len(payload)), frames, padded


def convert_one(src, dst, scene, ffmpeg):
    if ffmpeg:
        samples = decode_with_ffmpeg(ffmpeg, src)
    else:
        if not src.lower().endswith(".wav"):
            raise RuntimeError("没有 ffmpeg，只能处理 WAV 输入")
        samples = decode_with_wave(src)

    blob, frames, data_bytes = pack_container(samples, scene)
    with open(dst, "wb") as fh:
        fh.write(blob)
    return frames, data_bytes, len(blob)


def collect_inputs(paths):
    files = []
    for p in paths:
        if os.path.isdir(p):
            for name in sorted(os.listdir(p)):
                if name.lower().endswith(AUDIO_SUFFIXES):
                    files.append(os.path.join(p, name))
        elif os.path.isfile(p):
            files.append(p)
        else:
            print("[跳过] 找不到：%s" % p)
    return files


def announce_decoder(ffmpeg):
    if ffmpeg:
        print("使用 ffmpeg：%s" % ffmpeg)
    else:
        print("未找到 ffmpeg，只能处理 WAV 输入（16 位/24 位/单声道均可）")


def convert_all(files, outdir, start_scene, ffmpeg):
    os.makedirs(outdir, exist_ok=True)
    ok = 0
    for idx, src in enumerate(files):
        scene = start_scene + idx
        stem = os.path.splitext(os.path.basename(src))[0]
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in stem)
        dst = os.path.join(outdir, "%02d_%s.aud" % (scene, safe))
        try:
            frames, data_bytes, total = convert_one(src, dst, scene, ffmpeg)
        except Exception as exc:  # noqa: BLE001
            print("[失败] %s : %s" % (src, exc))
            continue

        print("OK 场景%d  %s -> %s" % (scene, os.path.basename(src), os.path.basename(dst)))
        print("      %d 帧（%.1f 秒）  PCM %d 字节  文件 %d 字节"
              % (frames, frames / float(RATE), data_bytes, total))
        ok += 1

    print()
    print("共转换 %d 个音轨，输出目录：%s" % (ok, os.path.abspath(outdir)))
    return ok


def pause():
    """双击运行时控制台会一闪而过，这里停一下让人能看清结果。"""
    try:
        input("\n按回车键退出…")
    except EOFError:
        pass


def double_click_mode():
    """没有任何命令行参数时的行为：用固定的素材目录，双击即可完成转换。"""
    src = os.path.join(ROOT, "素材", "音乐")
    out = os.path.join(ROOT, "aud_out")
    os.makedirs(src, exist_ok=True)

    print("=" * 64)
    print("音轨转换：转成 48 kHz / 16 位 / 立体声（*.aud）")
    print("=" * 64)
    print("素材目录：%s" % src)
    print()

    ffmpeg = find_ffmpeg(None)
    announce_decoder(ffmpeg)
    print()

    files = collect_inputs([src])
    if not files:
        print("这个目录还是空的。")
        print("把要用的音乐复制进去（MP3 / WAV / FLAC 等），")
        print("然后**再双击一次本脚本**，就会自动全部转换。")
        print()
        print("配对规则：目录里第 1 个文件配第 1 张图，第 2 个配第 2 张，依此类推。")
        pause()
        return 0

    print("发现 %d 个文件，按文件名排序后依次转换……" % len(files))
    print()
    convert_all(files, out, 0, ffmpeg)
    pause()
    return 0


def main():
    ap = argparse.ArgumentParser(description="音乐转为 48kHz 16bit 立体声音轨（*.aud）")
    ap.add_argument("inputs", nargs="*", help="音频文件或目录；不填则使用 素材/音乐 目录")
    ap.add_argument("-o", "--outdir", default=os.path.join(ROOT, "aud_out"),
                    help="输出目录（默认 <工程根>/aud_out）")
    ap.add_argument("-s", "--start-scene", type=int, default=0, help="起始场景编号（默认 0）")
    ap.add_argument("--ffmpeg", default=None, help="ffmpeg 路径（默认自动查找）")
    args = ap.parse_args()

    if not args.inputs:
        return double_click_mode()

    files = collect_inputs(args.inputs)
    if not files:
        print("没有找到可转换的音频。")
        return 1

    ffmpeg = find_ffmpeg(args.ffmpeg)
    announce_decoder(ffmpeg)
    convert_all(files, args.outdir, args.start_scene, ffmpeg)
    return 0


if __name__ == "__main__":
    sys.exit(main())

