#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
write_card.py -- 把准备好的图片与音轨安全写入 TF 卡。

为什么需要它：
    FPGA 不解析文件系统，而是在卷内逐个扇区扫描文件头签名（'BM' / 'MMAU'）。
    因此"把文件删掉"对 FPGA 是无效的——旧文件的物理数据还留在扇区里，照样会被
    扫到，表现为显示错图或卡死。所以写卡必须由工具完成，不能手动拖拽。

本工具比官方样例的 sync_to_sd.py 多做一步：
    官方脚本只破坏"在文件系统里还看得见"的旧图片头。如果图片是手动删除的、或者
    是用别的工具删的，那些扇区里的旧签名依然存在，官方脚本清不掉。
    本工具在破坏签名并删除之后，还会用全 0 大文件把可用空间整体覆写一遍，强制
    文件系统重新分配这些扇区，从而盖掉任何历史残留；不需要管理员权限。

写入顺序：
    场景0 图片 → 场景0 音轨 → 场景1 图片 → 场景1 音轨 → …
    这样每个场景的两份数据各自连续，FPGA 扫描一次即可拿到全部起始扇区。

用法：
    python write_card.py --no-audio --dry-run     # 只看计划（盘符默认 G:）
    python write_card.py --no-audio               # 真写，会先让你输 yes
    python write_card.py H: --no-audio            # 指定别的盘符（会警告）

这台机器上 TF 卡固定是 G:，所以盘符不用写；只有拿别的卡/读卡器时才需要显式指定，
指定成别的盘符会打印一行警告，提醒你别写错设备。

双击运行时没有任何参数，这时只打印用法并等你按回车，不会碰任何卡。
"""

import argparse
import glob
import os
import shutil
import sys

IMAGE_MAGIC = b"BM"
AUDIO_MAGIC = b"MMAU"
# 本机 TF 卡的固定盘符：不写盘符就默认用它，写成别的会打印警告
DEFAULT_DRIVE = "G:"
# 比这个还大的卷一律当成数据盘/系统盘，不允许写（除非显式 --force-drive）
MAX_CARD_GB   = 64
KEEP = ("System Volume Information", "$RECYCLE.BIN", "System Volume Information")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def log(msg):
    print(msg, flush=True)


def normalise_drive(raw):
    """盘符（E:）补成 E:\\；普通目录原样保留，方便用目录模拟卡做测试。"""
    d = raw.rstrip("\\/")
    if len(d) == 1 and d.isalpha():
        return d.upper() + ":\\"
    if len(d) == 2 and d[1] == ":" and d[0].isalpha():
        return d.upper() + "\\"
    return d + os.sep


def guard(drive, assume_yes, dry_run, is_default_drive, force_drive):
    """动卡之前的最后一道闸。这个工具会清空整只卷，所以这里宁可啰嗦。"""
    # 只拦"盘符根目录"是系统盘的情况；测试时把卡模拟成普通目录也能跑
    abspath = os.path.abspath(drive)
    is_drive_root = os.path.dirname(abspath) == abspath
    if is_drive_root and abspath.upper().startswith("C:"):
        raise SystemExit("[致命] 拒绝了 C 盘根目录。请指定 TF 卡所在盘符。")
    if not os.path.exists(drive):
        raise SystemExit("[致命] 找不到驱动器 %s，卡插好了吗？" % drive)

    # 盘符安全闸：默认只认本机固定的 TF 卡盘符，而且卷不能大得像数据盘。
    # 目的就是拦住"手滑敲成 E: 然后一路 yes"这种事——那只卷会被根目录全删。
    total_gb = None
    try:
        total_gb = shutil.disk_usage(drive).total / 1e9
    except OSError:
        pass

    risky = []
    if not is_default_drive:
        risky.append("不是本机 TF 卡的固定盘符（%s）" % DEFAULT_DRIVE)
    if total_gb is not None and total_gb > MAX_CARD_GB:
        risky.append("容量 %.0f GB，大得像数据盘/系统盘，不像 TF 卡" % total_gb)

    if risky:
        msg = ("[警告] %s %s。\n"
               "       这个工具会把整只卷清空（根目录全删 + 覆写可用空间），"
               "所以默认只允许写 %s。" % (drive, "；".join(risky), DEFAULT_DRIVE))
        if dry_run:
            log(msg + "\n       试运行不做任何改动，继续打印计划。")
        elif not force_drive:
            raise SystemExit(msg + "\n       确认这就是那张卡，再显式加 --force-drive。")
        else:
            log(msg + "\n       已给 --force-drive，继续。")

    if dry_run or assume_yes:
        return
    log("即将清空并重写 %s 上的全部内容。" % drive)
    if input("确认继续？(输入 yes) : ").strip().lower() != "yes":
        raise SystemExit("已取消。")


# ---------------------------------------------------------------------------
# 步骤 1：破坏旧签名
# ---------------------------------------------------------------------------
def break_signatures(drive, dry_run):
    log("\n[1/4] 破坏旧文件签名")
    hit = 0
    # Windows 上通配符不区分大小写，先把路径去重，免得同一文件被处理两次
    candidates = set()
    for pattern in ("*.bmp", "*.aud"):
        candidates.update(glob.glob(os.path.join(drive, pattern)))

    for path in sorted(candidates):
        magic = IMAGE_MAGIC if path.lower().endswith(".bmp") else AUDIO_MAGIC
        try:
            with open(path, "r+b") as fh:
                if fh.read(len(magic)) == magic:
                    if not dry_run:
                        fh.seek(0)
                        fh.write(b"XX" * len(magic))
                    hit += 1
                    log("      破坏签名 %s" % os.path.basename(path))
        except OSError as exc:
            log("      [警告] %s : %s" % (path, exc))
    log("      共处理 %d 个文件" % hit)
    return hit


# ---------------------------------------------------------------------------
# 步骤 2：删除可见文件
# ---------------------------------------------------------------------------
def clean_drive(drive, dry_run):
    log("\n[2/4] 删除卡上可见文件")
    removed = 0
    for name in os.listdir(drive):
        if name in KEEP or name.startswith("."):
            continue
        path = os.path.join(drive, name)
        try:
            if dry_run:
                log("      将删除 %s" % name)
            elif os.path.isfile(path):
                os.remove(path)
            else:
                shutil.rmtree(path)
            removed += 1
        except OSError as exc:
            log("      [警告] 无法删除 %s : %s" % (path, exc))
    log("      共删除 %d 项" % removed)


# ---------------------------------------------------------------------------
# 步骤 3：覆写可用空间，盖掉无法通过文件系统访问的历史残留
# ---------------------------------------------------------------------------
def wipe_free_space(drive, megabytes, dry_run):
    log("\n[3/4] 覆写可用空间 %d MB（清除历史残留扇区）" % megabytes)
    if megabytes <= 0:
        log("      已按参数跳过")
        return
    target = os.path.join(drive, "wipe.bin")
    if dry_run:
        log("      将写入并删除 %s" % target)
        return

    chunk = b"\x00" * (1 << 20)
    written = 0
    try:
        with open(target, "wb") as fh:
            for _ in range(megabytes):
                fh.write(chunk)
                written += 1
                if written % 16 == 0:
                    log("      已写入 %d MB" % written)
                if shutil.disk_usage(drive).free < (64 << 20):
                    log("      剩余空间不足，提前结束")
                    break
    except OSError as exc:
        log("      [警告] 覆写中断：%s" % exc)
    finally:
        try:
            os.remove(target)
        except OSError:
            pass
    log("      已覆写并删除临时文件 %d MB" % written)


# ---------------------------------------------------------------------------
# 步骤 4：按场景顺序写入
# ---------------------------------------------------------------------------
def pick(directory, scene, kind):
    matches = sorted(glob.glob(os.path.join(directory, "%02d_*.%s" % (scene, kind))))
    return matches[0] if matches else None


def plan_scenes(image_dir, audio_dir, scenes, want_audio):
    """先把素材点清。必须在动卡之前调用——不然资料不齐时卡已经被清空了。"""
    missing = []
    plan = []
    for scene in range(scenes):
        img = pick(image_dir, scene, "bmp")
        aud = pick(audio_dir, scene, "aud") if want_audio else None
        if img is None:
            missing.append("场景 %d 缺图片（%s/%02d_*.bmp）" % (scene, image_dir, scene))
        if want_audio and aud is None:
            missing.append("场景 %d 缺音轨（%s/%02d_*.aud）" % (scene, audio_dir, scene))
        plan.append((scene, img, aud))
    return plan, missing


def write_scenes(drive, plan, dry_run):
    log("\n[4/4] 按场景顺序写入")
    total = 0
    for scene, img, aud in plan:
        for kind, src in (("img", img), ("aud", aud)):
            if src is None:
                continue
            ext = os.path.splitext(src)[1]
            dst = os.path.join(drive, "%02d_%s%s" % (scene, kind, ext))
            size = os.path.getsize(src)
            total += size
            log("      场景%d %s  %s -> %s  (%.1f MB)"
                % (scene, kind, os.path.basename(src), os.path.basename(dst), size / 1e6))
            if not dry_run:
                shutil.copyfile(src, dst)
    log("      合计 %.1f MB" % (total / 1e6))
    return True


def verify(drive, scenes, want_audio, dry_run):
    if dry_run:
        return
    log("\n校验")
    ok = True
    wanted = [("img", IMAGE_MAGIC)]
    if want_audio:
        wanted.append(("aud", AUDIO_MAGIC))
    for scene in range(scenes):
        for kind, magic in wanted:
            files = glob.glob(os.path.join(drive, "%02d_%s.*" % (scene, kind)))
            if not files:
                log("      [缺失] 场景%d %s" % (scene, kind))
                ok = False
                continue
            with open(files[0], "rb") as fh:
                head = fh.read(len(magic))
            if head != magic:
                log("      [签名错] %s" % os.path.basename(files[0]))
                ok = False
    log("      校验%s" % ("通过" if ok else "未通过"))


def main():
    ap = argparse.ArgumentParser(description="把图片与音轨安全写入 TF 卡")
    # 这台机器的 TF 卡固定是 G:，所以盘符默认就是它；显式给别的盘符会警告
    ap.add_argument("drive", nargs="?", default=DEFAULT_DRIVE,
                    help="TF 卡盘符（默认 %s，本机固定值）" % DEFAULT_DRIVE)
    ap.add_argument("-i", "--images", default=os.path.join(ROOT, "bmp_out"),
                    help="图片目录（默认 bmp_out）")
    ap.add_argument("-a", "--audio", default=os.path.join(ROOT, "aud_out"),
                    help="音轨目录（默认 aud_out）")
    ap.add_argument("-n", "--scenes", type=int, default=8, help="场景数量（默认 8）")
    ap.add_argument("-w", "--wipe-mb", type=int, default=256,
                    help="覆写多少 MB 可用空间，0 表示跳过（默认 256）")
    ap.add_argument("--no-audio", action="store_true",
                    help="只写图片，不带音轨（图像通路调试阶段用）")
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不动卡")
    ap.add_argument("--yes", action="store_true", help="跳过确认")
    ap.add_argument("--force-drive", action="store_true",
                    help="允许写非默认盘符（%s）或超过 %d GB 的卷，换卡/换读卡器时才用"
                         % (DEFAULT_DRIVE, MAX_CARD_GB))
    args = ap.parse_args()

    # 双击运行：一个参数都没有，只说明用法，绝不碰任何卡
    if len(sys.argv) == 1:
        log("这个工具不会自己猜盘符，请从命令行运行（在工程根目录下）：")
        log("")
        log("    python tools/write_card.py --no-audio --dry-run   先看计划，不动卡")
        log("    python tools/write_card.py --no-audio             确认无误再真写")
        log("")
        log("盘符不用写：本机 TF 卡固定是 %s（要写别的卡才需要显式指定）。" % DEFAULT_DRIVE)
        log("双击运行传不了参数，所以这里只说明用法，不会动任何卡。")
        try:
            input("\n按回车键退出…")
        except EOFError:
            pass
        return 2

    given      = args.drive.strip()
    is_default = (given.upper().rstrip("\\/") == DEFAULT_DRIVE.upper())
    drive      = normalise_drive(given)

    log("TF 卡      : %s%s"
        % (drive, "   （没写盘符，用本机固定值 %s）" % DEFAULT_DRIVE if is_default else ""))
    if not is_default:
        log("注意       : 你指定的是 %s，不是本机 TF 卡的固定盘符 %s，"
            "确认这是你要写的设备。" % (given, DEFAULT_DRIVE))
    try:
        usage = shutil.disk_usage(drive)
        log("卡容量     : 共 %.1f GB，可用 %.1f GB（对照一下是不是这张卡）"
            % (usage.total / 1e9, usage.free / 1e9))
    except OSError as exc:
        log("卡容量     : 读不到（%s）" % exc)
    log("图片目录   : %s" % os.path.abspath(args.images))
    log("音轨目录   : %s" % os.path.abspath(args.audio))
    log("场景数量   : %d" % args.scenes)
    if args.dry_run:
        log("模式       : 试运行，不会改动任何内容")

    # 素材先点清，缺东西就直接退出，绝不动卡
    want_audio = not args.no_audio
    if not want_audio:
        log("音轨       : 不写入（--no-audio）")

    plan, missing = plan_scenes(args.images, args.audio, args.scenes, want_audio)
    if missing:
        log("\n资料不齐，未对 TF 卡做任何改动：")
        for m in missing:
            log("  - " + m)
        return 1
    log("素材检查   : %d 个场景齐备（%s）"
        % (args.scenes, "图 + 音轨" if want_audio else "仅图片"))

    guard(drive, args.yes, args.dry_run, is_default, args.force_drive)
    break_signatures(drive, args.dry_run)
    clean_drive(drive, args.dry_run)
    wipe_free_space(drive, args.wipe_mb, args.dry_run)
    write_scenes(drive, plan, args.dry_run)
    verify(drive, args.scenes, want_audio, args.dry_run)

    log("\n完成。安全弹出 TF 卡后插入开发板。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

