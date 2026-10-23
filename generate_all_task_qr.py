"""生成所有有效（颜色不重复）的搬运机器人任务码二维码。

总数：P(6,3) * P(3,3) * P(6,3) * P(3,3) = 518400。
文件名示例：task_qr_156_123_516_231.png

用法：
    python generate_all_task_qr.py -o "D:\\yushizhe\\比赛\\工程创新比赛\\二维码"

脚本支持断点续跑：已存在且大小大于0的 PNG 会自动跳过。
"""

from __future__ import annotations

import argparse
import itertools
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import qrcode
from qrcode.constants import ERROR_CORRECT_M


COLOR_ORDERS = tuple("".join(x) for x in itertools.permutations("123456", 3))
RING_ORDERS = tuple("".join(x) for x in itertools.permutations("123", 3))


def task_codes():
    for a in COLOR_ORDERS:
        for b in RING_ORDERS:
            for c in COLOR_ORDERS:
                for d in RING_ORDERS:
                    yield f"{a}+{b}+{c}+{d}"


def make_one(args):
    output_dir, code, box_size, border = args
    path = Path(output_dir) / f"task_qr_{code.replace('+', '_')}.png"
    if path.exists() and path.stat().st_size > 0:
        return False
    qr = qrcode.QRCode(
        version=None,
        error_correction=ERROR_CORRECT_M,
        box_size=box_size,
        border=border,
    )
    qr.add_data(code)
    qr.make(fit=True)
    qr.make_image(fill_color="black", back_color="white").save(path, format="PNG")
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", required=True, type=Path)
    parser.add_argument("--box-size", type=int, default=12)
    parser.add_argument("--border", type=int, default=4)
    parser.add_argument("--workers", type=int, default=max(1, min(16, os.cpu_count() or 1)))
    args = parser.parse_args()
    if args.box_size < 1 or args.border < 4 or args.workers < 1:
        raise SystemExit("box-size/workers必须为正数，border至少为4")
    output_dir = args.output.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    jobs = ((str(output_dir), code, args.box_size, args.border) for code in task_codes())
    done = 0
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for index, created in enumerate(pool.map(make_one, jobs), 1):
            done += created
            if index % 10000 == 0:
                print(f"checked={index}, created={done}", flush=True)
    print(f"完成：共检查518400个组合，新生成{done}个二维码")
    print(f"输出目录：{output_dir}")


if __name__ == "__main__":
    main()
