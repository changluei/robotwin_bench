"""Chinese annotated cut of the existing physical runs; no simulation or retiming."""
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import imageio.v2 as imageio

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'experiments/held_module_key/video_delivery_v1'
RUNS = [ROOT / 'result/held_module_key' / name for name in ['final400_H', 'final400_S']]
REGULAR = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BOLD = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
BG = (17, 25, 38)
WHITE = (237, 242, 249)
MUTED = (165, 182, 200)
COLORS = [(87, 183, 248), (106, 221, 175)]


def write(draw, xy, text, size=30, color=WHITE, bold=False):
    draw.text(xy, text, fill=color, font=ImageFont.truetype(BOLD if bold else REGULAR, size))


def card(end=False):
    im = Image.new('RGB', (1920, 1080), BG)
    d = ImageDraw.Draw(im)
    write(d, (100, 80), '持物模块底部安装键｜H / S 实际执行对比', 58, bold=True)
    write(d, (100, 175), '同一开发实例 400 · 同版本 · 同初始物理状态 · GPU 0', 34, MUTED)
    d.line((100, 260, 1820, 260), fill=(67, 84, 103), width=2)
    write(d, (100, 310), 'H：左臂主动换位观察', 43, COLORS[0], True)
    write(d, (1010, 310), 'S：右臂展示给 head', 43, COLORS[1], True)
    write(d, (100, 390), '调动左腕获取底键信息，再恢复左臂', 32)
    write(d, (1010, 390), '保持夹持；左臂未被调走', 32)
    for y, label, h, s in [(485, '有效信息', '4.300 秒', '2.600 秒'),
                            (565, '右侧稳定完成', '8.380 秒', '8.212 秒'),
                            (645, '右腕退出完成', '9.900 秒', '9.736 秒')]:
        write(d, (100, y), f'{label}：{h}', 36)
        write(d, (1010, y), f'{label}：{s}', 36)
    if end:
        write(d, (100, 790), '两种路线均能定位并真实入座；本实例未发现 H 的局部收益。', 36, bold=True)
        write(d, (100, 860), 'S 本身也是主动观察。相对“无专门观察”的净收益，本轮没有验证。', 29, MUTED)
        write(d, (100, 928), '严格 2 mm 静止就绪标志两组均未触发；单实例结果不作跨实例结论。', 27, MUTED)
    else:
        write(d, (100, 790), '下面为同步正片：1× 播放，所有时刻均为实际物理时间。', 36, bold=True)
        write(d, (100, 860), '每侧保留全景、head、左腕、右腕；overview 只用于录制。', 30, MUTED)
        write(d, (100, 928), '此片仅核验局部几何，不是“双臂均在操作”的机会成本 benchmark。', 27, MUTED)
    return im


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    results = [json.loads((p / 'result.json').read_text()) for p in RUNS]
    readers = [imageio.get_reader(str(p / 'execution.mp4')) for p in RUNS]
    counts = [r.count_frames() for r in readers]
    assert all(r.get_meta_data()['fps'] == 10 for r in readers)
    writer = imageio.get_writer(str(OUT / 'H_S_comparison_zh.mp4'), fps=10,
        codec='libx264', quality=8, macro_block_size=1, pixelformat='yuv420p',
        ffmpeg_params=['-movflags', '+faststart'])
    opening = card(); opening.save(OUT / 'poster.png')
    for _ in range(30): writer.append_data(np.array(opening))
    last = [None, None]
    for i in range(max(counts)):
        t = i / 10
        im = Image.new('RGB', (1920, 1080), BG)
        d = ImageDraw.Draw(im)
        write(d, (30, 15), '实际同步执行｜1×', 37, bold=True)
        write(d, (1430, 18), f'物理时钟  {t:05.1f} s', 38, bold=True)
        for side, (reader, result) in enumerate(zip(readers, results)):
            x = side * 960
            if i < counts[side]: last[side] = reader.get_next_data()
            frame = last[side]
            assert frame.shape == (720, 960, 3)
            im.paste(Image.fromarray(frame), (x, 158))
            write(d, (x + 24, 74), 'H：左腕主动换位' if side == 0 else 'S：持物展示给 head', 33, COLORS[side], True)
            if t < result['t_information']:
                stage = '观察中｜尚无有效底键估计'
            elif t < result['t_install_ready']:
                stage = '已获得信息｜移动至插入预备位'
            elif t < result['t_right_stable']:
                stage = '插入 / 释放 / 接触稳定'
            elif t < result['t_right_exit']:
                stage = '右侧已稳定完成｜手腕退出中'
            else:
                stage = '右侧稳定入座，右腕已退出'
            if i >= counts[side]: stage += '（原片结束，保持末帧）'
            write(d, (x + 24, 120), stage, 24, MUTED)
            if result['t_information'] <= t < result['t_information'] + 1:
                # Highlight the actual source view without hiding any camera.
                box = (x + 2, 158 + 362, x + 478, 158 + 663) if side == 0 else (x + 482, 160, x + 958, 158 + 358)
                d.rectangle(box, outline=COLORS[side], width=6)
            write(d, (x + 24, 891), f"有效信息 {result['t_information']:.3f} s    稳定完成 {result['t_right_stable']:.3f} s", 29, COLORS[side], True)
            write(d, (x + 24, 938), f"插入启动 {result['t_install_ready']:.3f} s    右腕退出 {result['t_right_exit']:.3f} s", 26)
            write(d, (x + 24, 983), '左臂恢复原位 9.196 s' if side == 0 else '左臂未被调走；三路相机均保留', 25, MUTED)
        write(d, (24, 1040), '插入启动不等同于严格静止就绪；无假等待、无时间加速。开场和结尾为说明画面。', 23, MUTED)
        if i == 43: im.save(OUT / 'comparison_frame.png')
        writer.append_data(np.array(im))
    ending = card(True)
    for _ in range(40): writer.append_data(np.array(ending))
    writer.close()
    for r in readers: r.close()
    manifest = dict(source_runs=[str(p) for p in RUNS],
        source_sha256={str(p / 'execution.mp4'): hashlib.sha256((p / 'execution.mp4').read_bytes()).hexdigest() for p in RUNS},
        fps=10, resolution=[1920, 1080], opening_seconds=3, physical_replay_seconds=max(counts)/10,
        closing_seconds=4, physical_replay_speed=1, source_frames=counts,
        shorter_clip_tail_hold_frames=max(counts)-min(counts), simulation_rerun=False)
    (OUT / 'provenance.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(OUT / 'H_S_comparison_zh.mp4')


if __name__ == '__main__': main()
