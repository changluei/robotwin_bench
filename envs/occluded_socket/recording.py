"""Run-isolated JSONL and actual rendered video; GT stays in separate files."""
import hashlib
import json
import subprocess
from pathlib import Path
import cv2
import imageio.v2 as imageio
import numpy as np


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, default=json_default)+'\n', encoding='utf-8')


def state_hash(state):
    return hashlib.sha256(json.dumps(state, sort_keys=True, default=json_default).encode()).hexdigest()


class Recorder:
    def __init__(self, output, config, record):
        self.output = Path(output)
        self.files = {name: (self.output/f'{name}.jsonl').open('w', encoding='utf-8')
                      for name in ['controller', 'perception', 'joints', 'evaluator_gt', 'contacts_gt']}
        self.writer = imageio.get_writer(str(self.output/'execution.mp4'), fps=config['video_fps'],
                        codec='libx264', quality=7, macro_block_size=1) if record else None
        self.frames = 0

    def log(self, stream, value):
        self.files[stream].write(json.dumps(value, ensure_ascii=False, default=json_default)+'\n')

    def video(self, env, frames, policy, condition, status, information):
        if self.writer is None:
            return
        overview = env.camera_map['overview']
        overview.take_picture()
        image = (overview.get_picture('Color')[...,:3]*255).clip(0,255).astype('uint8')
        tiles = [image, frames['head'].rgb.copy(), frames['left'].rgb.copy(), frames['right'].rgb.copy()]
        labels = ['Overview (recording only)', 'Head', 'Left wrist', 'Right wrist']
        for tile, label in zip(tiles, labels):
            cv2.rectangle(tile, (0,0), (tile.shape[1],28), (20,20,20), -1)
            cv2.putText(tile,label,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        canvas = np.concatenate([np.concatenate(tiles[:2],axis=1),np.concatenate(tiles[2:],axis=1)],axis=0)
        canvas = cv2.resize(canvas,(960,720))
        cv2.rectangle(canvas,(0,665),(960,720),(16,16,16),-1)
        for line, y in [(f'{policy} / {condition}   physics t={env.sim_time:.3f}s   vision={information}',686),
                        (f'L: {status["left"]}    R: {status["right"]}',710)]:
            cv2.putText(canvas,line,(8,y),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        self.writer.append_data(canvas)
        self.frames += 1

    def close(self):
        for f in self.files.values():
            f.close()
        if self.writer is not None:
            self.writer.close()
