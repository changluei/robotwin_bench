"""Aligned 1x comparison from retained real simulation videos and raw camera frames."""
import argparse
import json
from pathlib import Path
import subprocess
import imageio.v2 as imageio
import imageio_ffmpeg


def compose(left,right,output):
    output.mkdir(parents=True,exist_ok=False)
    videos=[p/'execution.mp4' for p in [left,right]];lengths=[]
    for p in videos:
        reader=imageio.get_reader(p)
        try:
            assert reader.get_meta_data()['fps']==10
            lengths.append(reader.count_frames()/10)
        finally:reader.close()
    duration=max(lengths)
    cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-threads','2']
    for p in videos:cmd+=['-i',str(p)]
    filters=f'[0:v]setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={duration}[a];'
    filters+=f'[1:v]setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={duration}[b];[a][b]hstack=inputs=2[out]'
    cmd+=['-filter_complex_threads','1','-filter_complex',filters,'-map','[out]','-t',str(duration),
          '-r','10','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(output/'comparison.mp4')]
    subprocess.run(cmd,check=True)
    results=[json.loads((p/'result.json').read_text()) for p in [left,right]]
    (output/'alignment.json').write_text(json.dumps(dict(inputs=list(map(str,videos)),results=results,
        fps=10,speed=1,video_duration=duration,command=cmd,
        note='Same checkpoint origin and playback speed. Completed source holds last frame. Original terminal frames retained.'),indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    samples=[(left/'audit_frames/initial','Initial t=0')]
    for p,r in zip([left,right],results):
        a=json.loads((p/'audit_summary.json').read_text())
        samples.append((p/'audit_frames/first_information',f"{r['policy_id']} t={r['t_information_ready']:.3f}s; source={a['information_source']}"))
    fig,axes=plt.subplots(3,3,figsize=(13,10.5))
    for i,(folder,label) in enumerate(samples):
        for j,camera in enumerate(['head','left','right']):
            path=folder/f'{camera}.png'
            if path.exists():axes[i,j].imshow(plt.imread(path))
            axes[i,j].set_title(f'{label}\n{camera}',fontsize=9);axes[i,j].axis('off')
    fig.tight_layout();fig.savefig(output/'actual_cameras.png',dpi=150);plt.close(fig)
    print(output,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left',required=True,type=Path);p.add_argument('--right',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    compose(args.left.resolve(),args.right.resolve(),args.output.resolve())
