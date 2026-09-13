"""Compose actual simulator frames and timing plots, never synthesized imagery."""
import json
from pathlib import Path
import imageio.v2 as imageio
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
RUNS=ROOT/'result/held_module_key'
OUT=ROOT/'experiments/held_module_key'
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'


def main():
    font=ImageFont.truetype(FONT,22)
    canvas=Image.new('RGB',(1920,3*520),(245,245,245));draw=ImageDraw.Draw(canvas)
    groups=[('Same t=0; key hidden by module body',RUNS/'final400_H/frames/initial'),
            ('H: first valid key at 4.3 s, left wrist',RUNS/'final400_H/frames/first_information'),
            ('S: first valid key at 2.6 s, head',RUNS/'final400_S/frames/first_information')]
    for row,(label,folder) in enumerate(groups):
        for col,name in enumerate(['head','left','right']):
            canvas.paste(Image.open(folder/f'{name}.png'),(col*640,row*520+40))
            draw.text((col*640+8,row*520+8),f'{label} | {name}' if col==0 else name,fill=(20,20,20),font=font)
    canvas.save(OUT/'actual_camera_grid.jpg',quality=94)
    zoom=Image.new('RGB',(1000,500),(245,245,245));draw=ImageDraw.Draw(zoom)
    for col,(run,cam,label) in enumerate([('final400_H','left','H / left / t=4.3 s'),('final400_S','head','S / head / t=2.6 s')]):
        d=next(json.loads(x) for x in (RUNS/run/'perception.jsonl').read_text().splitlines() if json.loads(x)['valid'])
        uv=np.array(d['pixels']);center=uv.mean(0).astype(int);im=Image.open(RUNS/run/'frames/first_information'/f'{cam}.png')
        crop=im.crop((center[0]-55,center[1]-55,center[0]+55,center[1]+55)).resize((440,440),Image.Resampling.NEAREST)
        zoom.paste(crop,(col*500+30,50));draw.text((col*500+15,12),label,fill=(20,20,20),font=font)
    zoom.save(OUT/'key_pixels_zoom.png')
    results=[json.loads((RUNS/run/'result.json').read_text()) for run in ['final400_H','final400_S']]
    fig,ax=plt.subplots(figsize=(11,4.4))
    colors=['#687f96','#4c9bce','#42a085','#b4c4ca']
    for i,r in enumerate(results):
        ts=[0,r['t_information'],r['t_install_ready'],r['t_right_stable'],r['t_right_exit']]
        for j,(a,b) in enumerate(zip(ts[:-1],ts[1:])):
            ax.barh(1-i,b-a,left=a,height=.42,color=colors[j],label=['Acquire information','Move to insertion prepose','Insert, release, stabilize','Finish wrist withdrawal'][j] if i==0 else None)
        for j,t in enumerate(ts[1:]):ax.text(t,1-i+.26,f'{t:.3f}',ha='center',fontsize=9)
    ax.set_yticks([1,0],['H: active left wrist','S: present to head']);ax.set_xlim(0,10.4)
    ax.set_xlabel('Physical seconds after contact-held checkpoint (same development instance)')
    ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.34),ncol=2,frameon=False,fontsize=9)
    fig.text(.02,.02,'Prepose boundary is checked with actual FK; strict 2 mm stopped-readiness gate remained null in both. No left workload benchmark.',fontsize=8)
    fig.subplots_adjust(left=.17,right=.985,top=.90,bottom=.40)
    fig.savefig(OUT/'physical_timing.png',dpi=170);plt.close(fig)
    readers=[imageio.get_reader(str(RUNS/run/'execution.mp4')) for run in ['final400_H','final400_S']]
    counts=[r.count_frames() for r in readers]
    last=[None,None]
    writer=imageio.get_writer(str(OUT/'H_S_actual_video.mp4'),fps=10,codec='libx264',quality=7,macro_block_size=1)
    for i in range(max(counts)):
        for j,r in enumerate(readers):
            if i<counts[j]:last[j]=r.get_next_data()
        writer.append_data(np.concatenate(last,axis=1))
    writer.close()
    for r in readers:r.close()


if __name__=='__main__':main()
