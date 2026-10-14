import sys, os, numpy as np, mujoco, cv2
sys.path.insert(0,"."); from cell_twin.scene import compile_model, load_part_offsets
from scripts.replay import joint_ids
from cell_twin.bench import GRIP_SLIDE
from cell_twin.layout import j6_offset
def set_qpos(d, jid, st):
    for arm, off in (('left', 0), ('right', 7)):
        for i in range(6): d.qpos[jid[(arm, i)]] = st[off + i] + (j6_offset(arm) if i == 5 else 0.0)
        d.qpos[jid[(arm, 'fl')]] = GRIP_SLIDE; d.qpos[jid[(arm, 'fr')]] = GRIP_SLIDE
from scripts.fit_camera import read_frame
EPS="inputs/nuc/episodes/objects"; W,H=320,240
spec,m=compile_model(part_mode="geometric"); d=mujoco.MjData(m); jid=joint_ids(m)
r=mujoco.Renderer(m,H,W); r.enable_segmentation_rendering(); rc=mujoco.Renderer(m,H,W)
tiles=[]; eps=sorted(os.listdir(EPS))[:3]
for arm,other in (("right","left"),("left","right")):
    own=np.zeros(m.ngeom+1,bool); oth=np.zeros(m.ngeom+1,bool)
    for i in range(m.ngeom):
        bn=m.body(m.geom_bodyid[i]).name
        if bn.startswith(arm+"_") and ("finger" in bn or "pad" in bn): own[i]=True
        if bn.startswith(other+"_"): oth[i]=True
    for ep in eps:
        z=np.load(f"{EPS}/{ep}/frames.npz"); st=z["observation.state"][0]; fr=read_frame(f"{EPS}/{ep}/{arm}_wrist.mp4",0)
        set_qpos(d,jid,st); mujoco.mj_forward(m,d); r.update_scene(d,camera=f"{arm}_wrist_cam"); seg=r.render()[:,:,0].astype(int); idx=np.clip(seg,-1,m.ngeom); valid=seg>=0
        mo,mw=own[idx]&valid, oth[idx]&valid; ov=fr.copy(); ov[cv2.Canny(mw.astype(np.uint8)*255,50,150)>0]=(0,0,255); ov[cv2.Canny(mo.astype(np.uint8)*255,50,150)>0]=(0,255,255)
        rc.update_scene(d,camera=f"{arm}_wrist_cam"); twin=cv2.cvtColor(rc.render(),cv2.COLOR_RGB2BGR)
        cv2.putText(ov,f"{arm} wrist",(6,18),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,255),2); tiles.append(np.hstack([ov,twin]))
cv2.imwrite("out/wrist_cam_check.png", np.vstack(tiles)); print("wrote out/wrist_cam_check.png")
