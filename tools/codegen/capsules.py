"""Fit conservative capsule envelopes to URDF collision meshes and hand joint sweeps."""
import itertools
import json
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np
import pinocchio as pin
from scipy.spatial import ConvexHull
from clearance import certify, clearance
from model import PKG, ROOT, OUT, FRAMES, SIDES, origin, vector
sys.path.insert(0,str(ROOT/"tools/model"))
from verify_description import load_mesh, READY


def hull(points):
    return points[ConvexHull(points).vertices]


def fit(points, margin):
    # Minimum enclosing circle perpendicular to the principal axis. At the
    # resulting radius, shorten both ends as far as vertex containment allows.
    p = hull(points)
    centre = p.mean(axis=0)
    _, axes = np.linalg.eigh(np.cov((p-centre).T))
    xy = (p-centre) @ axes[:,:2]
    z = (p-centre) @ axes[:,2]
    def radius(c):
        return np.max(np.linalg.norm(xy-c,axis=1))
    c = np.zeros(2)
    # Minimize the maximum projected radius by successively finer planar searches.
    directions = np.stack((np.cos(np.arange(24)*np.pi/12),np.sin(np.arange(24)*np.pi/12)),axis=1)
    step = np.ptp(xy,axis=0).max()/2
    while step > 1e-9:
        candidates = c + step*directions
        best = min(candidates,key=radius)
        if radius(best) < radius(c): c = best
        else: step *= .5
    r = radius(c) + 1e-9
    slack = np.sqrt(np.maximum(0,r*r-np.sum((xy-c)**2,axis=1)))
    lo,hi = np.min(z+slack),np.max(z-slack)
    if lo>hi: lo=hi=(lo+hi)/2
    centre += axes[:,:2]@c
    a,b = centre + lo*axes[:,2], centre + hi*axes[:,2]
    distances = np.linalg.norm(points-a-np.outer(np.clip((points-a)@axes[:,2],0,hi-lo),axes[:,2]),axis=1)
    if np.max(distances)>r+1e-12: raise ValueError("Incomplete mesh coverage")
    return a,b,r+margin


def mesh(link):
    geom=link.find("collision")
    obj=geom.find("geometry/mesh")
    if obj is None:
        r=float(geom.find("geometry/sphere").get("radius"))
        t=origin(geom)
        return np.array(list(itertools.product([-r,r],repeat=3)))@t[:3,:3].T+t[:3,3], None
    v,f=load_mesh(PKG/obj.get("filename").split("/",3)[-1])
    t=origin(geom)
    scale=np.fromstring(obj.get("scale","1 1 1"),sep=" ")
    return (v*scale)@t[:3,:3].T+t[:3,3],f


def sweep(points,chain):
    drivers=[j for j in chain if j.find("mimic") is None]
    grids={}
    for j in drivers:
        low,hi=limits(j)
        grids[j.get("name")]=np.linspace(low,hi,1+int(np.ceil((hi-low)/.05)))
    # For each ancestor, |dp/dq| <= maximum possible radius about that joint.
    # Clamping a mimic joint cannot increase its driver's angular displacement.
    bound=0.
    reach=np.linalg.norm(points,axis=1).max()
    for j in reversed(chain):
        mimic=j.find("mimic")
        driver=j.get("name") if mimic is None else mimic.get("joint")
        gain=1 if mimic is None else abs(float(mimic.get("multiplier","1")))
        grid=grids[driver]
        bound+=reach*gain*(grid[-1]-grid[0])/(2*(len(grid)-1))
        reach+=np.linalg.norm(origin(j)[:3,3])
    cloud=[]
    for vals in itertools.product(*grids.values()):
        angles=dict(zip(grids,vals)); t=np.eye(4)
        for j in chain:
            mimic=j.find("mimic")
            if mimic is None: angle=angles[j.get("name")]
            else: angle=np.clip(angles[mimic.get("joint")]*float(mimic.get("multiplier","1"))+float(mimic.get("offset","0")),*limits(j))
            t=t@origin(j)
            t[:3,:3]=t[:3,:3]@pin.AngleAxis(angle,vector(j.find("axis"),"xyz")).matrix()
        cloud.append(points@t[:3,:3].T+t[:3,3])
    return hull(np.vstack(cloud)),bound


def limits(j):
    return tuple(float(j.find("limit").get(k)) for k in ("lower","upper"))


def make(side):
    tree=ET.parse(PKG/"urdf"/f"panthera_{side}_a2l1.urdf")
    joints={j.find("child").get("link"):j for j in tree.iter("joint")}
    caps=[]
    def add(frame,names,points,margin=0,triangles=None):
        a,b,r=fit(points,margin)
        caps.append(dict(points=hull(points),triangles=triangles,frame=FRAMES.index(frame),side=SIDES.index(side), links=names,
                         a=a.tolist(),b=b.tolist(),r=r,step_margin=margin))
    groups=defaultdict(list)
    for link in tree.iter("link"):
        if link.find("collision") is None: continue
        name=link.get("name"); short=name[len(side)+1:]
        v,f=mesh(link)
        if short in FRAMES:
            # Assign complete triangles so a face crossing the split remains covered.
            if short in ("link2","link3","link4","link5","base_link","link1"):
                axis=np.linalg.eigh(np.cov(v.T))[1][:,-1]
                centres=v[f].mean(axis=1)@axis
                mask=centres<(.5*(centres.min()+centres.max()))
                for part in (mask,~mask): add(short,[name],v[np.unique(f[part])],triangles=v[f[part]])
            else: add(short,[name],v,triangles=v[f])
        else:
            chain=[]; parent=name
            while parent!=side+"_palm_base_link":
                j=joints[parent]; chain.insert(0,j); parent=j.find("parent").get("link")
            while chain[-1].get("type") == "fixed":
                t=origin(chain.pop()); v=v@t[:3,:3].T+t[:3,3]
            pts,delta=sweep(hull(v),chain)
            group="thumb" if "thumb" in short else "fingers"
            groups[group].append((name,pts,delta))
    for items in groups.values():
        add("palm_base_link",[x[0] for x in items],np.vstack([x[1] for x in items]),max(x[2] for x in items))
    return caps


def segment_distance(a,b,c,d):
    u,v,w=b-a,d-c,a-c
    candidates=[]
    for p,x,y in ((a,c,d),(b,c,d),(c,a,b),(d,a,b)):
        dr=y-x
        t=np.clip(np.dot(p-x,dr)/max(np.dot(dr,dr),1e-28),0,1)
        candidates.append(np.linalg.norm(p-x-t*dr))
    cross=np.cross(u,v)
    det=cross@cross
    if det>1e-24:
        s=np.dot(np.cross(v,w),cross)/det
        t=np.dot(np.cross(u,w),cross)/det
        if 0<=s<=1 and 0<=t<=1: candidates.append(np.linalg.norm(w+s*u-t*v))
    return min(candidates)


def run():
    caps=sum((make(s) for s in SIDES),[])
    srdf=ET.parse(PKG/"srdf/panthera_dual_a2l1.srdf")
    disabled={frozenset([p.get("link1"),p.get("link2")]) for p in srdf.iter("disable_collisions")}
    pairs=[(i,j) for j,b in enumerate(caps) for i,a in enumerate(caps[:j])
           if any(x!=y and frozenset([x,y]) not in disabled for x in a["links"] for y in b["links"])]
    m=pin.buildModelFromUrdf(str(PKG/"urdf/panthera_dual_a2l1.urdf"))
    data=m.createData()
    dist={}
    poses={}
    for pose,angles in dict(zero=[0]*6,ready=READY,home_waypoint=[-.030775491,.434116572,.752034128,-.051771853,-.036911413,.132018223],lift=[0,np.deg2rad(5),np.deg2rad(5),0,0,0]).items():
        q=pin.neutral(m)
        for side in SIDES:
            for k,x in enumerate(angles,1): q[m.joints[m.getJointId(f"{side}_joint{k}")].idx_q]=x
        pin.framesForwardKinematics(m,data,q)
        poses[pose]=[data.oMf[m.getFrameId(SIDES[c["side"]]+"_"+FRAMES[c["frame"]])].copy() for c in caps]
        segments=[]
        for c in caps:
            t=data.oMf[m.getFrameId(SIDES[c["side"]]+"_"+FRAMES[c["frame"]])]
            segments.append([t.act(np.asarray(c[k])) for k in ("a","b")])
        dist[pose]=[segment_distance(*segments[i],*segments[j])-caps[i]["r"]-caps[j]["r"] for i,j in pairs]
    thresholds=[]
    for k,(i,j) in enumerate(pairs):
        a,b=caps[i],caps[j]
        baseline=min(dist[pose][k] for pose in poses)
        inter=a["side"]!=b["side"]
        proofs=[]
        for pose,frames in poses.items():
            if inter or dist[pose][k]>.012: continue
            ta,tb=frames[i],frames[j]
            if any(clearance(group,ta,tb)>.001 for group in proofs): continue
            proofs.append(certify(a["points"],b["points"],ta,tb,a["step_margin"],b["step_margin"],
                                  a["triangles"],b["triangles"]))
        effective={pose:max([dist[pose][k]]+[clearance(group,frames[i],frames[j]) for group in proofs])
                   for pose,frames in poses.items()}
        minimum=min(effective.values())
        if minimum<=0:raise ValueError((i,j,effective))
        hard=.005 if inter else min(.005,minimum*.4)
        thresholds.append(dict(hard=hard,safe=.02 if inter else min(.02, max(.001,minimum*.8)),
                               capsule_hard=.005 if inter else min(hard,baseline-.001),
                               certified=effective, proofs=proofs))
    for c in caps: del c["points"],c["triangles"]
    emit_tables(caps,pairs,thresholds)
    for t in thresholds: t["witness_counts"]=[len(g) for g in t.pop("proofs")]
    out={"capsules":caps,"pairs":pairs,"clearances_m":dist,"thresholds":thresholds}
    (OUT/"capsules.json").write_text(json.dumps(out,indent=1)+"\n")
    print(len(caps),"capsules,",len(pairs),"pairs")
    for pose,ds in dist.items():
        worst=int(np.argmin(ds)); i,j=pairs[worst]
        print(pose,ds[worst],caps[i]["links"][0],caps[j]["links"][0])
def emit_tables(caps,pairs,thresholds):
    def cpp(x):
        if isinstance(x,(list,tuple)): return "{"+",".join(map(cpp,x))+"}"
        return format(x,".17g")
    lines=["// Generated by tools/codegen/capsules.py"]
    def table(name,rows):
        lines.append(name+"[] = {")
        lines.extend(cpp(row)+"," for row in rows)
        lines.append("};")
    table("constexpr Capsule capsules",[[c["side"],c["frame"],c["a"],c["b"],c["r"]] for c in caps])
    witnesses=[];groups=[];spec=[]
    for (i,j),t in zip(pairs,thresholds):
        spec.append([i,j,t["hard"],t["safe"],t["capsule_hard"],len(groups),len(t["proofs"])])
        for group in t["proofs"]:
            groups.append([len(witnesses),len(group)])
            witnesses.extend([[p["normal"],p["offset"],p["center"],p["half_axes"]] for p in group])
    table("constexpr Witness witnesses",witnesses);table("constexpr Group groups",groups)
    table("constexpr Pair pairs",spec)
    tree=ET.parse(PKG/"urdf/panthera_left_a2l1.urdf")
    lim=[limits(tree.find(f"joint[@name='left_joint{j}']")) for j in range(1,7)]
    lim[4]=(-np.pi/3,np.pi/3)
    table("constexpr double lower", [p[0] for p in lim])
    table("constexpr double upper", [p[1] for p in lim])
    (OUT/"geometry.inc").write_text("\n".join(lines)+"\n")
if __name__=="__main__":run()
