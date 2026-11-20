"""Separating-plane lower bounds for conservative capsules overlapping at home.

Each witness plane is fixed to A. All vertices of its triangle subset lie on
one side; an oriented box covers the associated subset of B. The minimum
corner projection stays a lower bound under arbitrary rigid transforms.
"""
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull


def certify(a, b, ta, tb, margin_a, margin_b, triangles_a=None, triangles_b=None):
    def world(p,t): return p@t.rotation.T+t.translation
    aw,bw=world(a,ta),world(b,tb)
    inequalities=np.vstack((np.c_[aw,-np.ones(len(a)),np.zeros(len(a))],
                            np.c_[-bw,np.ones((len(b),2))]))
    lp=linprog([0,0,0,0,-1],inequalities,np.zeros(len(inequalities)),
               bounds=[(-1,1)]*3+[(None,None)]*2)
    if not lp.success: raise ValueError(lp.message)
    norm=np.linalg.norm(lp.x[:3])
    gap=lp.x[-1]/max(norm,1e-12)-margin_a-margin_b
    if gap>1e-6:
        normal=lp.x[:3]/norm
        na=ta.rotation.T@normal
        nb=tb.rotation.T@normal
        v=np.eye(3)[np.argmin(abs(nb))]
        v=np.cross(nb,v);v/=np.linalg.norm(v)
        axes=np.stack((nb,v,np.cross(nb,v)),axis=1)
        projection=b@axes
        low=projection.min(axis=0);high=projection.max(axis=0)
        return [dict(normal=na.tolist(),offset=np.max(a@na)+margin_a+margin_b,
                     center=(axes@((low+high)/2)).tolist(),
                     half_axes=(axes*((high-low)/2)).T.tolist())]
    # Concave links can have intersecting convex hulls even when their meshes
    # are separated. Split complete triangles until every Cartesian subset pair
    # has a positive certificate. A single inseparable triangle pair is an error.
    def spread(tris):
        return 0 if tris is None or len(tris)<2 else np.ptp(tris.mean(axis=1),axis=0).max()
    choose_a=spread(triangles_a)>spread(triangles_b)
    tris=triangles_a if choose_a else triangles_b
    if spread(tris)<1e-10: raise ValueError("No positive mesh separation certificate")
    centers=tris.mean(axis=1)
    order=np.argsort(centers[:,np.argmax(np.ptp(centers,axis=0))])
    results=[]
    for indices in np.array_split(order,2):
        part=tris[indices]
        pts=np.unique(part.reshape(-1,3),axis=0)
        if len(pts)>4 and np.linalg.matrix_rank(pts-pts[0])==3:
            pts=pts[ConvexHull(pts).vertices]
        if choose_a:
            results+=certify(pts,b,ta,tb,margin_a,margin_b,part,triangles_b)
        else:
            results+=certify(a,pts,ta,tb,margin_a,margin_b,triangles_a,part)
    return results


def clearance(proofs,ta,tb):
    def separation(p):
        n=ta.rotation@p["normal"]
        nb=tb.rotation.T@n
        return n@(tb.act(np.array(p["center"]))-ta.translation)-p["offset"]-abs(p["half_axes"]@nb).sum()
    return min(map(separation,proofs))
