import numpy as np

def umeyama_3d(src, tgt):
    mu_s, mu_t = src.mean(0), tgt.mean(0)
    sc, tc = src - mu_s, tgt - mu_t
    u, _, vt = np.linalg.svd((tc.T @ sc) / len(src))
    s = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    r = u @ s @ vt
    return (sc @ r.T) + (mu_t - r @ mu_s)

def umeyama_2d(src, tgt):
    src3 = np.column_stack([src, np.zeros(len(src))])
    tgt3 = np.column_stack([tgt, np.zeros(len(tgt))])
    return umeyama_3d(src3, tgt3)[:, :2]

def ate_3d(est, gt):
    return float(np.sqrt(np.mean(np.sum((est - gt) ** 2, axis=1))))

def ate_2d(est, gt):
    return float(np.sqrt(np.mean(np.sum((est - gt) ** 2, axis=1))))

def path_length(xyz):
    if len(xyz) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)))

def xyz_from_poses(poses):
    return np.array([t[:3, 3] for t in poses])

def chain_poses(relative_poses):
    t_w = np.eye(4)
    traj = [t_w.copy()]
    for t_rel in relative_poses:
        t_w = t_w @ np.linalg.inv(t_rel)
        traj.append(t_w.copy())
    return traj
