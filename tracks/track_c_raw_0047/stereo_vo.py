import cv2, numpy as np
from . import config

orb = cv2.ORB_create(nfeatures=config.MAX_LANDMARKS, scaleFactor=1.2, nlevels=8)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

def triangulate_stereo(pts_l, pts_r, p2, p3):
    x = cv2.triangulatePoints(p2, p3, pts_l.T, pts_r.T)
    return (x[:3] / x[3]).T

def detect_orb(gray):
    kps, desc = orb.detectAndCompute(gray, None)
    if desc is None:
        desc = np.zeros((0, 32), dtype=np.uint8)
    return kps, desc

def match_ratio(d1, d2, ratio=0.75):
    if len(d1) < 2 or len(d2) < 2:
        return []
    pairs = bf.knnMatch(d1, d2, k=2)
    return [p[0] for p in pairs if len(p) == 2 and p[0].distance < ratio * p[1].distance]

def build_stereo_landmarks(gray_l, gray_r, p2, p3):
    kps_l, desc_l = detect_orb(gray_l)
    kps_r, desc_r = detect_orb(gray_r)
    if len(kps_l) < 20 or len(kps_r) < 20:
        return None, None
    matches = sorted(bf.match(desc_l, desc_r), key=lambda m: m.distance)[:config.MAX_LANDMARKS]
    if len(matches) < 20:
        return None, None
    pts_l = np.float32([kps_l[m.queryIdx].pt for m in matches])
    pts_r = np.float32([kps_r[m.trainIdx].pt for m in matches])
    pts3d = triangulate_stereo(pts_l, pts_r, p2, p3)
    valid = (pts3d[:, 2] > 1.0) & (pts3d[:, 2] < 80.0)
    if valid.sum() < 20:
        return None, None
    return pts3d[valid], pts_l[valid]

def pnp_pose(obj_pts, img_pts, k):
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj_pts.astype(np.float64), img_pts.astype(np.float32), k, None,
        flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=config.PNP_REPROJ_ERR,
        confidence=0.999, iterationsCount=200,
    )
    if not ok or inliers is None or len(inliers) < config.MIN_PNP_INLIERS:
        return None, 0
    r, _ = cv2.Rodrigues(rvec)
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = tvec.ravel()
    return t, len(inliers)

def run_vo_step(gray_l_prev, gray_l, landmarks_3d, landmarks_2d, k):
    kps_prev, desc_prev = detect_orb(gray_l_prev)
    kps_curr, desc_curr = detect_orb(gray_l)
    m_ll = match_ratio(desc_prev, desc_curr)
    if len(m_ll) < config.MIN_PNP_INLIERS:
        return None, 0
    pts_prev = np.float32([kps_prev[m.queryIdx].pt for m in m_ll])
    pts_curr = np.float32([kps_curr[m.trainIdx].pt for m in m_ll])
    obj_pts, img_pts = [], []
    for p_prev, p_curr in zip(pts_prev, pts_curr):
        d = np.linalg.norm(landmarks_2d - p_prev, axis=1)
        j = int(np.argmin(d))
        if d[j] < config.NN_THRESH_PX:
            obj_pts.append(landmarks_3d[j])
            img_pts.append(p_curr)
    if len(obj_pts) < config.MIN_PNP_INLIERS:
        return None, 0
    return pnp_pose(np.array(obj_pts), np.array(img_pts), k)

def precompute_vo_rel(left_images, right_images, k, p2, p3):
    gray_l_prev = cv2.imread(left_images[0], cv2.IMREAD_GRAYSCALE)
    gray_r_prev = cv2.imread(right_images[0], cv2.IMREAD_GRAYSCALE)
    landmarks_3d, landmarks_2d = build_stereo_landmarks(gray_l_prev, gray_r_prev, p2, p3)
    if landmarks_3d is None:
        raise RuntimeError("VO init failed on frame 0")
    rel, inl_hist = [], []
    for i in range(1, len(left_images)):
        gray_l = cv2.imread(left_images[i], cv2.IMREAD_GRAYSCALE)
        gray_r = cv2.imread(right_images[i], cv2.IMREAD_GRAYSCALE)
        if gray_l is None:
            rel.append(np.eye(4)); inl_hist.append(0); continue
        t_vo, n_inl = run_vo_step(gray_l_prev, gray_l, landmarks_3d, landmarks_2d, k)
        if t_vo is None:
            t_vo, n_inl = np.eye(4), 0
        else:
            lm3, lm2 = build_stereo_landmarks(gray_l, gray_r, p2, p3)
            if lm3 is not None:
                landmarks_3d, landmarks_2d = lm3, lm2
        rel.append(t_vo)
        inl_hist.append(n_inl)
        gray_l_prev = gray_l
    return rel, inl_hist
