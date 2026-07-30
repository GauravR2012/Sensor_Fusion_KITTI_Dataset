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

def match_descriptors(d1, d2, ratio=0.75):
    if len(d1) < 2 or len(d2) < 2:
        return []
    pairs = bf.knnMatch(d1, d2, k=2)
    return [p[0] for p in pairs if len(p) == 2 and p[0].distance < ratio * p[1].distance]

def build_stereo_landmarks(gray_l, gray_r, p2, p3):
    kps_l, desc_l = detect_orb(gray_l)
    kps_r, desc_r = detect_orb(gray_r)
    m_lr = match_descriptors(desc_l, desc_r)
    if len(m_lr) < 20:
        return None, None
    pts_l = np.float32([kps_l[m.queryIdx].pt for m in m_lr])
    pts_r = np.float32([kps_r[m.trainIdx].pt for m in m_lr])
    pts3d = triangulate_stereo(pts_l, pts_r, p2, p3)
    valid = (pts3d[:, 2] > 1.0) & (pts3d[:, 2] < 80.0)
    if valid.sum() < 20:
        return None, None
    return pts3d[valid], pts_l[valid]

def pnp_relative_pose(obj_pts, img_pts, k):
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj_pts.astype(np.float64), img_pts.astype(np.float64), k, None,
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

def run_stereo_vo(img_l_files, img_r_files, k, p2, p3):
    gray_l_prev = cv2.imread(img_l_files[0], cv2.IMREAD_GRAYSCALE)
    gray_r_prev = cv2.imread(img_r_files[0], cv2.IMREAD_GRAYSCALE)
    landmarks_3d, landmarks_2d = build_stereo_landmarks(gray_l_prev, gray_r_prev, p2, p3)
    if landmarks_3d is None:
        raise RuntimeError("Failed to triangulate landmarks on frame 0.")

    t_world = np.eye(4)
    vo_poses, inlier_counts, failed = [t_world.copy()], [], 0

    for i in range(1, len(img_l_files)):
        gray_l = cv2.imread(img_l_files[i], cv2.IMREAD_GRAYSCALE)
        gray_r = cv2.imread(img_r_files[i], cv2.IMREAD_GRAYSCALE)
        if gray_l is None or gray_r is None:
            failed += 1
            vo_poses.append(t_world.copy())
            inlier_counts.append(0)
            continue

        kps_prev, desc_prev = detect_orb(gray_l_prev)
        kps_curr, desc_curr = detect_orb(gray_l)
        m_ll = match_descriptors(desc_prev, desc_curr)

        if len(m_ll) < config.MIN_PNP_INLIERS:
            failed += 1
            vo_poses.append(t_world.copy())
            inlier_counts.append(0)
            gray_l_prev, gray_r_prev = gray_l, gray_r
            landmarks_3d, landmarks_2d = build_stereo_landmarks(gray_l, gray_r, p2, p3)
            continue

        pts_curr = np.float32([kps_curr[m.trainIdx].pt for m in m_ll])
        pts_prev = np.float32([kps_prev[m.queryIdx].pt for m in m_ll])
        obj_pts, img_pts = [], []
        for p_prev, p_curr in zip(pts_prev, pts_curr):
            d = np.linalg.norm(landmarks_2d - p_prev, axis=1)
            j = int(np.argmin(d))
            if d[j] < config.NN_THRESH_PX:
                obj_pts.append(landmarks_3d[j])
                img_pts.append(p_curr)

        if len(obj_pts) < config.MIN_PNP_INLIERS:
            failed += 1
            vo_poses.append(t_world.copy())
            inlier_counts.append(0)
        else:
            t_rel, n_inl = pnp_relative_pose(np.array(obj_pts), np.array(img_pts), k)
            if t_rel is None:
                failed += 1
                vo_poses.append(t_world.copy())
                inlier_counts.append(0)
            else:
                t_world = t_world @ np.linalg.inv(t_rel)
                vo_poses.append(t_world.copy())
                inlier_counts.append(n_inl)

        landmarks_3d, landmarks_2d = build_stereo_landmarks(gray_l, gray_r, p2, p3)
        gray_l_prev, gray_r_prev = gray_l, gray_r

    return vo_poses, inlier_counts, failed
