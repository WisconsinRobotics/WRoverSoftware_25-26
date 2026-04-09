import numpy as np
import depthai as dai
import cv2
import time
# ── Camera intrinsics ──────────────────────────────────────────────────────────
CX, CY = 605.0, 360.0
FX, FY = 563.33333, 563.33333
H_CAM, W_CAM = 720, 1280

# ── Pre-compute full-res normalised pixel offsets ONCE at module load ──────────
# These never change frame-to-frame — no reason to recompute inside a function.
_uu_full, _vv_full = np.meshgrid(np.arange(W_CAM), np.arange(H_CAM))
_dx_full = (_uu_full - CX) / FX    # (u - cx) / fx  shape: (720, 1280)
_dy_full = (_vv_full - CY) / FY    # (v - cy) / fy  shape: (720, 1280)
_row_mask_full = _vv_full > (H_CAM // 3)

# ── Pre-compute subsampled grids for RANSAC (step=10) ─────────────────────────
_STEP = 10
_uu_sub, _vv_sub = np.meshgrid(
    np.arange(0, W_CAM, _STEP),
    np.arange(0, H_CAM, _STEP)
)
_dx_sub = (_uu_sub - CX) / FX
_dy_sub = (_vv_sub - CY) / FY
_row_mask_sub = _vv_sub > (H_CAM // 3)


def backproject_depth_sub(depth,
                           z_min=0.1,  z_max=3.5,
                           y_min=0.15, y_max=1.80):
    """
    Subsampled (step=10) backprojection with Y-band filter baked in.
    Returns pts (Nx3), u_flat, v_flat.
    Uses pre-computed grids — zero reallocation of coordinate arrays.
    """
    Z = depth[_vv_sub, _uu_sub]             # grab depth at sub-sampled pixels
    X = _dx_sub * Z
    Y = _dy_sub * Z

    valid = (
        (Z > z_min) & (Z < z_max) &
        _row_mask_sub   &               # skip top 1/3 of image
        (Y > y_min) & (Y < y_max)       # only plausible floor Y-band
    )

    pts = np.stack([X[valid], Y[valid], Z[valid]], axis=-1)
    return pts, _uu_sub[valid], _vv_sub[valid]


def apply_plane_to_full_depth(depth, n, d, dist_thresh=0.06):
    """
    Compute per-pixel plane distance WITHOUT building a full Nx3 point cloud.

    For any pixel (u, v) with depth Z its 3D position is:
        point = [dx*Z,  dy*Z,  Z]

    Distance to plane:
        dist = |n · point + d|
             = |Z * (n[0]*dx + n[1]*dy + n[2]) + d|

    Everything here is a (720,1280) array operation — no temporary Nx3 array,
    no Python loops. This replaces the entire step=1 backproject + inliers_to_pixels
    pipeline and is the single biggest speed win.
    """
    A = n[0] * _dx_full + n[1] * _dy_full + n[2]   # (720, 1280)
    dist = np.abs(depth * A + d)                     # (720, 1280)

    return (
        (dist < dist_thresh)           &
        (depth > 0.01) & (depth < 5.5) &
        _row_mask_full
    )  # boolean mask (720, 1280), ready for cv2 directly


def ransac_plane(pts,
                 iters=400,
                 dist_thresh=0.12,
                 angle_thresh_deg=30,
                 target=None, tol=1.0,
                 min_inliers=400,
                 early_exit_ratio=0.5):
    """
    RANSAC with two speed improvements:
      1. Transpose trick  — pts.T computed once, avoids per-iteration reshaping
      2. Early exit       — stops as soon as a plane owns 65% of points;
                            there's no better answer at that point
    Also refines the winning plane with SVD for higher accuracy.
    """
    N = pts.shape[0]
    if N < 3:
        return False, None, None, None

    up        = np.array([0.0, 1.0, 0.0])
    cos_thresh = np.cos(np.deg2rad(angle_thresh_deg))
    rng       = np.random.default_rng()

    # Transpose once so distance calc is a single np.dot broadcast, not 3 ops
    pts_T     = pts.T   # (3, N)

    best_inliers = -1
    n_best = d_best = mask_best = None

    for _ in range(iters):
        idx      = rng.choice(N, size=3, replace=False)
        p1, p2, p3 = pts[idx]

        n    = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(n)
        if norm < 1e-6:
            continue
        n /= norm

        if np.dot(n, up) < 0:
            n = -n
        if np.dot(n, up) < cos_thresh:     # reject tilted planes (walls etc.)
            continue

        d = -np.dot(n, p1)

        if target is not None and abs(d - target) > tol:
            continue

        # vectorised distance for all N points in one shot
        dist  = np.abs(pts_T[0]*n[0] + pts_T[1]*n[1] + pts_T[2]*n[2] + d)
        mask  = dist < dist_thresh
        count = int(mask.sum())

        if count > best_inliers and count >= min_inliers:
            best_inliers = count
            n_best       = n.copy()
            d_best       = float(d)
            mask_best    = mask.copy()

            # ── Early exit ────────────────────────────────────────────────────
            # If this plane already explains most of the scene, further
            # iterations are very unlikely to do better.
            if count >= N * early_exit_ratio:
                break

    if n_best is None:
        return False, None, None, None

    # ── SVD refinement ────────────────────────────────────────────────────────
    # The 3-point RANSAC winner is a rough estimate.
    # Refitting a plane through ALL inliers with SVD gives a much more
    # accurate normal, which tightens the full-res mask in apply_plane_to_full_depth.
    inlier_pts = pts[mask_best]
    centroid   = inlier_pts.mean(axis=0)
    _, _, Vt   = np.linalg.svd(inlier_pts - centroid)
    n_refined  = Vt[-1]                        # eigenvector of smallest variance
    if np.dot(n_refined, up) < 0:
        n_refined = -n_refined

    # Only use refinement if it still passes the angle constraint
    if np.dot(n_refined, up) >= cos_thresh:
        d_refined = -np.dot(n_refined, centroid)
    else:
        n_refined, d_refined = n_best, d_best

    return True, n_refined, d_refined, mask_best


def main(depth_full):
    # 1. Subsampled backprojection (Y-band filter baked in)
    pts_sub, _, _ = backproject_depth_sub(depth_full)

    if len(pts_sub) < 100:
        print("❌ Not enough valid depth points to find a floor.")
        cv2.imshow("obstacle avoidance", depth_full)
        cv2.waitKey(1)
        return

    # 2. RANSAC + SVD refinement
    success, n_best, d_best, _ = ransac_plane(
        pts=pts_sub,
        iters=400,
        dist_thresh=0.08,
        angle_thresh_deg=30,
        min_inliers=450,
    )

    depth_vis = cv2.normalize(depth_full, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

    if success:
        # 3. Full-res mask — pure vectorised math, no second backprojection
        ground_mask_2d = apply_plane_to_full_depth(depth_full, n_best, d_best, dist_thresh=0.06)

        mask_8u = (ground_mask_2d * 255).astype(np.uint8)
        kernel  = np.ones((5, 5), np.uint8)
        mask_8u = cv2.morphologyEx(mask_8u, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_8u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            largest_area    = cv2.contourArea(largest_contour)

            if largest_area > 50000:
                floor_mask = np.zeros_like(mask_8u)
                cv2.drawContours(floor_mask, [largest_contour], -1, 255, cv2.FILLED)
                depth_vis[floor_mask == 255] = (255, 0, 0)
                print(f"✅ Solid Floor! Continuous Area: {largest_area:.0f} px")
            else:
                cv2.putText(depth_vis, "Plane found, not continuous",
                            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
        else:
            print("❌ RANSAC found a plane but no valid contours.")
    else:
        cv2.putText(depth_vis, "No Ground Found",
                    (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("obstacle avoidance", depth_vis)
    cv2.waitKey(1)

if __name__ == "__main__":
    with dai.Pipeline() as pipeline:
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(1280, 720)

        config = stereo.initialConfig

        # Median filter to remove the salt n pepper type pixels
        config.postProcessing.median = dai.MedianFilter.KERNEL_7x7
        config.postProcessing.thresholdFilter.maxRange = 8000  # 8.0m

        config.setConfidenceThreshold(40)
        config.setSubpixel(True)
        config.setExtendedDisparity(True)

        monoLeftOut = monoLeft.requestOutput((1280, 720))
        monoRightOut = monoRight.requestOutput((1280, 720))

        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

        rightOut = monoRightOut.createOutputQueue()
        stereoOut = stereo.depth.createOutputQueue()

        pipeline.start()
        while pipeline.isRunning():
            start_time = time.perf_counter()
            ## --- Depth Data Processing ---
            stereoFrame = stereoOut.get()

            assert stereoFrame.validateTransformations()

            # Get frame and convert to meters
            depth = stereoFrame.getCvFrame().astype(np.float32) / 1000.0

            # Call the processing function, now passing the heading
            main(depth_full=depth)

            # if cv2.waitKey(1) == ord('q'):
            #     break
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print(f"Elapsed time: {elapsed_time:.4f} seconds")
        pipeline.stop()

    cv2.destroyAllWindows()
    ## --- END: Pipeline and Device Loop ---