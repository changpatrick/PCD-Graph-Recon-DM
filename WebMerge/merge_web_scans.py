#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import copy
import argparse
import os
import sys
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors

# =========================================================
# SKELETONIZATION LOGIC
# =========================================================

def farthest_point_sampling(points, n_samples):
    N = points.shape[0]
    if n_samples >= N: return np.arange(N)
    selected_indices = [np.random.randint(N)]
    distances = np.full(N, np.inf)
    for _ in range(1, n_samples):
        last_point = points[selected_indices[-1]]
        dists = np.linalg.norm(points - last_point, axis=1)
        distances = np.minimum(distances, dists)
        next_index = np.argmax(distances)
        selected_indices.append(next_index)
    return np.array(selected_indices)

def laplacian_contraction(points, knn_idx, lam=0.4, iterations=30):
    pts = points.copy()
    N = pts.shape[0]
    for _ in range(iterations):
        new_pts = pts.copy()
        for i in range(N):
            neigh_idx = knn_idx[i, 1:]
            neigh = pts[neigh_idx]
            if len(neigh) == 0: continue
            centroid = neigh.mean(axis=0)
            lam_eff = lam / (1.0 + len(neigh) * 0.1)
            new_pts[i] = pts[i] + lam_eff * (centroid - pts[i])
        pts = new_pts
    return pts

def tensor_vote_extrapolate(points, search_radius=20.0, vote_steps=5, step_size=2.5):
    points = np.asarray(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=30))
    pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=30))
    covariances = np.asarray(pcd.covariances) 
    new_points = []
    nbrs = NearestNeighbors(n_neighbors=10).fit(points)
    for i in range(len(points)):
        cov = covariances[i]
        eigvals, eigvecs = np.linalg.eigh(cov) 
        idx = eigvals.argsort()[::-1]
        v1 = eigvecs[:, idx[0]]
        lambda1, lambda2 = eigvals[idx[0]], eigvals[idx[1]]
        if lambda1 < 1e-6: continue 
        if (lambda1 - lambda2) / lambda1 > 0.8:
            indices = nbrs.kneighbors([points[i]], return_distance=False)[0]
            local_cloud = points[indices]
            vec_to_center = np.mean(local_cloud, axis=0) - points[i]
            shoot_dir = -v1 if np.dot(vec_to_center, v1) > 0 else v1
            if np.linalg.norm(vec_to_center) > (search_radius * 0.2):
                curr = points[i].copy()
                for _ in range(vote_steps):
                    curr += shoot_dir * step_size
                    new_points.append(curr)
    return np.array(new_points)

def skeletonize(pcd_path):
    print(f"Skeletonizing {os.path.basename(pcd_path)}...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        raise FileNotFoundError(f"Could not load PCD file at {pcd_path}")
    points = np.asarray(pcd.points)
    
    nbrs = NearestNeighbors(n_neighbors=2).fit(points)
    dists, _ = nbrs.kneighbors(points)
    eps = np.median(dists[:, 1]) * 8
    labels = DBSCAN(eps=eps, min_samples=5).fit(points).labels_
    
    skel_pts = []
    for lbl in np.unique(labels):
        c_pts = points[labels == lbl]
        k = max(1, int(len(c_pts) * 0.02))
        fps_idx = farthest_point_sampling(c_pts, k)
        c_red = c_pts[fps_idx]
        if len(c_red) < 15: skel_pts.append(c_red); continue
        knn_idx = NearestNeighbors(n_neighbors=8).fit(c_red).kneighbors(c_red, return_distance=False)
        skel_pts.append(laplacian_contraction(c_red, knn_idx))
    
    base_points = np.vstack(skel_pts)
    accum_new = np.empty((0, 3))
    for i in range(8):
        ctx = np.vstack((base_points, accum_new)) if len(accum_new) > 0 else base_points
        growth = tensor_vote_extrapolate(ctx)
        if len(growth) == 0: break
        accum_new = np.vstack((accum_new, growth))
        p_temp = o3d.geometry.PointCloud()
        p_temp.points = o3d.utility.Vector3dVector(accum_new)
        accum_new = np.asarray(p_temp.voxel_down_sample(1.1).points)
    
    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(np.vstack((base_points, accum_new)) if len(accum_new) > 0 else base_points)
    return final_pcd

# =========================================================
# REGISTRATION LOGIC
# =========================================================

def register_and_fuse(source_skel, target_skel, fusion_voxel=5.0):
    s_center = source_skel.get_center()
    t_center = target_skel.get_center()
    s_temp = copy.deepcopy(source_skel).translate(-s_center)
    t_temp = copy.deepcopy(target_skel).translate(-t_center)

    print("Searching for best Z-rotation...")
    best_fitness, best_T = -1, np.eye(4)
    for deg in range(0, 360, 10):
        angle = np.radians(deg)
        R = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        T_init = np.eye(4); T_init[:3, :3] = R
        reg = o3d.pipelines.registration.registration_icp(
            s_temp, t_temp, 30.0, T_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
        if reg.fitness > best_fitness:
            best_fitness, best_T = reg.fitness, reg.transformation

    print(f"Refining alignment (Best rotation found at approx {np.degrees(np.arctan2(best_T[1,0], best_T[0,0])):.1f} deg)...")
    reg_final = o3d.pipelines.registration.registration_icp(
        s_temp, t_temp, 25.0, best_T, 
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000))

    T_s_c = np.eye(4); T_s_c[:3, 3] = -s_center
    T_t_u = np.eye(4); T_t_u[:3, 3] = t_center
    full_T = T_t_u @ reg_final.transformation @ T_s_c
    
    source_aligned = copy.deepcopy(source_skel).transform(full_T)
    
    # Paint for verification
    source_aligned.paint_uniform_color([1, 0, 0]) # Red
    target_skel.paint_uniform_color([0, 1, 0])    # Green
    
    print(f"Fusing skeletons (voxel size {fusion_voxel})...")
    combined = source_aligned + target_skel
    return combined.voxel_down_sample(voxel_size=fusion_voxel), combined

# =========================================================
# MAIN
# =========================================================

def main():
    parser = argparse.ArgumentParser(description="Merge two spider web scans into a single skeleton.")
    parser.add_argument("pcd1", help="Path to first raw PCD scan")
    parser.add_argument("pcd2", help="Path to second raw PCD scan")
    parser.add_argument("--output", default="merged_skeleton.pcd", help="Path to output fused skeleton")
    parser.add_argument("--fusion-radius", type=float, default=5.0, help="Voxel size for thread fusion")
    args = parser.parse_args()

    try:
        skel1 = skeletonize(args.pcd1)
        skel2 = skeletonize(args.pcd2)
        fused, combined = register_and_fuse(skel1, skel2, fusion_voxel=args.fusion_radius)
        
        # Save final fused result
        o3d.io.write_point_cloud(args.output, fused)
        
        # Save debug verification file
        debug_path = args.output.replace(".pcd", "_debug.pcd")
        o3d.io.write_point_cloud(debug_path, combined)
        
        print(f"\nSUCCESS!")
        print(f"  Unified skeleton saved to: {args.output}")
        print(f"  Color-coded debug view saved to: {debug_path}")
    except Exception as e:
        print(f"\nERROR: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
