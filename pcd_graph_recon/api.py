import os
import shutil
import tempfile
import numpy as np
import open3d as o3d
import networkx as nx

from .utils import (
    perform_advanced_skeletonization,
    split_cloud_with_overlap,
    run_reconstruction,
    merge_graphs,
    handshake_join,
    prune_degree1_once,
    grow_rays_and_connect,
    beam_latch_from_degree1,
    remove_isolated_points,
    connect_interior_leaves_to_nearest_k_no_same_branch_xy,
    collapse_small_triangles,
    edge_length_percentile_filter
)

def generate_graph(
    pcd: o3d.geometry.PointCloud,
    tau_detour: float = 1.5,
    keep_tau: float = 0.5,
    voxel_size: float = 10.0,
    k: int = 15,
    metric: str = 'euclidean',
    epsilon: float = 0.99,
    persistence_threshold: float = 0.99,
    length_percentile: float = 99.95
):
    """
    Generate a discrete Morse graph on a given Open3D point cloud object using the advanced pipeline.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.
        tau_detour (float): Threshold for detour filter.
        keep_tau (float): Threshold for momentum connect.
        voxel_size (float): Voxel size for reconnecting scale.
        k (int): k value for weight estimation.
        metric (str): Metric for distance.
        epsilon (float): Sparsification priority.
        persistence_threshold (float): Persistence threshold.
        length_percentile (float): Percentile for filtering edge lengths.

    Returns:
        dict: A dictionary containing:
            - 'nodes': Filtered and sorted points as a numpy array (N, 3).
            - 'edges': Graph edges as a numpy array (M, 2).
            - 'edges_full': Graph edges with markers as a numpy array (M, 3).
            - 'pcd_vis': An Open3D PointCloud visually represented.
            - 'line_set_vis': An Open3D LineSet representing the graph edges.
    """
    points = np.asarray(pcd.points)

    # 1. PREPROCESSING (Phase 1)
    skeletonized_points = perform_advanced_skeletonization(points)

    # 2. INNER/OUTER SPLIT (Phase 2)
    P_in, P_out = split_cloud_with_overlap(skeletonized_points, inner_ratio=0.8, overlap_ratio=0.02)

    with tempfile.TemporaryDirectory() as tmpdir:
        inner_dir = os.path.join(tmpdir, "inner")
        outer_dir = os.path.join(tmpdir, "outer")

        os.makedirs(inner_dir, exist_ok=True)
        os.makedirs(outer_dir, exist_ok=True)

        P_in_sorted, E_in = run_reconstruction(
            P_in, inner_dir, k, metric, epsilon, persistence_threshold, tau_detour, keep_tau, prefix="INNER"
        )
        P_out_sorted, E_out = run_reconstruction(
            P_out, outer_dir, k, metric, epsilon, persistence_threshold, tau_detour, keep_tau, prefix="OUTER"
        )

    # Merge
    P_comb, E_comb = merge_graphs(P_in_sorted, E_in, P_out_sorted, E_out)

    # Snapping overlap boundaries together
    E_comb = handshake_join(P_comb, E_comb, max_snap_distance=voxel_size * 1.5)

    # 3. GRAPH RECONNECTION (Phase 3)
    MAX_REACH = voxel_size * 3.0 

    # 3a. Grow Rays
    E_pruned1, _ = prune_degree1_once(E_comb)
    P_rays, E_rays = grow_rays_and_connect(
        P_comb, E_pruned1, tol=5.0, max_length=MAX_REACH, connect_triangle=False
    )

    # 3b. Beam Latch
    E_pruned2, _ = prune_degree1_once(E_rays)
    P_latched, E_latched = beam_latch_from_degree1(
        P_rays, E_pruned2, beam_radius=voxel_size * 2.0, max_length=MAX_REACH, pick="forward"
    )

    P_clean1, E_clean1 = remove_isolated_points(P_latched, E_latched)

    # 3c. Connect Interior Leaves
    P_conn, E_conn = connect_interior_leaves_to_nearest_k_no_same_branch_xy(
        P_clean1, E_clean1, k=2, max_dist=MAX_REACH, interior_quantile=0.10, x_axis=0, y_axis=1
    )

    # 3d. Collapse micro-triangles
    P_collapsed, E_collapsed = collapse_small_triangles(
        P_conn, E_conn, threshold=voxel_size * 20 
    )

    # 3e. Final Clean
    P_final, E_final = remove_isolated_points(P_collapsed, E_collapsed)

    pcd_vis = o3d.geometry.PointCloud()
    pcd_vis.points = o3d.utility.Vector3dVector(P_final)
    pcd_vis.paint_uniform_color([0.1, 0.7, 0.9])

    line_set_vis = o3d.geometry.LineSet()
    line_set_vis.points = o3d.utility.Vector3dVector(P_final)
    if len(E_final) > 0:
        line_set_vis.lines = o3d.utility.Vector2iVector(E_final[:, :2])
    line_set_vis.paint_uniform_color([0, 0, 0])

    return {
        'nodes': P_final,
        'edges': E_final[:, :2] if len(E_final) > 0 else np.empty((0, 2), dtype=int),
        'edges_full': E_final,
        'pcd_vis': pcd_vis,
        'line_set_vis': line_set_vis
    }
