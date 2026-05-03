import os
import shutil
import tempfile
import numpy as np
import open3d as o3d
import networkx as nx
import dmpcd as dm
import dmpcd.pcd as pcd

try:
    from MomentumConnect import MomentumConnect
except ImportError:
    import sys
    # Fallback to look at the project root for MomentumConnect
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from MomentumConnect import MomentumConnect

def filter_marker2_by_detour(P, E, tau_detour=1.5, base_markers=(-1, 1)):
    """
    P: (N,3) float array of points
    E: (M,3) int array of [u, v, marker]
    """
    G = nx.Graph()
    G.add_nodes_from(range(len(P)))
    for u, v, m in E:
        if m in base_markers:
            w = float(np.linalg.norm(P[v] - P[u]))
            if w > 0:
                G.add_edge(int(u), int(v), weight=w)

    E2 = E[E[:, 2] == 2, :2]
    keep = []
    delete = []

    for u, v in E2:
        u = int(u); v = int(v)
        Le = float(np.linalg.norm(P[v] - P[u]))
        if Le <= 0:
            continue
        try:
            Lg = float(nx.shortest_path_length(G, u, v, weight="weight"))
        except nx.NetworkXNoPath:
            continue 

        DR = Lg / Le
        if DR <= tau_detour:
            keep.append([u, v])
        else: 
            delete.append([u, v])

    return np.array(keep, dtype=int), np.array(delete, dtype=int)

def edge_length_percentile_filter(P, edges_full, percentile=75):
    edges_full = np.asarray(edges_full, dtype=int).reshape(-1, 3)
    if edges_full.shape[0] == 0:
        return edges_full, edges_full[:0], float("inf")

    uv = edges_full[:, :2]
    lengths = np.linalg.norm(P[uv[:, 1]] - P[uv[:, 0]], axis=1)

    threshold = float(np.percentile(lengths, percentile))
    mask = lengths <= threshold

    kept_full = edges_full[mask]
    removed_full = edges_full[~mask]
    return kept_full, removed_full, threshold

def generate_graph(
    pcd: o3d.geometry.PointCloud,
    tau_detour: float = 1.5,
    keep_tau: float = 0.5,
    voxel_size: float = None,
    k: int = 15,
    metric: str = 'euclidean',
    epsilon: float = 0.99,
    persistence_threshold: float = 0.99,
    length_percentile: float = 75.0
):
    """
    Generate a discrete Morse graph on a given Open3D point cloud object.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.
        tau_detour (float): Threshold for detour filter.
        keep_tau (float): Threshold for momentum connect.
        voxel_size (float): Voxel size for downsampling. If None, input is used directly.
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
    # Downsample if specified
    if voxel_size is not None:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    else:
        pcd_down = pcd

    points = np.asarray(pcd_down.points)

    with tempfile.TemporaryDirectory() as tmpdir:
        # Create a subdirectory for outputs specifically
        output_dir = os.path.join(tmpdir, "output")
        os.makedirs(output_dir, exist_ok=True)

        feature_filename = os.path.join(tmpdir, "features.txt")
        np.savetxt(feature_filename, points, fmt="%.6f")

        dm.pcd.build_sparse_weighted_rips_filtration(
            feature_filename, output_dir, k, metric, epsilon
        )
        
        filtration_filename = os.path.join(output_dir, 'sparse_weighted_rips_filtration.txt')
        weights_filename = os.path.join(output_dir, 'weights.txt')

        dm.pcd.compute_persistence_swr(filtration_filename, output_dir)
        
        edge_filename = os.path.join(output_dir, "edge_for_morse_only.txt")
        sorted_weights_filename = os.path.join(output_dir, "sorted-weights.txt")

        dm.pcd.reorder_weights(weights_filename, sorted_weights_filename)

        morse_dir = output_dir
        dm.pcd.compute_graph_reconstruction(
            sorted_weights_filename, edge_filename, persistence_threshold, morse_dir
        )
        
        potential_edge_files = ['edge.txt', 'dimo_edge.txt']
        edge_txt_path = None
        for f in potential_edge_files:
            p = os.path.join(output_dir, f)
            if os.path.exists(p):
                edge_txt_path = p
                break
                
        if not edge_txt_path:
            raise RuntimeError("Graph reconstruction failed to produce an edge file. "
                               "Check for C++ errors in underlying binaries.")
            
        final_edge_txt_path = os.path.join(output_dir, "edge.txt")
        if edge_txt_path != final_edge_txt_path:
            os.rename(edge_txt_path, final_edge_txt_path)
            edge_txt_path = final_edge_txt_path
        
        sorted_feature_filename = os.path.join(output_dir, 'sorted-feature.txt')
        dm.pcd.reorder_verts_by_weight(weights_filename, feature_filename, sorted_feature_filename)

        points_sorted = np.loadtxt(sorted_feature_filename)
        if points_sorted.ndim == 1:
            points_sorted = points_sorted.reshape(-1, 3)
        if len(points_sorted) > 0 and points_sorted.shape[1] == 2:
            points_sorted = np.hstack([points_sorted, np.zeros((points_sorted.shape[0], 1))])

        edges_list = []
        with open(edge_txt_path) as f:
            for line in f:
                s = line.strip().split()
                if len(s) >= 3:
                    u, v, m = int(s[0]), int(s[1]), int(s[2])
                    edges_list.append([u, v, m])
        
        if not edges_list:
            E = np.empty((0, 3), dtype=int)
        else:
            E = np.array(edges_list, dtype=int)
        
        good2, bad2 = filter_marker2_by_detour(points_sorted, E, tau_detour=tau_detour)

        base_mask = np.isin(E[:, 2], (-1, 1))
        base_edges = E[base_mask, :2]
        
        filtered_edges = np.vstack([
            base_edges,
            good2
        ]) if len(good2) > 0 else base_edges
        
        added_back = MomentumConnect(filtered_edges, bad2, points_sorted, keep_tau, 30)
        
        final_edges_indices = np.vstack([
            filtered_edges,
            added_back
        ]) if len(added_back) > 0 else filtered_edges

        base_edges_full = E[base_mask, :]
        good2_full = np.hstack([good2, 2 * np.ones((len(good2), 1), dtype=int)]) if len(good2) > 0 else np.empty((0, 3), dtype=int)
        added_back_full = np.hstack([added_back, 2 * np.ones((len(added_back), 1), dtype=int)]) if len(added_back) > 0 else np.empty((0, 3), dtype=int)

        final_edges_full = np.vstack([base_edges_full, good2_full, added_back_full])

        if final_edges_full.shape[0] > 0:
            final_edges_full, removed_edges_full, thr = edge_length_percentile_filter( points_sorted, final_edges_full, percentile=length_percentile )
        
        # Visualize
        pcd_vis = o3d.geometry.PointCloud()
        pcd_vis.points = o3d.utility.Vector3dVector(points_sorted)
        pcd_vis.paint_uniform_color([0.1, 0.7, 0.9])

        line_set_vis = o3d.geometry.LineSet()
        line_set_vis.points = o3d.utility.Vector3dVector(points_sorted)
        if len(final_edges_full) > 0:
            line_set_vis.lines = o3d.utility.Vector2iVector(final_edges_full[:, :2])
        line_set_vis.paint_uniform_color([0, 0, 0])

        return {
            'nodes': points_sorted,
            'edges': final_edges_full[:, :2] if len(final_edges_full) > 0 else np.empty((0, 2), dtype=int),
            'edges_full': final_edges_full,
            'pcd_vis': pcd_vis,
            'line_set_vis': line_set_vis
        }
