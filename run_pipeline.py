import argparse
import os
import sys
import shutil
import numpy as np
import open3d as o3d
import networkx as nx
import dmpcd as dm
import dmpcd.pcd as pcd
from MomentumConnect import MomentumConnect

def parse_args():
    parser = argparse.ArgumentParser(description="Run PCD Graph Reconstruction Pipeline")
    parser.add_argument("dataset", help="Name of the dataset folder (e.g., tangle01)")
    parser.add_argument("pcd_name", help="Name of the PCD file within the dataset folder")
    parser.add_argument("--tau-detour", type=float, default=1.5, help="Threshold for detour filter")
    parser.add_argument("--keep-tau", type=float, default=0.5, help="Threshold for momentum connect")
    parser.add_argument("--no-vis", action="store_true", help="Disable visualization")
    parser.add_argument("--voxel-size", type=float, default=15, help="Voxel size for downsampling")
    parser.add_argument("--k", type=int, default=15, help="k value for weight estimation")
    parser.add_argument("--metric", type=str, default='euclidean', help="Metric for distance")
    parser.add_argument("--epsilon", type=float, default=0.99, help="Sparsification parameter")
    parser.add_argument("--persistence-threshold", type=float, default=0.99, help="Persistence threshold")
    return parser.parse_args()

def filter_marker2_by_detour(P, E, tau_detour=1.5, base_markers=(-1, 1)):
    """
    P: (N,3) float array of points
    E: (M,3) int array of [u, v, marker]
    """
    # Build base graph (weighted by Euclidean length)
    G = nx.Graph()
    G.add_nodes_from(range(len(P)))
    for u, v, m in E:
        if m in base_markers:
            w = float(np.linalg.norm(P[v] - P[u]))
            if w > 0:
                G.add_edge(int(u), int(v), weight=w)

    # Candidate marker-2 edges
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
            continue  # treat as infinite detour

        DR = Lg / Le
        if DR <= tau_detour:
            keep.append([u, v])
        else: 
            delete.append([u, v])

    return np.array(keep, dtype=int), np.array(delete, dtype=int)

def main():
    args = parse_args()
    dataset = args.dataset
    pcd_name = args.pcd_name

    print(f"Processing dataset: {dataset}, PCD: {pcd_name}")

    # 1. Prepare Data
    data_dir = f"data/{dataset}"
    if not os.path.exists(data_dir):
        print(f"Error: Directory {data_dir} does not exist.")
        sys.exit(1)

    pcd_path = os.path.join(data_dir, pcd_name)
    if not os.path.exists(pcd_path):
        print(f"Error: File {pcd_path} does not exist.")
        sys.exit(1)

    # Load and downsample PCD
    print("Loading and downsampling PCD...")
    pcd_obj = o3d.io.read_point_cloud(pcd_path)
    pcd_down = pcd_obj.voxel_down_sample(args.voxel_size)
    points = np.asarray(pcd_down.points)
    
    # Save features.txt
    feature_filename = os.path.join(data_dir, "features.txt")
    np.savetxt(feature_filename, points, fmt="%.6f")
    print(f"Saved features to {feature_filename}")

    # 2. Run Sparse Weighted Rips Filtration
    output_dir = f"results/{dataset}-pcd/"
    
    # Clean output directory to avoid stale files
    if os.path.exists(output_dir):
        print(f"Cleaning output directory {output_dir}...")
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    print("Building sparse weighted rips filtration...")
    dm.pcd.build_sparse_weighted_rips_filtration(
        feature_filename, output_dir, args.k, args.metric, args.epsilon
    )
    
    filtration_filename = os.path.join(output_dir, 'sparse_weighted_rips_filtration.txt')
    weights_filename = os.path.join(output_dir, 'weights.txt')

    print("Computing persistence...")
    dm.pcd.compute_persistence_swr(filtration_filename, output_dir)
    
    edge_filename = os.path.join(output_dir, "edge_for_morse_only.txt")
    sorted_weights_filename = os.path.join(output_dir, "sorted-weights.txt")

    print("Reordering weights...")
    dm.pcd.reorder_weights(weights_filename, sorted_weights_filename)

    print("Computing graph reconstruction...")
    morse_dir = output_dir
    dm.pcd.compute_graph_reconstruction(
        sorted_weights_filename, edge_filename, args.persistence_threshold, morse_dir
    )
    
    # Check if graph reconstruction succeeded
    potential_edge_files = ['edge.txt', 'dimo_edge.txt']
    edge_txt_path = None
    for f in potential_edge_files:
        p = os.path.join(output_dir, f)
        if os.path.exists(p):
            edge_txt_path = p
            break
            
    if not edge_txt_path:
        print("Error: Graph reconstruction failed to produce an edge file. Check for C++ errors (e.g., bad_alloc).")
        sys.exit(1)
        
    # Normalize name to edge.txt
    final_edge_txt_path = os.path.join(output_dir, "edge.txt")
    if edge_txt_path != final_edge_txt_path:
        os.rename(edge_txt_path, final_edge_txt_path)
        edge_txt_path = final_edge_txt_path
    
    sorted_feature_filename = os.path.join(output_dir, 'sorted-feature.txt')
    print("Reordering vertices by weight...")
    dm.pcd.reorder_verts_by_weight(weights_filename, feature_filename, sorted_feature_filename)

    # 3. Filter and Refine
    print("Filtering edges...")
    
    # Load points (sorted)
    points_sorted = np.loadtxt(sorted_feature_filename)
    if points_sorted.shape[1] == 2:
        points_sorted = np.hstack([points_sorted, np.zeros((points_sorted.shape[0], 1))])

    # Load edges
    edges_list = []
    with open(edge_txt_path) as f:
        for line in f:
            s = line.strip().split()
            if len(s) >= 3:
                u, v, m = int(s[0]), int(s[1]), int(s[2])
                edges_list.append([u, v, m])
    
    if not edges_list:
        print("Warning: No edges found in graph.")
        E = np.empty((0, 3), dtype=int)
    else:
        E = np.array(edges_list, dtype=int)
    
    # Detour Filter
    good2, bad2 = filter_marker2_by_detour(points_sorted, E, tau_detour=args.tau_detour)
    
    # Build filtered edges
    base_mask = np.isin(E[:, 2], (-1, 1))
    base_edges = E[base_mask, :2]
    
    filtered_edges = np.vstack([
        base_edges,
        good2
    ])
    
    # Momentum Connect
    print("Applying Momentum Connect...")
    added_back = MomentumConnect(filtered_edges, bad2, points_sorted, args.keep_tau, 30) # 30 is keepDist, hardcoded in original
    
    final_edges_indices = np.vstack([
        filtered_edges,
        added_back
    ]) if len(added_back) > 0 else filtered_edges

    # Save final edges
    output_path = os.path.join(output_dir, "edge_detour_filtered.txt")
    
    # Reconstruct full edge list with markers for saving
    # Base edges: keep original marker
    base_edges_full = E[base_mask, :]
    
    # Good2: marker 2
    good2_full = np.hstack([good2, 2 * np.ones((len(good2), 1), dtype=int)]) if len(good2) > 0 else np.empty((0, 3), dtype=int)
    
    # Added back: marker 2 (assuming they are recovered marker 2 edges)
    added_back_full = np.hstack([added_back, 2 * np.ones((len(added_back), 1), dtype=int)]) if len(added_back) > 0 else np.empty((0, 3), dtype=int)

    final_edges_full = np.vstack([base_edges_full, good2_full, added_back_full])
    np.savetxt(output_path, final_edges_full, fmt="%d")
    print(f"Saved filtered edges to {output_path}")

    # 4. Visualize
    if not args.no_vis:
        print("Visualizing...")
        pcd_vis = o3d.geometry.PointCloud()
        pcd_vis.points = o3d.utility.Vector3dVector(points_sorted)
        pcd_vis.paint_uniform_color([0.1, 0.7, 0.9])

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points_sorted)
        line_set.lines = o3d.utility.Vector2iVector(final_edges_indices)
        line_set.paint_uniform_color([0, 0, 0])

        o3d.visualization.draw_geometries([pcd_vis, line_set])

if __name__ == "__main__":
    main()
