import os
import shutil
import numpy as np
import open3d as o3d
import networkx as nx
from collections import Counter, defaultdict
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN
import dmpcd as dm
import dmpcd.pcd as pcd

try:
    from MomentumConnect import MomentumConnect
except ImportError:
    import sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from MomentumConnect import MomentumConnect

def crop_edges_xz(pcd, fraction=0.2):
    if fraction < 0 or fraction >= 0.5:
        raise ValueError("Fraction must be between 0.0 and 0.499")

    points = np.asarray(pcd.points)
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)

    width_x = max_bound[0] - min_bound[0]
    width_z = max_bound[2] - min_bound[2]

    margin_x = width_x * fraction
    margin_z = width_z * fraction

    new_min_bound = np.array([min_bound[0] + margin_x, min_bound[1], min_bound[2] + margin_z])
    new_max_bound = np.array([max_bound[0] - margin_x, max_bound[1], max_bound[2] - margin_z])

    bbox = o3d.geometry.AxisAlignedBoundingBox(new_min_bound, new_max_bound)
    cropped_pcd = pcd.crop(bbox)
    inside_indices = bbox.get_point_indices_within_bounding_box(pcd.points)
    remainder_pcd = pcd.select_by_index(inside_indices, invert=True)

    return cropped_pcd, remainder_pcd

def farthest_point_sampling(points, n_samples):
    N = points.shape[0]
    if n_samples >= N:
        return np.arange(N)

    selected_indices = [np.random.randint(N)]
    distances = np.full(N, np.inf)

    for _ in range(1, n_samples):
        last_point = points[selected_indices[-1]]
        dists = np.linalg.norm(points - last_point, axis=1)
        distances = np.minimum(distances, dists)
        next_index = np.argmax(distances)
        selected_indices.append(next_index)

    return np.array(selected_indices)

def build_knn(points, k=10):
    nbrs = NearestNeighbors(n_neighbors=k).fit(points)
    _, indices = nbrs.kneighbors(points)
    return indices

def laplacian_contraction(points, knn_idx, lam=0.2, iterations=8):
    pts = points.copy()
    N, k = knn_idx.shape

    for _ in range(iterations):
        new_pts = pts.copy()
        for i in range(N):
            neigh_idx = knn_idx[i, 1:]  
            neigh = pts[neigh_idx]

            if len(neigh) == 0:
                continue

            centroid = neigh.mean(axis=0)
            degree = len(neigh)
            lam_eff = lam / (1.0 + degree * 0.1)
            new_pts[i] = pts[i] + lam_eff * (centroid - pts[i])
        pts = new_pts

    return pts

def tensor_vote_extrapolate(points, search_radius=5.0, vote_steps=5, step_size=1.0, max_nn=30, n_neighbors=10):
    points = np.asarray(points)
    num_points = len(points)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=max_nn))
    pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=max_nn))
    
    covariances = np.asarray(pcd.covariances) 
    new_points = []
    
    nbrs = NearestNeighbors(n_neighbors=n_neighbors, algorithm='kd_tree').fit(points)
    
    for i in range(num_points):
        cov = covariances[i]
        eigvals, eigvecs = np.linalg.eigh(cov) 
        
        idx = eigvals.argsort()[::-1]
        eigvals = eigvals[idx]
        eigvecs = eigvecs[:, idx]
        
        lambda1, lambda2, lambda3 = eigvals
        v1 = eigvecs[:, 0] 
        if lambda1 < 1e-6: continue 
        linearity = (lambda1 - lambda2) / lambda1
        if linearity > 0.8:
            indices = nbrs.kneighbors([points[i]], return_distance=False)[0]
            local_cloud = points[indices]
            centroid = np.mean(local_cloud, axis=0)
            
            vec_to_center = centroid - points[i]
            alignment = np.dot(vec_to_center, v1)
            shoot_dir = -v1 if alignment > 0 else v1
                
            if np.linalg.norm(vec_to_center) > (search_radius * 0.2):
                current_pos = points[i].copy()
                for s in range(vote_steps):
                    current_pos += shoot_dir * step_size
                    new_points.append(current_pos)
 
    return np.array(new_points)

def perform_advanced_skeletonization(points):
    """Wraps the entire DBSCAN -> Contraction -> Tensor Vote pipeline."""
    nbrs = NearestNeighbors(n_neighbors=2).fit(points)
    distances, _ = nbrs.kneighbors(points)
    median_dist = np.median(distances[:, 1])
    print(f"  Median distance between points: {median_dist}")

    eps = median_dist * 8
    clustering = DBSCAN(eps=eps, min_samples=5).fit(points)
    labels = clustering.labels_

    downsampled_points = []

    print("  Skeletonizing clusters via Contraction...")
    for label in np.unique(labels):
        cluster_points = points[labels == label]
        
        sample_ratio = 0.02
        k_pts = max(1, int(len(cluster_points) * sample_ratio))
        fps_indices = farthest_point_sampling(cluster_points, k_pts)
        cluster_points_reduced = cluster_points[fps_indices]
        
        if len(cluster_points_reduced) < 15:
            downsampled_points.append(cluster_points_reduced)
            continue

        knn_idx = build_knn(cluster_points_reduced, k=8)
        contracted = laplacian_contraction(cluster_points_reduced, knn_idx, lam=0.4, iterations=30)
        downsampled_points.append(contracted)

    downsampled_points = np.vstack(downsampled_points)
    pcd_down = o3d.geometry.PointCloud()
    pcd_down.points = o3d.utility.Vector3dVector(downsampled_points)

    print("  Applying crop to XZ edges (15%)...")
    cropped_pcd, remainder_pcd = crop_edges_xz(pcd_down, fraction=0.15)

    print("  Starting Tensor Vote Extrapolation...")
    base_points = np.asarray(cropped_pcd.points)
    accumulated_new_points = np.empty((0, 3))
    voxel_size = 1.1 

    for i in range(8):
        print(f"    Growth Pass {i+1}...")
        context_points = np.vstack((base_points, accumulated_new_points)) if len(accumulated_new_points) > 0 else base_points

        new_growth = tensor_vote_extrapolate(
            context_points, search_radius=20.0, vote_steps=5, step_size=2.5, max_nn=30, n_neighbors=10
        )
        if len(new_growth) == 0:
            break
            
        accumulated_new_points = np.vstack((accumulated_new_points, new_growth))
        
        pcd_temp = o3d.geometry.PointCloud()
        pcd_temp.points = o3d.utility.Vector3dVector(accumulated_new_points)
        pcd_temp = pcd_temp.voxel_down_sample(voxel_size=voxel_size)
        accumulated_new_points = np.asarray(pcd_temp.points)

    remainder_points = np.asarray(remainder_pcd.points)

    print(f"  Result: {len(base_points)} base + {len(accumulated_new_points)} extrapolated + {len(remainder_points)} outer.")
    if len(accumulated_new_points) > 0:
        return np.vstack((base_points, accumulated_new_points, remainder_points))
    return np.vstack((base_points, remainder_points))


# =========================================================
# PHASE 2: GRAPH RECONSTRUCTION & MERGING FUNCTIONS
# =========================================================

def filter_marker2_by_detour(P, E, tau_detour=1.5, base_markers=(-1, 1)):
    G = nx.Graph()
    G.add_nodes_from(range(len(P)))
    for u, v, m in E:
        if m in base_markers:
            w = float(np.linalg.norm(P[v] - P[u]))
            if w > 0:
                G.add_edge(int(u), int(v), weight=w)

    E2 = E[E[:, 2] == 2, :2]
    keep, delete = [], []
    for u, v in E2:
        u, v = int(u), int(v)
        Le = float(np.linalg.norm(P[v] - P[u]))
        if Le <= 0: continue
        try:
            Lg = float(nx.shortest_path_length(G, u, v, weight="weight"))
        except nx.NetworkXNoPath:
            continue
        
        if (Lg / Le) <= tau_detour:
            keep.append([u, v])
        else: 
            delete.append([u, v])

    return np.array(keep, dtype=int), np.array(delete, dtype=int)

def edge_length_percentile_filter(P, edges_full, percentile=75):
    edges_full = np.asarray(edges_full, dtype=int).reshape(-1, 3)
    if edges_full.shape[0] == 0: return edges_full, edges_full[:0], float("inf")
    lengths = np.linalg.norm(P[edges_full[:, 1]] - P[edges_full[:, 0]], axis=1)
    threshold = float(np.percentile(lengths, percentile))
    mask = lengths <= threshold
    return edges_full[mask], edges_full[~mask], threshold

def split_cloud_with_overlap(points, inner_ratio=0.8, overlap_ratio=0.02):
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)
    rng = max_bound - min_bound

    margin = (1.0 - inner_ratio) / 2.0
    inner_min = min_bound + (margin - overlap_ratio) * rng
    inner_max = max_bound - (margin - overlap_ratio) * rng
    
    outer_exclusion_min = min_bound + (margin + overlap_ratio) * rng
    outer_exclusion_max = max_bound - (margin + overlap_ratio) * rng

    in_x = (points[:, 0] >= inner_min[0]) & (points[:, 0] <= inner_max[0])
    in_z = (points[:, 2] >= inner_min[2]) & (points[:, 2] <= inner_max[2])
    mask_inner = in_x & in_z

    out_x = (points[:, 0] < outer_exclusion_min[0]) | (points[:, 0] > outer_exclusion_max[0])
    out_z = (points[:, 2] < outer_exclusion_min[2]) | (points[:, 2] > outer_exclusion_max[2])
    mask_outer = out_x | out_z

    return points[mask_inner], points[mask_outer]

def run_reconstruction(points, output_dir, k, metric, epsilon, persistence_threshold, tau_detour, keep_tau, prefix=""):
    if os.path.exists(output_dir): shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    feature_filename = os.path.join(output_dir, "features.txt")
    np.savetxt(feature_filename, points, fmt="%.6f")

    dm.pcd.build_sparse_weighted_rips_filtration(feature_filename, output_dir, k, metric, epsilon)
    
    filtration_filename = os.path.join(output_dir, 'sparse_weighted_rips_filtration.txt')
    weights_filename = os.path.join(output_dir, 'weights.txt')
    dm.pcd.compute_persistence_swr(filtration_filename, output_dir)
    
    edge_filename = os.path.join(output_dir, "edge_for_morse_only.txt")
    sorted_weights_filename = os.path.join(output_dir, "sorted-weights.txt")
    dm.pcd.reorder_weights(weights_filename, sorted_weights_filename)

    dm.pcd.compute_graph_reconstruction(sorted_weights_filename, edge_filename, persistence_threshold, output_dir)
    
    edge_txt_path = next((os.path.join(output_dir, f) for f in ['edge.txt', 'dimo_edge.txt'] if os.path.exists(os.path.join(output_dir, f))), None)
    if not edge_txt_path: raise RuntimeError(f"[{prefix}] C++ Graph reconstruction failed.")
        
    sorted_feature_filename = os.path.join(output_dir, 'sorted-feature.txt')
    dm.pcd.reorder_verts_by_weight(weights_filename, feature_filename, sorted_feature_filename)

    points_sorted = np.loadtxt(sorted_feature_filename)
    if points_sorted.shape[1] == 2: points_sorted = np.hstack([points_sorted, np.zeros((points_sorted.shape[0], 1))])

    edges_list = []
    with open(edge_txt_path) as f:
        for line in f:
            s = line.strip().split()
            if len(s) >= 3: edges_list.append([int(s[0]), int(s[1]), int(s[2])])
    
    E = np.array(edges_list, dtype=int) if edges_list else np.empty((0, 3), dtype=int)

    good2, bad2 = filter_marker2_by_detour(points_sorted, E, tau_detour=tau_detour)
    base_mask = np.isin(E[:, 2], (-1, 1))
    filtered_edges = np.vstack([E[base_mask, :2], good2]) if len(good2) > 0 else E[base_mask, :2]
    
    added_back = MomentumConnect(filtered_edges, bad2, points_sorted, keep_tau, 30) 
    
    final_edges_full = np.vstack([
        E[base_mask, :],
        np.hstack([good2, 2 * np.ones((len(good2), 1), dtype=int)]) if len(good2) > 0 else np.empty((0, 3), dtype=int),
        np.hstack([added_back, 2 * np.ones((len(added_back), 1), dtype=int)]) if len(added_back) > 0 else np.empty((0, 3), dtype=int)
    ])

    final_edges_full, _, _ = edge_length_percentile_filter(points_sorted, final_edges_full, percentile=99.95)
    return points_sorted, final_edges_full

def merge_graphs(P_inner, E_inner, P_outer, E_outer):
    P_combined = P_inner.copy()
    nbrs = NearestNeighbors(n_neighbors=1, radius=1e-4).fit(P_inner)
    distances, indices = nbrs.kneighbors(P_outer)

    P_outer_to_combined = np.zeros(len(P_outer), dtype=int)
    new_pts = []

    for i, (dist, idx) in enumerate(zip(distances, indices)):
        if dist[0] < 1e-4: P_outer_to_combined[i] = idx[0]
        else: 
            new_idx = len(P_combined) + len(new_pts)
            P_outer_to_combined[i] = new_idx
            new_pts.append(P_outer[i])

    if new_pts: P_combined = np.vstack([P_combined, np.array(new_pts)])

    E_outer_mapped = E_outer.copy()
    E_outer_mapped[:, 0] = P_outer_to_combined[E_outer[:, 0]]
    E_outer_mapped[:, 1] = P_outer_to_combined[E_outer[:, 1]]

    return P_combined, np.vstack([E_inner, E_outer_mapped])

def handshake_join(P, E, max_snap_distance=15.0):
    uv = E[:, :2].astype(int)
    deg = Counter()
    for u, v in uv:
        deg[u] += 1; deg[v] += 1
        
    leaves = [v for v, d in deg.items() if d == 1]
    if len(leaves) < 2: return E
        
    leaf_pts = P[leaves]
    nbrs = NearestNeighbors(n_neighbors=2).fit(leaf_pts)
    distances, indices = nbrs.kneighbors(leaf_pts)
    
    new_edges, connected = [], set()
    for i, leaf_idx in enumerate(leaves):
        if leaf_idx in connected: continue
        dist = distances[i, 1]
        closest_leaf_idx = leaves[indices[i, 1]]
        if dist <= max_snap_distance:
            new_edges.append([leaf_idx, closest_leaf_idx, 9]) 
            connected.add(leaf_idx)
            connected.add(closest_leaf_idx)
            
    if new_edges: return np.vstack([E, np.array(new_edges, dtype=int)])
    return E


# =========================================================
# PHASE 3: RECONNECTION GRAPH UTILITIES
# =========================================================

def _dedup_undirected_edges_uv(edges_uv):
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
    if edges_uv.size == 0: return edges_uv
    uv = np.sort(edges_uv, axis=1)
    _, idx = np.unique(uv, axis=0, return_index=True)
    return edges_uv[np.sort(idx)]

def _degree1_vertices(edges_uv):
    deg = Counter()
    for u, v in edges_uv:
        deg[int(u)] += 1; deg[int(v)] += 1
    return [v for v, d in deg.items() if d == 1], deg

def _build_adj(edges_uv):
    adj = defaultdict(list)
    for u, v in edges_uv:
        adj[int(u)].append(int(v))
        adj[int(v)].append(int(u))
    return adj

def _max_edge_length(points, edges_uv):
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
    if edges_uv.size == 0: return 0.0
    d = points[edges_uv[:, 1]] - points[edges_uv[:, 0]]
    return float(np.max(np.linalg.norm(d, axis=1)))

def filter_interior_leaves(leaves, P, quantile=0.10, x_axis=0, y_axis=1):
    x_lo, x_hi = np.quantile(P[:, x_axis], [quantile, 1.0 - quantile])
    y_lo, y_hi = np.quantile(P[:, y_axis], [quantile, 1.0 - quantile])
    return [int(u) for u in leaves if (x_lo <= P[int(u), x_axis] <= x_hi) and (y_lo <= P[int(u), y_axis] <= y_hi)]

def prune_degree1_once(edges_uv):
    edges_uv = np.asarray(edges_uv[:, :2], dtype=int).reshape(-1, 2)
    if edges_uv.size == 0: return edges_uv, set()
    deg = Counter()
    for u, v in edges_uv:
        deg[int(u)] += 1; deg[int(v)] += 1
    leaves = {v for v, d in deg.items() if d == 1}
    if not leaves: return edges_uv, set()
    mask = np.array([(u not in leaves) and (v not in leaves) for u, v in edges_uv], dtype=bool)
    return edges_uv[mask], leaves

def _closest_points_on_segments(p1, q1, p2, q2, eps=1e-12):
    d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2
    a = float(np.dot(d1, d1)); e = float(np.dot(d2, d2)); f = float(np.dot(d2, r))
    if a <= eps and e <= eps: return p1, p2, float(np.linalg.norm(p1 - p2)), 0.0, 0.0
    if a <= eps: s = 0.0; t = np.clip(f / e, 0.0, 1.0) if e > eps else 0.0
    else:
        c = float(np.dot(d1, r))
        if e <= eps: t = 0.0; s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(np.dot(d1, d2)); denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom != 0.0 else 0.0
            t = (b * s + f) / e
            if t < 0.0: t = 0.0; s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0: t = 1.0; s = np.clip((b - c) / a, 0.0, 1.0)
    c1 = p1 + d1 * s; c2 = p2 + d2 * t
    return c1, c2, float(np.linalg.norm(c1 - c2)), s, t

def grow_rays_and_connect(points, edges_full_or_uv, tol=1.0, max_length=None, connect_triangle=False):
    P = np.asarray(points, dtype=float); E = np.asarray(edges_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)
    uv = _dedup_undirected_edges_uv(E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int))
    deg1, _ = _degree1_vertices(uv)
    deg1 = filter_interior_leaves(deg1, P, quantile=0.10)
    adj = _build_adj(uv)
    if len(deg1) == 0: return P, E
    L = _max_edge_length(P, uv) if max_length is None else float(max_length)
    if L <= 0: return P, E

    rays = []
    for u in deg1:
        if len(adj[int(u)]) != 1: continue
        dir_vec = P[u] - P[adj[int(u)][0]]
        n = np.linalg.norm(dir_vec)
        if n == 0: continue
        dir_vec = dir_vec / n
        rays.append((P[u], P[u] + dir_vec * L, int(u)))

    new_pts, new_edges_uv = [], []
    for i in range(len(rays)):
        p1, q1, u1 = rays[i]
        for j in range(i + 1, len(rays)):
            p2, q2, u2 = rays[j]
            c1, c2, dist, _, _ = _closest_points_on_segments(p1, q1, p2, q2)
            if dist <= tol:
                new_pts.append(0.5 * (c1 + c2))
                new_idx = P.shape[0] + len(new_pts) - 1
                new_edges_uv.extend([[u1, new_idx], [u2, new_idx]])
                if connect_triangle: new_edges_uv.append([u1, u2])

    if not new_pts: return P, E
    P2 = np.vstack([P, np.asarray(new_pts, dtype=float)])
    uv2 = _dedup_undirected_edges_uv(np.vstack([uv, np.asarray(new_edges_uv, dtype=int)]))

    if has_marker:
        orig_map = {tuple(sorted(x[:2])): x[2] for x in E[:, :3].astype(int)}
        return P2, np.asarray([[int(u), int(v), orig_map.get(tuple(sorted((int(u), int(v)))), 9)] for u, v in uv2], dtype=int)
    return P2, uv2

def _ray_beam_candidates(Pu, d_hat, P_all, beam_radius, max_length, exclude_idx=()):
    W = P_all - Pu; t = W @ d_hat                     
    mask = (t > 0) & (t <= max_length)
    if np.any(mask):
        Wm = W[mask]; tm = t[mask]
        perp = np.linalg.norm(Wm - tm[:, None] * d_hat[None, :], axis=1)
        mask2 = perp <= beam_radius
        cand_idx, t_keep, perp_keep = np.flatnonzero(mask)[mask2], tm[mask2], perp[mask2]
    else: cand_idx, t_keep, perp_keep = np.array([], dtype=int), np.array([], dtype=float), np.array([], dtype=float)

    if exclude_idx and cand_idx.size > 0:
        exclude_idx = set(int(x) for x in exclude_idx)
        keep = np.array([int(i) not in exclude_idx for i in cand_idx], dtype=bool)
        cand_idx, t_keep, perp_keep = cand_idx[keep], t_keep[keep], perp_keep[keep]
    return cand_idx, t_keep, perp_keep

def beam_latch_from_degree1(points, edges_full_or_uv, beam_radius=5.0, max_length=None, pick="forward", perp_tiebreak=True, marker_new=9):
    P = np.asarray(points, dtype=float); E = np.asarray(edges_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)
    uv = _dedup_undirected_edges_uv(E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int))
    deg1, _ = _degree1_vertices(uv)
    deg1 = filter_interior_leaves(deg1, P, quantile=0.10)
    adj = _build_adj(uv)                 
    L = _max_edge_length(P, uv) if max_length is None else float(max_length)

    existing = set(map(tuple, np.sort(uv, axis=1)))
    new_edges = []
    if len(deg1) == 0 or L <= 0: return P, E

    for u in deg1:
        u = int(u)
        if len(adj.get(u, [])) != 1: continue
        v = int(adj[u][0])
        d = P[u] - P[v]      
        n = np.linalg.norm(d)
        if n == 0: continue
        d_hat = d / n

        cand_idx, t, perp = _ray_beam_candidates(P[u], d_hat, P, float(beam_radius), float(L), exclude_idx=(u, v))
        if cand_idx.size == 0: continue

        if pick == "forward":
            order = np.lexsort((perp, t)) if perp_tiebreak else np.argsort(t)
            w = int(cand_idx[order[0]])
        elif pick == "euclid":
            dists = np.linalg.norm(P[cand_idx] - P[u], axis=1)
            order = np.lexsort((perp, dists)) if perp_tiebreak else np.argsort(dists)
            w = int(cand_idx[order[0]])

        key = tuple(sorted((u, w)))
        if key not in existing:
            new_edges.append([u, w])
            existing.add(key)

    if not new_edges: return P, E
    uv2 = _dedup_undirected_edges_uv(np.vstack([uv, np.asarray(new_edges, dtype=int)]))

    if has_marker:
        orig_map = {tuple(sorted(x[:2])): x[2] for x in E[:, :3].astype(int)}
        return P, np.asarray([[int(a), int(b), orig_map.get(tuple(sorted((int(a), int(b)))), int(marker_new))] for a, b in uv2], dtype=int)
    return P, uv2

def _branch_vertices_from_leaf(u: int, adj, deg):
    if deg.get(u, 0) != 1 or len(adj.get(u, [])) != 1: return {u}
    branch = {u}
    prev, cur = u, int(adj[u][0])
    branch.add(cur)
    while deg.get(cur, 0) == 2:
        nbs = adj.get(cur, [])
        nxt = int(nbs[0]) if int(nbs[1]) == prev else int(nbs[1])
        if nxt == prev: break
        prev, cur = cur, nxt
        if cur in branch: break
        branch.add(cur)
    return branch

def connect_interior_leaves_to_nearest_k_no_same_branch_xy(
    points, edges_full_or_uv, k=2, max_dist=np.inf, interior_quantile=0.10, x_axis=0, y_axis=1, 
    marker_new=9, exclude_existing_neighbors=True, exclude_same_branch=True
):
    P = np.asarray(points, dtype=float); E = np.asarray(edges_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)
    uv = _dedup_undirected_edges_uv(E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int))
    leaves, deg = _degree1_vertices(uv)
    adj = _build_adj(uv)

    interior_leaves = filter_interior_leaves(leaves, P, quantile=interior_quantile, x_axis=x_axis, y_axis=y_axis)
    if len(interior_leaves) == 0: return P, E

    existing = set(map(tuple, np.sort(uv, axis=1)))
    new_edges = []

    for u in interior_leaves:
        forbid = {int(u)}
        if exclude_existing_neighbors: forbid.update(int(v) for v in adj.get(u, []))
        if exclude_same_branch: forbid.update(_branch_vertices_from_leaf(u, adj, deg))

        d = np.linalg.norm(P - P[u], axis=1)
        d[list(forbid)] = np.inf

        for w in np.argsort(d)[:k]:
            if not np.isfinite(d[w]) or d[w] > max_dist: continue
            key = tuple(sorted((int(u), int(w))))
            if key not in existing:
                new_edges.append([int(u), int(w)])
                existing.add(key)

    if not new_edges: return P, E
    uv2 = _dedup_undirected_edges_uv(np.vstack([uv, np.asarray(new_edges, dtype=int)]))

    if has_marker:
        orig_map = {tuple(sorted(x[:2])): x[2] for x in E[:, :3].astype(int)}
        return P, np.asarray([[int(a), int(b), orig_map.get(tuple(sorted((int(a), int(b)))), int(marker_new))] for a, b in uv2], dtype=int)
    return P, uv2

def collapse_small_triangles(points, edges_full_or_uv, threshold=5.0, marker_new=9):
    P = np.asarray(points, dtype=float); E = np.asarray(edges_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)
    uv = _dedup_undirected_edges_uv(E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int))
    adj = _build_adj(uv)
    
    small_triangles = []
    nodes = sorted(adj.keys())
    for i, u in enumerate(nodes):
        for v in adj[u]:
            if v <= u or np.linalg.norm(P[u] - P[v]) > threshold: continue
            for w in adj[u]:
                if w <= v or w not in adj[v]: continue
                if np.linalg.norm(P[u] - P[w]) <= threshold and np.linalg.norm(P[v] - P[w]) <= threshold:
                    small_triangles.append((u, v, w))

    if not small_triangles: return P, E
    parent = {n: n for n in nodes}
    def find(i): 
        if parent[i] == i: return i
        parent[i] = find(parent[i]); return parent[i]
    def union(i, j):
        root_i = find(i); root_j = find(j)
        if root_i != root_j: parent[root_i] = root_j

    for u, v, w in small_triangles: union(u, v); union(v, w)

    groups = defaultdict(list)
    for n in nodes: groups[find(n)].append(n)

    P2 = P.copy(); remap = {n: n for n in range(len(P))}
    for root, members in groups.items():
        if len(members) > 1:
            P2[members[0]] = np.mean(P[members], axis=0)
            for m in members: remap[m] = members[0]

    new_edges_uv = [sorted((remap[u], remap[v])) for u, v in uv if remap[u] != remap[v]]
    uv_final = np.unique(new_edges_uv, axis=0) if new_edges_uv else np.empty((0, 2), dtype=int)

    if has_marker:
        orig_map = {tuple(sorted(x[:2])): x[2] for x in E[:, :3].astype(int)}
        return P2, np.asarray([[int(a), int(b), orig_map.get(tuple(sorted((int(a), int(b)))), int(marker_new))] for a, b in uv_final], dtype=int)
    return P2, uv_final

def remove_isolated_points(P, E_full_or_uv):
    P = np.asarray(P, float); E = np.asarray(E_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)
    if E.size == 0: return P[:0], E  

    uv = E[:, :2].astype(int).reshape(-1, 2)
    used = np.zeros(P.shape[0], dtype=bool)
    used[uv[:, 0]] = True; used[uv[:, 1]] = True

    old_idx = np.flatnonzero(used)
    old_to_new = -np.ones(P.shape[0], dtype=int)
    old_to_new[old_idx] = np.arange(old_idx.size)

    if has_marker: return P[old_idx], np.hstack([old_to_new[uv], E[:, 2].astype(int).reshape(-1, 1)])
    return P[old_idx], old_to_new[uv]


# =========================================================
# SYSTEM ARGUMENTS
# =========================================================

