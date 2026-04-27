import open3d as o3d
import numpy as np
import copy
import argparse
import os
import sys
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from pcd_graph_recon.api import generate_graph
from RayRecon.RayRecon_simplified_w_Pajeck import rebuild_graph_from_arrays
import open3d as o3d





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

#convert edges into thread segments
def edge_features(P, E):
    features = []

    for idx, (i, j) in enumerate(E):
        p1 = P[i]
        p2 = P[j]

        vec = p2 - p1
        length = np.linalg.norm(vec)

        if length == 0:
            continue

        direction = vec / length
        midpoint = (p1 + p2) / 2

        features.append({
            "edge_idx": idx,
            "i": i,
            "j": j,
            "p1": p1,
            "p2": p2,
            "midpoint": midpoint,
            "direction": direction,
            "length": length
        })

    return features














# =========================================================
# REGISTRATION LOGIC
# =========================================================
def estimate_rough_transform(source_skel, target_skel):
    s_center = source_skel.get_center()
    t_center = target_skel.get_center()

    s_temp = copy.deepcopy(source_skel).translate(-s_center)
    t_temp = copy.deepcopy(target_skel).translate(-t_center)

    best_fitness, best_T = -1, np.eye(4)

    for deg in range(0, 360, 10):
        angle = np.radians(deg)
        R = np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle),  np.cos(angle), 0],
            [0,              0,             1]
        ])

        T_init = np.eye(4)
        T_init[:3, :3] = R

        reg = o3d.pipelines.registration.registration_icp(
            s_temp,
            t_temp,
            30.0,
            T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        if reg.fitness > best_fitness:
            best_fitness = reg.fitness
            best_T = reg.transformation

    reg_final = o3d.pipelines.registration.registration_icp(
        s_temp,
        t_temp,
        25.0,
        best_T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)
    )

    T_s_c = np.eye(4)
    T_s_c[:3, 3] = -s_center

    T_t_u = np.eye(4)
    T_t_u[:3, 3] = t_center

    full_T = T_t_u @ reg_final.transformation @ T_s_c

    return full_T



# =========================================================
# TOPO FIX LOGIC
# =========================================================
#For each edge in scan 2, compare it against nearby edges in scan 1.
def edge_match_score(e1, e2):
    # midpoint distance
    mid_dist = np.linalg.norm(e1["midpoint"] - e2["midpoint"])

    # direction similarity; abs because edge direction can be reversed
    dir_sim = abs(np.dot(e1["direction"], e2["direction"]))

    # endpoint distance, allowing reversed orientation
    same_orientation = (
        np.linalg.norm(e1["p1"] - e2["p1"]) +
        np.linalg.norm(e1["p2"] - e2["p2"])
    )

    flipped_orientation = (
        np.linalg.norm(e1["p1"] - e2["p2"]) +
        np.linalg.norm(e1["p2"] - e2["p1"])
    )

    endpoint_dist = min(same_orientation, flipped_orientation)

    return mid_dist, dir_sim, endpoint_dist

def is_same_edge(
    e1,
    e2,
    max_mid_dist=0.02,
    min_dir_sim=0.90,
    max_endpoint_dist=0.04
):
    mid_dist, dir_sim, endpoint_dist = edge_match_score(e1, e2)

    return (
        mid_dist <= max_mid_dist and
        dir_sim >= min_dir_sim and
        endpoint_dist <= max_endpoint_dist
    )

def apply_transform_to_points(P, T):
    P_h = np.hstack([P, np.ones((len(P), 1))])
    P_t = (T @ P_h.T).T[:, :3]
    return P_t

def find_or_add_vertex(P_merged, p, snap_radius=3.0):
    P_arr = np.asarray(P_merged)

    if len(P_arr) == 0:
        P_merged.append(p.tolist())
        return 0

    dists = np.linalg.norm(P_arr - p, axis=1)
    nearest = int(np.argmin(dists))

    if dists[nearest] <= snap_radius:
        return nearest

    P_merged.append(p.tolist())
    return len(P_merged) - 1
def merge_graphs_topology_aware(
    P1,
    E1,
    P2,
    E2,
    max_mid_dist=5.0,
    min_dir_sim=0.90,
    max_endpoint_dist=10.0,
    snap_radius=3.0
):
    F1 = edge_features(P1, E1)
    F2 = edge_features(P2, E2)

    P_merged = P1.tolist()
    E_merged = [list(map(int, e)) for e in E1]

    for e2 in F2:
        duplicate = False

        for e1 in F1:
            mid_dist, dir_sim, endpoint_dist = edge_match_score(e1, e2)

            if (
                mid_dist <= max_mid_dist and
                dir_sim >= min_dir_sim and
                endpoint_dist <= max_endpoint_dist
            ):
                duplicate = True
                break

        if duplicate:
            continue

        a = find_or_add_vertex(P_merged, e2["p1"], snap_radius=snap_radius)
        b = find_or_add_vertex(P_merged, e2["p2"], snap_radius=snap_radius)

        if a != b:
            E_merged.append([a, b])

    return np.asarray(P_merged), np.asarray(E_merged, dtype=int)

def remove_duplicate_edges(P, E):
    seen = set()
    cleaned = []

    for a, b in E:
        a, b = int(a), int(b)

        if a == b:
            continue

        key = tuple(sorted((a, b)))

        if key not in seen:
            seen.add(key)
            cleaned.append([a, b])

    return P, np.asarray(cleaned, dtype=int)
import networkx as nx

def remove_small_components(P, E, min_component_size=3):
    G = nx.Graph()

    for i, p in enumerate(P):
        G.add_node(i, pos=p)

    for a, b in E:
        G.add_edge(int(a), int(b))

    keep_nodes = set()

    for comp in nx.connected_components(G):
        if len(comp) >= min_component_size:
            keep_nodes.update(comp)

    old_to_new = {}
    new_P = []

    for old_idx in sorted(keep_nodes):
        old_to_new[old_idx] = len(new_P)
        new_P.append(P[old_idx])

    new_E = []

    for a, b in E:
        a, b = int(a), int(b)

        if a in old_to_new and b in old_to_new:
            new_E.append([old_to_new[a], old_to_new[b]])

    return np.asarray(new_P), np.asarray(new_E, dtype=int)


# =========================================================
# EXPORT LOGIC
# =========================================================
def export_pajek_with_positions(P, E, output_path):
    with open(output_path, "w") as f:
        f.write(f"*Vertices {len(P)}\n")

        for i, p in enumerate(P, start=1):
            x, y, z = p
            f.write(f'{i} "{i}" {x:.6f} {y:.6f} {z:.6f}\n')

        f.write("*Edges\n")

        for a, b in E:
            f.write(f"{int(a) + 1} {int(b) + 1}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Topology-aware merge of two spiderweb PCD scans."
    )

    parser.add_argument("pcd1", help="Path to first raw PCD scan")
    parser.add_argument("pcd2", help="Path to second raw PCD scan")

    parser.add_argument(
        "--output",
        default="merged_graph.net",
        help="Path to output Pajek .net file"
    )

    parser.add_argument(
        "--pcd-output",
        default="merged_graph.pcd",
        help="Path to output merged graph vertices as PCD"
    )

    parser.add_argument(
        "--max-mid-dist",
        type=float,
        default=5.0,
        help="Maximum midpoint distance for duplicate edge matching"
    )

    parser.add_argument(
        "--min-dir-sim",
        type=float,
        default=0.90,
        help="Minimum direction similarity for duplicate edge matching"
    )

    parser.add_argument(
        "--max-endpoint-dist",
        type=float,
        default=10.0,
        help="Maximum endpoint distance for duplicate edge matching"
    )

    parser.add_argument(
        "--snap-radius",
        type=float,
        default=3.0,
        help="Radius for snapping compatible vertices"
    )

    parser.add_argument(
        "--min-component-size",
        type=int,
        default=3,
        help="Minimum connected component size to keep"
    )

    args = parser.parse_args()

    try:
        print("\n=== 1. Reconstructing graph for scan 1 ===")
        result1 = generate_graph(args.pcd1)

        points1 = np.loadtxt(result1["nodes"])
        edges1 = np.loadtxt(result1["edges"], dtype=int)

        P_graph_1, E_graph_1 = rebuild_graph_from_arrays(points1, edges1)

        print(f"Scan 1 graph: {len(P_graph_1)} vertices, {len(E_graph_1)} edges")

        print("\n=== 2. Reconstructing graph for scan 2 ===")
        result2 = generate_graph(args.pcd2)

        points2 = np.loadtxt(result2["nodes"])
        edges2 = np.loadtxt(result2["edges"], dtype=int)

        P_graph_2, E_graph_2 = rebuild_graph_from_arrays(points2, edges2)

        print(f"Scan 2 graph: {len(P_graph_2)} vertices, {len(E_graph_2)} edges")

        print("\n=== 3. Building Open3D graph point clouds for rough registration ===")
        pcd_graph_1 = o3d.geometry.PointCloud()
        pcd_graph_1.points = o3d.utility.Vector3dVector(P_graph_1)

        pcd_graph_2 = o3d.geometry.PointCloud()
        pcd_graph_2.points = o3d.utility.Vector3dVector(P_graph_2)

        print("\n=== 4. Estimating rough Z-rotation + ICP transform ===")
        rough_T = estimate_rough_transform(
            source_skel=pcd_graph_2,
            target_skel=pcd_graph_1
        )

        print("Estimated transform:")
        print(rough_T)

        print("\n=== 5. Applying transform to scan 2 graph ===")
        P_graph_2_aligned = apply_transform_to_points(P_graph_2, rough_T)

        print("\n=== 6. Topology-aware edge merge ===")
        P_merged, E_merged = merge_graphs_topology_aware(
            P1=P_graph_1,
            E1=E_graph_1,
            P2=P_graph_2_aligned,
            E2=E_graph_2,
            max_mid_dist=args.max_mid_dist,
            min_dir_sim=args.min_dir_sim,
            max_endpoint_dist=args.max_endpoint_dist,
            snap_radius=args.snap_radius
        )

        print(f"After topo merge: {len(P_merged)} vertices, {len(E_merged)} edges")

        print("\n=== 7. Removing duplicate edges ===")
        P_merged, E_merged = remove_duplicate_edges(P_merged, E_merged)

        print(f"After duplicate cleanup: {len(P_merged)} vertices, {len(E_merged)} edges")

        print("\n=== 8. Removing tiny disconnected components ===")
        P_merged, E_merged = remove_small_components(
            P_merged,
            E_merged,
            min_component_size=args.min_component_size
        )

        print(f"After component cleanup: {len(P_merged)} vertices, {len(E_merged)} edges")

        print("\n=== 9. Exporting merged PCD vertices ===")
        merged_pcd = o3d.geometry.PointCloud()
        merged_pcd.points = o3d.utility.Vector3dVector(P_merged)
        o3d.io.write_point_cloud(args.pcd_output, merged_pcd)

        print(f"Wrote merged PCD to: {args.pcd_output}")

        print("\n=== 10. Exporting merged Pajek .net graph ===")
        export_pajek_with_positions(P_merged, E_merged, args.output)

        print(f"Wrote merged graph to: {args.output}")

        print("\nDone.")

    except Exception as e:
        print(f"\nERROR: {str(e)}")
        sys.exit(1)



if __name__ == "__main__":
    main()
