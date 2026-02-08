import networkx as nx
import numpy as np
from collections import Counter, defaultdict
import open3d as o3d


def num_components_from_edges(edges, n_nodes=None):
    edges = np.asarray(edges)
    uv = edges[:, :2].astype(int) if edges.size else np.empty((0,2), dtype=int)

    G = nx.Graph()
    if n_nodes is not None:
        G.add_nodes_from(range(int(n_nodes)))
    G.add_edges_from(map(tuple, uv))

    return nx.number_connected_components(G)


def deduplicate_edges(edges_full):
    """
    edges_full: (M,2) or (M,3) int array
    Returns:
      deduped edges with same shape as input
    """
    E = np.asarray(edges_full, dtype=int)[:, :2]

    if E.ndim == 1:
        E = E.reshape(1, -1)

    if E.shape[1] < 2:
        raise ValueError("edges must have at least 2 columns")

    # Canonicalize (u,v)
    uv = E[:, :2]
    uv_sorted = np.sort(uv, axis=1)

    # Use uv as key; keep first occurrence
    _, idx = np.unique(uv_sorted, axis=0, return_index=True)

    # Preserve original row order (optional but nice)
    idx = np.sort(idx)

    return E[idx]

def count_components(P, E):
    n_nodes = P.shape[0]

    uv = E[:, :2]

    G = nx.Graph()
    G.add_edges_from(map(tuple, uv.tolist()))  # nodes created from endpoints
    return nx.number_connected_components(G)


def num_degree_one_vertices(E):

    if E.ndim == 1 and E.size > 0:
        E = E.reshape(1, -1)

    if E.size == 0:
        return 0

    uv = E[:, :2]

    deg = Counter()
    for u, v in uv:
        deg[int(u)] += 1
        deg[int(v)] += 1

    return sum(1 for d in deg.values() if d == 1)




def visualize_degree1_vertices(points, edges_full_or_uv,
                               base_point_color=(0.1, 0.7, 0.9),
                               line_color=(0.0, 0.0, 0.0),
                               degree1_color=(1.0, 0.0, 0.0),
                               sphere_radius=0.1):

    points = np.asarray(points, dtype=float)
    edges = np.asarray(edges_full_or_uv)

    if edges.ndim == 1:
        edges = edges.reshape(1, -1)

    uv = edges[:, :2].astype(int)

    # ---- degree count (only vertices in edges) ----
    deg = Counter()
    for u, v in uv:
        deg[u] += 1
        deg[v] += 1

    used_vertices = np.array(sorted(deg.keys()), dtype=int)
    degree1_vertices = np.array([v for v, d in deg.items() if d == 1], dtype=int)

    print(f"Vertices shown (deg ≥ 1): {len(used_vertices)}")
    print(f"Degree-1 vertices: {len(degree1_vertices)}")

    # ---- base point cloud (ONLY deg ≥ 1 vertices) ----
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[used_vertices])
    pcd.paint_uniform_color(base_point_color)

    # ---- line set (unchanged) ----
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(uv)
    ls.paint_uniform_color(line_color)

    geoms = [pcd, ls]

    # ---- spheres at degree-1 vertices ----
    for idx in degree1_vertices:
        s = o3d.geometry.TriangleMesh.create_sphere(radius=float(sphere_radius))
        s.translate(points[int(idx)])
        s.paint_uniform_color(degree1_color)
        geoms.append(s)

    o3d.visualization.draw_geometries(geoms)


#remove all vertices w degree 1, then try growing again from all new vertices w degree 1
def prune_degree1_once(edges_uv):
    """
    edges_uv: (M,2) int array
    Returns:
      pruned_edges_uv: edges after removing all degree-1 vertices
      removed_vertices: set of vertices removed in this pass
    """
    
    edges_uv = np.asarray(edges_uv[:, :2], dtype=int).reshape(-1, 2)
    if edges_uv.size == 0:
        return edges_uv, set()

    deg = Counter()
    for u, v in edges_uv:
        deg[int(u)] += 1
        deg[int(v)] += 1

    leaves = {v for v, d in deg.items() if d == 1}
    if not leaves:
        return edges_uv, set()

    mask = np.array([(u not in leaves) and (v not in leaves) for u, v in edges_uv], dtype=bool)
    return edges_uv[mask], leaves



def _dedup_undirected_edges_uv(edges_uv: np.ndarray) -> np.ndarray:
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
    if edges_uv.size == 0:
        return edges_uv
    uv = np.sort(edges_uv, axis=1)
    _, idx = np.unique(uv, axis=0, return_index=True)
    return edges_uv[np.sort(idx)]
def _max_edge_length(points: np.ndarray, edges_uv: np.ndarray) -> float:
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
    if edges_uv.size == 0:
        return 0.0
    d = points[edges_uv[:, 1]] - points[edges_uv[:, 0]]
    return float(np.max(np.linalg.norm(d, axis=1)))

def _degree1_vertices(edges_uv: np.ndarray):
    deg = Counter()
    for u, v in edges_uv:
        deg[int(u)] += 1
        deg[int(v)] += 1
    return [v for v, d in deg.items() if d == 1], deg

def _build_adj(edges_uv: np.ndarray):
    adj = defaultdict(list)
    for u, v in edges_uv:
        u = int(u); v = int(v)
        adj[u].append(v)
        adj[v].append(u)
    return adj


def _closest_points_on_segments(p1, q1, p2, q2, eps=1e-12):
    """
    Returns (c1, c2, dist, s, t) where:
      c1 is closest point on segment p1->q1
      c2 is closest point on segment p2->q2
      s,t in [0,1] are the segment parameters
    Standard segment-segment closest approach.
    """
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))

    if a <= eps and e <= eps:
        c1 = p1
        c2 = p2
        return c1, c2, float(np.linalg.norm(c1 - c2)), 0.0, 0.0

    if a <= eps:
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0) if e > eps else 0.0
    else:
        c = float(np.dot(d1, r))
        if e <= eps:
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            if denom != 0.0:
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
            else:
                s = 0.0
            t = (b * s + f) / e

            if t < 0.0:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = np.clip((b - c) / a, 0.0, 1.0)

    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    dist = float(np.linalg.norm(c1 - c2))
    return c1, c2, dist, s, t


def grow_rays_and_connect(points, edges_full_or_uv, tol=1.0, connect_triangle=False):
    """
    points: (N,3)
    edges_full_or_uv: (M,2) or (M,3). If (M,3) uses first two cols.
    tol: max distance between rays to treat as "intersection" (units = your point units)
    connect_triangle: if True, also connect the two endpoints together (forms triangle)

    Returns:
      new_points: (N+K,3)
      new_edges_full: (M+2*I,3) with marker=9 for new edges (or (M+2*I,2) if input had no marker)
    """
    P = np.asarray(points, dtype=float)
    E = np.asarray(edges_full_or_uv)

    has_marker = (E.ndim == 2 and E.shape[1] >= 3)
    uv = E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int)

    uv = _dedup_undirected_edges_uv(uv)

    # degree-1 endpoints + adjacency
    deg1, _deg = _degree1_vertices(uv)
    adj = _build_adj(uv)

    if len(deg1) == 0:
        return P, E

    # ray length = max edge length in current graph
    L = _max_edge_length(P, uv)
    if L <= 0:
        return P, E

    # build ray segments: (start, end, endpoint_index)
    rays = []
    for u in deg1:
        nbrs = adj[int(u)]
        if len(nbrs) != 1:
            continue
        v = int(nbrs[0])
        dir_vec = P[u] - P[v]  # shoot outward from neighbor -> endpoint direction
        n = np.linalg.norm(dir_vec)
        if n == 0:
            continue
        dir_vec = dir_vec / n
        p1 = P[u]
        q1 = P[u] + dir_vec * L
        rays.append((p1, q1, int(u)))

    # pairwise intersections (closest approach within tol)
    new_pts = []
    new_edges_uv = []

    for i in range(len(rays)):
        p1, q1, u1 = rays[i]
        for j in range(i + 1, len(rays)):
            p2, q2, u2 = rays[j]

            c1, c2, dist, s, t = _closest_points_on_segments(p1, q1, p2, q2)
            if dist <= tol:
                x = 0.5 * (c1 + c2)  # "intersection" point
                new_index = P.shape[0] + len(new_pts)
                new_pts.append(x)

                # connect all three points: endpoints + intersection
                new_edges_uv.append([u1, new_index])
                new_edges_uv.append([u2, new_index])
                if connect_triangle:
                    new_edges_uv.append([u1, u2])

    if not new_pts:
        return P, E

    # append points
    P2 = np.vstack([P, np.asarray(new_pts, dtype=float)])

    # merge edges
    uv2 = np.vstack([uv, np.asarray(new_edges_uv, dtype=int)])
    uv2 = _dedup_undirected_edges_uv(uv2)

    if has_marker:
        # keep original markers, mark new edges as 9
        orig_full = E[:, :3].astype(int)
        orig_uv_sorted = np.sort(orig_full[:, :2], axis=1)
        orig_map = {tuple(orig_uv_sorted[k]): int(orig_full[k, 2]) for k in range(orig_full.shape[0])}

        full = []
        for u, v in uv2:
            key = tuple(sorted((int(u), int(v))))
            m = orig_map.get(key, 9)
            full.append([int(u), int(v), int(m)])
        E2 = np.asarray(full, dtype=int)
        return P2, E2

    return P2, uv2


if __name__ == "__main__":
    edges_path = "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/edge_detour_filtered.txt"
    points_path = "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/sorted-feature.txt"

    
    P = np.loadtxt(points_path)

    E = np.loadtxt(edges_path, dtype=int)
    if E.ndim == 1 and E.size > 0:
        E = E.reshape(1, -1) 

    E = deduplicate_edges(E)

    print(f"number of connected components: {count_components(P,E)}")

    
    print(f"number of deg 1 vert: {num_degree_one_vertices(E)}")


    pruned, leaves = prune_degree1_once(E)

    # visualize_degree1_vertices(P, pruned, sphere_radius=5.0)


        
    points_grown, edges_grown_full = grow_rays_and_connect(
        P,
        pruned,
        tol=5,               # pick based on your units (try 1, 5, 10, ...)
        connect_triangle=False  # set True if you want the 3rd edge between endpoints
    )



    visualize_degree1_vertices(points_grown, edges_grown_full, sphere_radius=5.0)




    # TO DO: make it so each deg 1 vertex shoots out a beam to latch onto closest point.