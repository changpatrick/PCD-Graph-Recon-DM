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

#Shoots a short ray segment outwards from new dangling endpoints. Whenever two such outward segments come close, you “snap” them together by creating a new vertex at the near-intersection location and connecting both endpoints to it.  
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




# Computes for candidate points the projection distance along the ray direction and perpendicular distance from point to ray line
#necessary for following function
def _point_to_ray_metrics(Pu: np.ndarray, d: np.ndarray, X: np.ndarray):
    """
    Pu: (3,)
    d : (3,) unit direction
    X : (K,3) candidate points
    Returns:
      t: (K,) signed distance along ray (projection)
      perp: (K,) perpendicular distance to ray line
    """
    w = X - Pu  # (K,3)
    t = w @ d   # (K,)
    perp_vec = w - np.outer(t, d)  # (K,3)
    perp = np.linalg.norm(perp_vec, axis=1)
    return t, perp





#make it so each deg 1 vertex shoots out a beam to latch onto closest point.

#shoots a beam in the direction the line segement the leaf is on already, but it si a fat beam.
#Then, of the other points caught in the beam, we grab them as our candidate set.
def _ray_beam_candidates(Pu, d_hat, P_all, beam_radius, max_length, exclude_idx=()):
    """
    Pu: (3,) ray origin
    d_hat: (3,) unit direction
    P_all: (N,3) all points
    Returns:
      cand_idx: indices of points inside the fat beam (tube) in front of Pu
      t:        forward distances along ray for cand_idx (used for "closest along ray")
      perp:     perpendicular distances to ray for cand_idx
    """
    # Vector from origin to all points
    W = P_all - Pu                    # (N,3)
    t = W @ d_hat                     # (N,) projection along direction

    # Keep only points in front and within length
    mask = (t > 0) & (t <= max_length)

    if np.any(mask):
        # Perpendicular distance to ray line
        Wm = W[mask]
        tm = t[mask]
        perp_vec = Wm - tm[:, None] * d_hat[None, :]
        perp = np.linalg.norm(perp_vec, axis=1)

        # Inside beam radius
        mask2 = perp <= beam_radius
        idx_mask = np.flatnonzero(mask)
        cand_idx = idx_mask[mask2]
        t_keep = tm[mask2]
        perp_keep = perp[mask2]
    else:
        cand_idx = np.array([], dtype=int)
        t_keep = np.array([], dtype=float)
        perp_keep = np.array([], dtype=float)

    if exclude_idx:
        exclude_idx = set(int(x) for x in exclude_idx)
        if cand_idx.size > 0:
            keep = np.array([int(i) not in exclude_idx for i in cand_idx], dtype=bool)
            cand_idx = cand_idx[keep]
            t_keep = t_keep[keep]
            perp_keep = perp_keep[keep]

    return cand_idx, t_keep, perp_keep



#Finds nearest candidate to connect to basically
def beam_latch_from_degree1(
    points,
    edges_full_or_uv,
    beam_radius=5.0,
    max_length=None,          # if None: uses max edge length (you already have _max_edge_length)
    pick="forward",           # "forward" (min t) or "euclid" (min ||P[w]-P[u]||)
    perp_tiebreak=True,       # break ties by smaller perp distance
    marker_new=9
):
    """
    For each degree-1 vertex u:
      - direction = (P[u] - P[v]) where v is u's only neighbor
      - find all vertices within a fat beam (tube) along that direction
      - pick the closest candidate among those
      - add edge (u, candidate)

    Returns:
      P2 (same as P), E2 with added edges (marker preserved if present)
    """
    P = np.asarray(points, dtype=float)
    E = np.asarray(edges_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)

    uv = E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int)
    uv = _dedup_undirected_edges_uv(uv)  # you already have this

    deg1, deg = _degree1_vertices(uv)    # you already have this
    if len(deg1) == 0:
        return P, E

    adj = _build_adj(uv)                 # you already have this

    # Ray length scale
    L = _max_edge_length(P, uv) if max_length is None else float(max_length)
    if L <= 0:
        return P, E

    # Existing edges set to avoid duplicates
    existing = set(map(tuple, np.sort(uv, axis=1)))

    new_edges = []

    for u in deg1:
        u = int(u)
        nbrs = adj.get(u, [])
        if len(nbrs) != 1:
            continue
        v = int(nbrs[0])

        d = P[u] - P[v]      # outward continuation of the leaf's segment
        n = np.linalg.norm(d)
        if n == 0:
            continue
        d_hat = d / n

        # candidates inside fat beam; exclude self + neighbor
        cand_idx, t, perp = _ray_beam_candidates(
            Pu=P[u],
            d_hat=d_hat,
            P_all=P,
            beam_radius=float(beam_radius),
            max_length=float(L),
            exclude_idx=(u, v)
        )

        if cand_idx.size == 0:
            continue

        # choose best candidate
        if pick == "forward":
            # minimize forward distance along ray (closest hit in front)
            order = np.argsort(t)
            if perp_tiebreak:
                # stable-ish: sort by t first, then perp
                # (numpy lexsort uses last key as primary)
                order = np.lexsort((perp, t))
            w = int(cand_idx[order[0]])
        elif pick == "euclid":
            dists = np.linalg.norm(P[cand_idx] - P[u], axis=1)
            order = np.argsort(dists)
            if perp_tiebreak:
                # tie-break by perp as secondary
                order = np.lexsort((perp, dists))
            w = int(cand_idx[order[0]])
        else:
            raise ValueError("pick must be 'forward' or 'euclid'")

        key = tuple(sorted((u, w)))
        if key in existing:
            continue

        new_edges.append([u, w])
        existing.add(key)

    if not new_edges:
        return P, E

    uv2 = np.vstack([uv, np.asarray(new_edges, dtype=int)])
    uv2 = _dedup_undirected_edges_uv(uv2)

    # Preserve markers if present, mark new ones
    if has_marker:
        orig_full = E[:, :3].astype(int)
        orig_map = {tuple(sorted(map(int, orig_full[k, :2]))): int(orig_full[k, 2])
                    for k in range(orig_full.shape[0])}
        full = []
        for a, b in uv2:
            key = tuple(sorted((int(a), int(b))))
            m = orig_map.get(key, int(marker_new))
            full.append([int(a), int(b), int(m)])
        return P, np.asarray(full, dtype=int)

    return P, uv2


def _branch_vertices_from_leaf(u: int, adj, deg):
    u = int(u)
    if deg.get(u, 0) != 1:
        return {u}

    nbrs = adj.get(u, [])
    if len(nbrs) != 1:
        return {u}

    branch = {u}
    prev = u
    cur = int(nbrs[0])
    branch.add(cur)

    while deg.get(cur, 0) == 2:
        nbs = adj.get(cur, [])
        if len(nbs) != 2:
            break
        nxt = int(nbs[0]) if int(nbs[1]) == prev else int(nbs[1])
        if nxt == prev:
            break
        prev, cur = cur, nxt
        if cur in branch:
            break
        branch.add(cur)

    return branch



def connect_interior_leaves_to_nearest_k_no_same_branch_xy(
    points,
    edges_full_or_uv,
    k=2,
    interior_quantile=0.10,   # middle 80% => 10%..90%
    x_axis=0,                 # X column
    y_axis=1,                 # Y column
    marker_new=9,
    exclude_existing_neighbors=True,
    exclude_same_branch=True,
):
    """
    For remaining degree-1 vertices (leaves) whose (X,Y) coordinates lie in the
    middle 80% quantile range (per-axis), connect each to its nearest k points,
    excluding points on the same branch.
    """
    P = np.asarray(points, dtype=float)
    E = np.asarray(edges_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)

    uv = E[:, :2].astype(int) if E.size else np.empty((0, 2), dtype=int)
    uv = _dedup_undirected_edges_uv(uv)

    leaves, deg = _degree1_vertices(uv)
    if len(leaves) == 0:
        return P, E

    adj = _build_adj(uv)

    # --- XY-only interior bounds ---
    q = float(interior_quantile)
    xs = P[:, int(x_axis)]
    ys = P[:, int(y_axis)]

    x_lo, x_hi = np.quantile(xs, [q, 1.0 - q])
    y_lo, y_hi = np.quantile(ys, [q, 1.0 - q])

    interior_leaves = [
        int(u) for u in leaves
        if (x_lo <= P[int(u), x_axis] <= x_hi) and
           (y_lo <= P[int(u), y_axis] <= y_hi)
    ]

    if len(interior_leaves) == 0:
        return P, E

    existing = set(map(tuple, np.sort(uv, axis=1)))
    new_edges = []

    for u in interior_leaves:
        forbid = {int(u)}

        if exclude_existing_neighbors:
            forbid.update(int(v) for v in adj.get(u, []))

        if exclude_same_branch:
            forbid.update(_branch_vertices_from_leaf(u, adj, deg))

        d = np.linalg.norm(P - P[u], axis=1)
        d[list(forbid)] = np.inf

        nn = np.argsort(d)[:k]
        for w in nn:
            if not np.isfinite(d[w]):
                continue
            key = tuple(sorted((int(u), int(w))))
            if key in existing:
                continue
            new_edges.append([int(u), int(w)])
            existing.add(key)

    if not new_edges:
        return P, E

    uv2 = np.vstack([uv, np.asarray(new_edges, dtype=int)])
    uv2 = _dedup_undirected_edges_uv(uv2)

    if has_marker:
        orig_full = E[:, :3].astype(int)
        orig_map = {
            tuple(sorted(map(int, orig_full[i, :2]))): int(orig_full[i, 2])
            for i in range(orig_full.shape[0])
        }
        full = []
        for a, b in uv2:
            key = tuple(sorted((int(a), int(b))))
            m = orig_map.get(key, int(marker_new))
            full.append([int(a), int(b), int(m)])
        return P, np.asarray(full, dtype=int)

    return P, uv2


def remove_isolated_points(P, E_full_or_uv):
    P = np.asarray(P, float)
    E = np.asarray(E_full_or_uv)
    has_marker = (E.ndim == 2 and E.shape[1] >= 3)

    if E.size == 0:
        return P[:0], E  # no edges => everything is isolated

    uv = E[:, :2].astype(int).reshape(-1, 2)

    used = np.zeros(P.shape[0], dtype=bool)
    used[uv[:, 0]] = True
    used[uv[:, 1]] = True

    old_idx = np.flatnonzero(used)
    old_to_new = -np.ones(P.shape[0], dtype=int)
    old_to_new[old_idx] = np.arange(old_idx.size)

    P2 = P[old_idx]

    uv2 = old_to_new[uv]  # remap endpoints

    if has_marker:
        m = E[:, 2].astype(int).reshape(-1, 1)
        E2 = np.hstack([uv2, m])
        return P2, E2
    else:
        return P2, uv2











def export_pajek_net(filepath, points, edges_full_or_uv, use_markers_as_weights=False):
    """
    Writes a Pajek .net file.

    points: (N,3) float array (we export x,y; keep z in a comment)
    edges_full_or_uv: (M,2) or (M,3) int array; first two cols are (u,v), optional marker/weight.

    Pajek uses 1-based vertex indices.
    """
    P = np.asarray(points, dtype=float)
    E = np.asarray(edges_full_or_uv)

    if E.size == 0:
        uv = np.empty((0, 2), dtype=int)
        w = None
    else:
        uv = E[:, :2].astype(int)
        w = E[:, 2].astype(float) if (E.ndim == 2 and E.shape[1] >= 3) else None

    N = P.shape[0]

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(f"*Vertices {N}\n")
        # Pajek vertex line: id "label" x y
        # We'll use id as label; include z as a trailing comment.
        for i in range(N):
            x, y, z = P[i]
            vid = i + 1  # 1-based
            f.write(f'{vid} "{vid}" {x:.8f} {y:.8f}  % z={z:.8f}\n')

        f.write("*Edges\n")  # undirected edges
        for k in range(uv.shape[0]):
            a = int(uv[k, 0]) + 1
            b = int(uv[k, 1]) + 1
            if use_markers_as_weights and w is not None:
                f.write(f"{a} {b} {w[k]:.6f}\n")
            else:
                f.write(f"{a} {b}\n")




def export_pajek_net_format(filepath, points, edges_full_or_uv, use_markers_as_weights=False):
    """
    Writes a Pajek .net file in a simple strict format.

    Format:
    *Vertices N
    1 "1"
    2 "2"
    ...
    *Edgeslist
    u v
    u v
    ...

    If use_markers_as_weights=True and a 3rd column exists, then:
    *Edges
    u v w
    ...
    """

    P = np.asarray(points, dtype=float)
    E = np.asarray(edges_full_or_uv)

    N = P.shape[0]

    if E.size == 0:
        uv = np.empty((0, 2), dtype=int)
        w = None
    else:
        if E.ndim != 2 or E.shape[1] < 2:
            raise ValueError("edges_full_or_uv must have shape (M,2) or (M,3)")
        uv = E[:, :2].astype(int)
        w = E[:, 2].astype(float) if E.shape[1] >= 3 else None

    with open(filepath, "w", encoding="utf-8", newline="\n") as f:
        # Vertices
        f.write(f"*Vertices {N}\n")
        for i in range(N):
            vid = i + 1   # Pajek is 1-based
            f.write(f'{vid} "{vid}"\n')

        # Edges
        if use_markers_as_weights and w is not None:
            f.write("*Edges\n")
            for k in range(uv.shape[0]):
                a = int(uv[k, 0]) + 1
                b = int(uv[k, 1]) + 1
                f.write(f"{a} {b} {w[k]:.6f}\n")
        else:
            f.write("*Edgeslist\n")
            for k in range(uv.shape[0]):
                a = int(uv[k, 0]) + 1
                b = int(uv[k, 1]) + 1
                f.write(f"{a} {b}\n")











if __name__ == "__main__":
    edges_path = "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/edge_detour_filtered1.txt"
    points_path = "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/sorted-feature1.txt"

    
    P = np.loadtxt(points_path)

    E = np.loadtxt(edges_path, dtype=int)
    if E.ndim == 1 and E.size > 0:
        E = E.reshape(1, -1) 

    E = deduplicate_edges(E)

    print(f"number of connected components: {count_components(P,E)}")

    
    print(f"number of deg 1 vert: {num_degree_one_vertices(E)}")


    pruned, leaves = prune_degree1_once(E)

    # visualize_degree1_vertices(P, pruned, sphere_radius=5.0)


        
    points_rays, edges_rays = grow_rays_and_connect(
    P,
    pruned,
    tol=5.0,                 # tighter than beam radius
    connect_triangle=False
    )

    edges_pruned2, _ = prune_degree1_once(edges_rays)


    points_final, edges_final = beam_latch_from_degree1(
    points_rays,
    edges_pruned2,
    beam_radius=60.0,   # can be slightly larger than ray tol
    pick="forward"
    )

    P_clean, E_clean = remove_isolated_points(points_final, edges_final)


#     edges_final, _ = prune_degree1_once(edges_final)

#     P_final, E_final = connect_remaining_leaves_to_k_nearest(
#     points_final,   # from beam stage
#     edges_final,
#     k=2,
#     max_dist=1000000000,          # or set to something like 10 or 20 to prevent crazy long edges
#     exclude_neighbor=True,
#     marker_new=9
# )

#     print("done")


    P2, E2 =  connect_interior_leaves_to_nearest_k_no_same_branch_xy(
    P_clean,
    E_clean,
    k=2,
    interior_quantile=0.10,  # middle 80%
    x_axis=0,                # X
    y_axis=1                 # Y
)
    
    export_pajek_net_format(
    "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/reconstructed.net",
    points_final,
    edges_final,
    use_markers_as_weights=True  # optional
)
    
    visualize_degree1_vertices(
    P2,
    E2,
    sphere_radius=5.0
    )



