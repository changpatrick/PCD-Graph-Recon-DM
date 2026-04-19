# number of disconnected components
# num of vertices w/ degree 1

import networkx as nx
import numpy as np
from collections import Counter

from MomentumConnect import MomentumConnect

edges_path = "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/edge_detour_filtered.txt"
points_path = "C:/Users/samue/Downloads/Research/Spider/Current/DisconnectedComp/sorted-feature.txt"



P = np.loadtxt(points_path)

E = np.loadtxt(edges_path, dtype=int)
if E.ndim == 1 and E.size > 0:
    E = E.reshape(1, -1) 


def num_components_from_edges(edges, n_nodes=None):
    edges = np.asarray(edges)
    uv = edges[:, :2].astype(int) if edges.size else np.empty((0,2), dtype=int)

    G = nx.Graph()
    if n_nodes is not None:
        G.add_nodes_from(range(int(n_nodes)))
    G.add_edges_from(map(tuple, uv))

    return nx.number_connected_components(G)



def count_components(P, E):
    n_nodes = P.shape[0]

    uv = E[:, :2]

    G = nx.Graph()
    G.add_edges_from(map(tuple, uv.tolist()))  # nodes created from endpoints
    return nx.number_connected_components(G)


print(f"number of connected components: {count_components(P,E)}")


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


print(f"number of deg 1 vert: {num_degree_one_vertices(E)}")



import numpy as np
import open3d as o3d
from collections import Counter
import numpy as np
import open3d as o3d
from collections import Counter

def visualize_degree1_vertices(points, edges_full_or_uv,
                               base_point_color=(0.1, 0.7, 0.9),
                               line_color=(0.0, 0.0, 0.0),
                               degree1_color=(1.0, 0.0, 0.0),
                               sphere_radius=0.1):
    """
    points: (N,3) float
    edges_full_or_uv: (M,2) or (M,3) int
    sphere_radius: if None, auto-set from point cloud scale
    """

    points = np.asarray(points, dtype=float)
    edges = np.asarray(edges_full_or_uv)

    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be (N,3); got {points.shape}")

    if edges.size == 0:
        raise ValueError("edges is empty")

    if edges.ndim == 1:
        edges = edges.reshape(1, -1)

    if edges.shape[1] < 2:
        raise ValueError(f"edges must have at least 2 cols; got {edges.shape}")

    uv = edges[:, :2].astype(int)

    # bounds check (common reason for 'not work' / crashes)
    n = points.shape[0]
    if uv.min() < 0 or uv.max() >= n:
        raise ValueError(f"Edge index out of bounds: min={uv.min()} max={uv.max()} but N={n}")

    # degree count (no isolated points)
    deg = Counter()
    for u, v in uv:
        deg[int(u)] += 1
        deg[int(v)] += 1

    degree1_vertices = np.array([i for i, d in deg.items() if d == 1], dtype=int)
    print(f"Degree-1 vertices: {len(degree1_vertices)}")

    # auto sphere radius from bounding box diagonal
    if sphere_radius is None:
        bbox = points.max(axis=0) - points.min(axis=0)
        diag = float(np.linalg.norm(bbox))
        sphere_radius = diag * 0.003  # tweak: 0.001–0.01 depending on scale

    # base point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color(base_point_color)

    # lines
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(uv)
    ls.paint_uniform_color(line_color)

    geoms = [pcd, ls]

    # spheres at degree-1 vertices
    for idx in degree1_vertices:
        s = o3d.geometry.TriangleMesh.create_sphere(radius=float(sphere_radius))
        s.translate(points[int(idx)])
        s.paint_uniform_color(degree1_color)
        geoms.append(s)

    o3d.visualization.draw_geometries(geoms)


# visualize_degree1_vertices(P, E, sphere_radius=5.0)



#remove all vertices w degree 1, then try growing again from all new vertices w degree 1
def prune_degree1_once(edges_uv):
    """
    edges_uv: (M,2) int array
    Returns:
      pruned_edges_uv: edges after removing all degree-1 vertices
      removed_vertices: set of vertices removed in this pass
    """
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
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


def prune_degree1_iter(edges_uv, max_rounds=100000):
    """
    Repeatedly prune degree-1 vertices until none remain (or max_rounds).
    Returns:
      core_edges_uv
      layers: list of sets of removed vertices per round (outer -> inner)
    """
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
    layers = []
    for _ in range(max_rounds):
        edges_uv, removed = prune_degree1_once(edges_uv)
        if not removed:
            break
        layers.append(removed)
        if edges_uv.size == 0:
            break
    return edges_uv, layers

def endpoints(edges_uv):
    """Return set of degree-1 vertices (considering only vertices in edges)."""
    deg = Counter()
    for u, v in edges_uv:
        deg[int(u)] += 1
        deg[int(v)] += 1
    return {v for v, d in deg.items() if d == 1}


def grow_from_endpoints(edges_uv, badedges_uv, points, KeepTau, keepDist=30, max_iters=50):
    """
    edges_uv: (M,2) current kept edges
    badedges_uv: (B,2) candidate edges to potentially add back
    Returns:
      edges_uv_new, remaining_badedges_uv, added_all
    """
    edges_uv = np.asarray(edges_uv, dtype=int).reshape(-1, 2)
    bad = np.asarray(badedges_uv, dtype=int).reshape(-1, 2)

    added_all = []

    for _ in range(max_iters):
        ep = endpoints(edges_uv)
        if not ep or bad.size == 0:
            break

        # Only try candidates that touch a current endpoint
        touch_mask = np.array([(u in ep) or (v in ep) for u, v in bad], dtype=bool)
        cand = bad[touch_mask]
        rest = bad[~touch_mask]

        if cand.size == 0:
            break

        added = MomentumConnect(edges_uv, cand, points, KeepTau, keepDist)
        added = np.asarray(added, dtype=int).reshape(-1, 2)
        if added.size == 0:
            break

        # Add new edges
        edges_uv = np.vstack([edges_uv, added])
        added_all.append(added)

        # Remove the added edges from candidate pool (simple set removal)
        added_set = {tuple(e) if e[0] <= e[1] else (e[1], e[0]) for e in added}
        def norm(u, v): return (u, v) if u <= v else (v, u)
        bad = np.array([e for e in np.vstack([rest, cand])
                        if norm(int(e[0]), int(e[1])) not in added_set], dtype=int).reshape(-1, 2)

    added_all = np.vstack(added_all) if added_all else np.empty((0, 2), dtype=int)
    return edges_uv, bad, added_all


core_edges, layers = prune_degree1_iter(E)
print("After pruning: edges =", len(core_edges), "rounds =", len(layers))
