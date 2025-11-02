import numpy as np
import open3d as o3d
import networkx as nx
from MomentumConnect import MomentumConnect

dataset = "tangle01"

# For 3d outputs, use open3d

# Load points
points = np.loadtxt(f"results/{dataset}-pcd/sorted-feature.txt")
if points.shape[1] == 2:  # pad z=0 for 2D
    points = np.hstack([points, np.zeros((points.shape[0], 1))])

# print(points.shape)
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

    return np.array(keep, dtype=int),np.array(delete, dtype=int) 

# --- Load edges (u v marker) ---
edges_list = []
with open(f"results/{dataset}-pcd/edge.txt") as f:
    for line in f:
        s = line.strip().split()
        if len(s) >= 3:
            u, v, m = int(s[0]), int(s[1]), int(s[2])
            edges_list.append([u, v, m])

# Convert ONCE to numpy
E = np.array(edges_list, dtype=int)

# Run detour filter on marker==2 edges
good2,bad2 = filter_marker2_by_detour(points, E)

# Build final edges: base (-1,1) + vetted 2s
final_edges = np.vstack([
    E[np.isin(E[:, 2], (-1, 1)), :2],
    good2
])

print(E.shape)
print(final_edges.shape)

edges = final_edges[:, :2]

badEdges = bad2

addedBack = MomentumConnect(edges, badEdges,points, 1/2, 30)

final_edges = np.vstack([
    final_edges, addedBack
])

# edges = np.array(edges, dtype=np.int32)
# print(f"len(edges) = {len(edges)}")

# Create Open3D objects
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.paint_uniform_color([0.1, 0.7, 0.9])

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(final_edges)
line_set.paint_uniform_color([0, 0, 0])

# --- Save filtered edges ---
output_path = f"results/{dataset}-pcd/edge_detour_filtered.txt"

# Reattach marker type for clarity: base edges get original markers, good2 get marker 2
base_mask = np.isin(E[:, 2], (-1, 1))
base_edges = E[base_mask, :]
good2_full = np.hstack([good2, 2 * np.ones((good2.shape[0], 1), dtype=int)])

final_edges_full = np.vstack([base_edges, good2_full])

# Save to file
np.savetxt(output_path, final_edges_full, fmt="%d")

print(f"Saved filtered edges to {output_path}")


# Visualize
o3d.visualization.draw_geometries([pcd, line_set])