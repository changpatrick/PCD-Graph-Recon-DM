import numpy as np
import open3d as o3d

dataset = "tangle01"

# For 3d outputs, use open3d

# Load points
points = np.loadtxt(f"results/{dataset}-pcd/sorted-feature.txt")
if points.shape[1] == 2:  # pad z=0 for 2D
    points = np.hstack([points, np.zeros((points.shape[0], 1))])

# Load original edges with presence marker
edges = []
with open(f"results/{dataset}-pcd/edge.txt") as f:
    for line in f:
        nums = line.strip().split()
        if len(nums) >= 3:
            u, v, marker = int(nums[0]), int(nums[1]), int(nums[2])
            if marker < 4: # can do == 2 to check for different markers (-1, 1, 2)
                edges.append([u, v])

edges = np.array(edges, dtype=np.int32)
print(f"len(edges) = {len(edges)}")

# Load detour filtered edges
edges_detour_filtered = []
with open(f"results/{dataset}-pcd/edge_detour_filtered.txt") as f:
    for line in f:
        nums = line.strip().split()
        if len(nums) >= 3:
            u, v, marker = int(nums[0]), int(nums[1]), int(nums[2])
            if marker < 4:
                edges_detour_filtered.append([u, v])

edges_detour_filtered = np.array(edges_detour_filtered, dtype=np.int32)
print(f"len(edges_detour_filtered) = {len(edges_detour_filtered)}")


# Create Open3D objects
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.paint_uniform_color([0.1, 0.7, 0.9])

# Original edges
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(edges)
line_set.paint_uniform_color([0, 0, 0])

# Filtered edges superimposed
line_detour = o3d.geometry.LineSet()
line_detour.points = o3d.utility.Vector3dVector(points)
line_detour.lines = o3d.utility.Vector2iVector(edges_detour_filtered)
line_detour.paint_uniform_color([1, 0.2, 0.2])  # red lines

# --- Visualize ---
o3d.visualization.draw_geometries(
    [pcd, line_set, line_detour],
    window_name="Edges + Detour Filtered Overlay",
    width=1200,
    height=800,
    point_show_normal=False,
)