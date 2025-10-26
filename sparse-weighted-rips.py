import dmpcd as dm
import dmpcd.pcd as pcd
import open3d as o3d
import numpy as np
import os

dataset = "tangle01"

"""
# if using pcd, and features.txt doesn't already exist then use below
pcd_name = "tangle001 255 2025-02-08 18-50-40 T0.35.pcd"

# load pcd
pcd = o3d.io.read_point_cloud(f"data/{dataset}/{pcd_name}")
voxel_size = 10
pcd_down = pcd.voxel_down_sample(voxel_size)
points = np.asarray(pcd_down.points)

# save to features.txt
np.savetxt(f"data/{dataset}/features.txt", points, fmt="%.6f")
"""

feature_filename = f"data/{dataset}/features.txt"
output_dir = f"results/{dataset}-pcd/"
os.makedirs(output_dir, exist_ok=True)
k = 15 # unsure
metric = 'euclidean' # could potentially affect result
epsilon = .99 # Does not affect result, slows runtime if small
persistence_threshold = .99 # Does not affect result, persistence values lie around 1

dm.pcd.build_sparse_weighted_rips_filtration(feature_filename, output_dir, k, metric, epsilon)
filtration_filename = os.path.join(output_dir, 'sparse_weighted_rips_filtration.txt')
weights_filename = os.path.join(output_dir, 'weights.txt')

dm.pcd.compute_persistence_swr(filtration_filename, output_dir)
edge_filename = os.path.join(output_dir, "edge_for_morse_only.txt")
sorted_weights_filename = os.path.join(output_dir, "sorted-weights.txt")

dm.pcd.reorder_weights(weights_filename, sorted_weights_filename)

morse_dir = output_dir
dm.pcd.compute_graph_reconstruction(sorted_weights_filename, edge_filename, persistence_threshold, morse_dir)
result_edge_filename = os.path.join(morse_dir, 'edge.txt')
sorted_feature_filename = os.path.join(output_dir, 'sorted-feature.txt')
dm.pcd.reorder_verts_by_weight(weights_filename, feature_filename, sorted_feature_filename)

# Only works for 2d outputs
# dm.visualize_results_2d(sorted_feature_filename, result_edge_filename)



# For 3d outputs, use open3d

# Load points
points = np.loadtxt(f"results/{dataset}-pcd/sorted-feature.txt")
if points.shape[1] == 2:  # pad z=0 for 2D
    points = np.hstack([points, np.zeros((points.shape[0], 1))])

# Load edges with presence marker
edges = []
with open(f"results/{dataset}-pcd/edge.txt") as f:
    for line in f:
        nums = line.strip().split()
        if len(nums) >= 3:
            u, v, marker = int(nums[0]), int(nums[1]), int(nums[2])
            edges.append([u, v])

edges = np.array(edges, dtype=np.int32)
print(f"len(edges) = {len(edges)}")

# Create Open3D objects
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.paint_uniform_color([0.1, 0.7, 0.9])

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(edges)
line_set.paint_uniform_color([0, 0, 0])

# Visualize
o3d.visualization.draw_geometries([pcd, line_set])