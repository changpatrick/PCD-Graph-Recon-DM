import numpy as np
import open3d as o3d
import glob
import os

# -----------------------------
# User settings
# -----------------------------

dataset = "tangle01"
path = f"data/{dataset}"
input_file = glob.glob(os.path.join(path, "*.pcd"))[0]
num_slices = 5                # Number of equal-height layers
voxel_sizes = [0.02, 0.05, 0.08, 0.12, 0.15]   # One voxel size per slice

assert len(voxel_sizes) == num_slices, "voxel_sizes list must match num_slices"

# -----------------------------
# Load point cloud
# -----------------------------
pcd = o3d.io.read_point_cloud(input_file)
points = np.asarray(pcd.points)

# -----------------------------
# Compute vertical slicing bounds
# -----------------------------
z_min = points[:, 2].min()
z_max = points[:, 2].max()
height = z_max - z_min
slice_height = height / num_slices

# -----------------------------
# Slice, downsample, merge
# -----------------------------
downsampled_slices = []

for i in range(num_slices):
    # Slice bounds
    z_low = z_min + i * slice_height
    z_high = z_min + (i + 1) * slice_height

    # Select points inside slice
    mask = (points[:, 2] >= z_low) & (points[:, 2] < z_high)
    slice_points = points[mask]

    if len(slice_points) == 0:
        continue

    # Create slice point cloud
    slice_pcd = o3d.geometry.PointCloud()
    slice_pcd.points = o3d.utility.Vector3dVector(slice_points)

    # Downsample slice using its specific voxel size
    print(f"Slice {i+1}: {len(slice_points)} pts → voxel {voxel_sizes[i]}")
    down = slice_pcd.voxel_down_sample(voxel_sizes[i])
    downsampled_slices.append(down)

# -----------------------------
# Merge all downsampled slices
# -----------------------------
merged_pcd = o3d.geometry.PointCloud()
for d in downsampled_slices:
    merged_pcd += d

# -----------------------------
# Save + visualize result
# -----------------------------
output_file = f"data/{dataset}/cloud_multires.pcd"
o3d.io.write_point_cloud(output_file, merged_pcd)

print("Saved merged multiresolution point cloud to:", output_file)

# Vis
merged_pcd.colors = o3d.utility.Vector3dVector(np.zeros((len(merged_pcd.points), 3)))
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(merged_pcd)
render_option = vis.get_render_option()
render_option.point_size = 3
vis.run()
vis.destroy_window()
