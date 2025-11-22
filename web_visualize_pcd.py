import pandas as pd
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import glob
import os

# Change these values
dataset = "tangle01"
voxel_size = 1

# Create path to pcd, read in pcd
path = f"data/{dataset}"
pcd_path = glob.glob(os.path.join(path, "*.pcd"))
pcd = o3d.io.read_point_cloud(pcd_path[0])

# Potential downsampling
pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

# Vis
pcd.colors = o3d.utility.Vector3dVector(np.zeros((len(pcd.points), 3)))
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
render_option = vis.get_render_option()
render_option.point_size = 3
vis.run()
vis.destroy_window()