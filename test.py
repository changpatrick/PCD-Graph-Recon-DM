import open3d as o3d
import sys
import os

from pcd_graph_recon.api import generate_graph

pcd_files = [f for f in os.listdir("data/tangle01") if f.endswith(".pcd")]
if pcd_files:
    pcd_path = os.path.join("data/tangle01", pcd_files[0])
    print(f"Testing on {pcd_path}...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    res = generate_graph(pcd)
    print(f"Generated graph with {res['nodes'].shape[0]} nodes and {res['edges'].shape[0]} edges.")
    print("SUCCESS")
else:
    print("No PCD files found in data/tangle01 to test with.")
