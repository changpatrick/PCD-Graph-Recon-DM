import open3d as o3d
import numpy as np

def visualize_pajek_3d(file_path):
    points = []
    edges = []
    reading_vertices = False
    reading_edges = False

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            
            if line.lower().startswith("*vertices"):
                reading_vertices = True
                reading_edges = False
                continue
            elif line.lower().startswith("*edges") or line.lower().startswith("*arcs"):
                reading_vertices = False
                reading_edges = True
                continue
            
            parts = line.split()
            if reading_vertices:
                # Format: ID "Label" X Y Z
                # We skip parts[0] (ID) and parts[1] (Label)
                try:
                    x, y, z = map(float, parts[2:5])
                    points.append([x, y, z])
                except (ValueError, IndexError):
                    continue
            
            elif reading_edges:
                # Format: Source Target
                try:
                    u, v = map(int, parts[:2])
                    # Pajek is 1-indexed, Open3D is 0-indexed
                    edges.append([u - 1, v - 1])
                except (ValueError, IndexError):
                    continue

    # Create Open3D LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(points))
    line_set.lines = o3d.utility.Vector2iVector(np.array(edges))
    
    # Color the lines (black or dark grey)
    colors = [[0.1, 0.1, 0.1] for _ in range(len(edges))]
    line_set.colors = o3d.utility.Vector3dVector(np.array(colors))

    # Launch Visualizer
    print(f"Visualizing {len(points)} nodes and {len(edges)} edges...")
    o3d.visualization.draw_geometries([line_set], window_name="Spiderweb Graph Visualization")

if __name__ == "__main__":
    visualize_pajek_3d("data/tangle19/tangle19m_g.net")