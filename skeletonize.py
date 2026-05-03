import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
import argparse
import os
import sys

# --- Functions (crop, fps, laplacian contraction, tensor voting) ---
def crop_edges_xz(pcd, fraction=0.2):
    if fraction < 0 or fraction >= 0.5:
        raise ValueError("Fraction must be between 0.0 and 0.499")

    points = np.asarray(pcd.points)
    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)

    width_x = max_bound[0] - min_bound[0]
    width_z = max_bound[2] - min_bound[2]

    margin_x = width_x * fraction
    margin_z = width_z * fraction

    new_min_bound = np.array([min_bound[0] + margin_x, min_bound[1], min_bound[2] + margin_z])
    new_max_bound = np.array([max_bound[0] - margin_x, max_bound[1], max_bound[2] - margin_z])

    # Create the bounding box
    bbox = o3d.geometry.AxisAlignedBoundingBox(new_min_bound, new_max_bound)
    
    # 1. Get the points INSIDE the box
    cropped_pcd = pcd.crop(bbox)
    
    # 2. Get the points OUTSIDE the box by finding the indices and inverting the selection
    inside_indices = bbox.get_point_indices_within_bounding_box(pcd.points)
    remainder_pcd = pcd.select_by_index(inside_indices, invert=True)

    print(f"Original point count: {len(points)}")
    print(f"Cropped inside count: {len(cropped_pcd.points)}")
    print(f"Cropped outside (remainder) count: {len(remainder_pcd.points)}")

    return cropped_pcd, remainder_pcd

def farthest_point_sampling(points, n_samples):
    N = points.shape[0]
    if n_samples >= N:
        return np.arange(N)

    selected_indices = [np.random.randint(N)]
    distances = np.full(N, np.inf)

    for _ in range(1, n_samples):
        last_point = points[selected_indices[-1]]
        dists = np.linalg.norm(points - last_point, axis=1)
        distances = np.minimum(distances, dists)
        next_index = np.argmax(distances)
        selected_indices.append(next_index)

    return np.array(selected_indices)

def build_knn(points, k=10):
    nbrs = NearestNeighbors(n_neighbors=k).fit(points)
    _, indices = nbrs.kneighbors(points)
    return indices

def laplacian_contraction(points, knn_idx, lam=0.2, iterations=8):
    pts = points.copy()
    N, k = knn_idx.shape

    for _ in range(iterations):
        new_pts = pts.copy()

        for i in range(N):
            neigh_idx = knn_idx[i, 1:]  # exclude self
            neigh = pts[neigh_idx]

            if len(neigh) == 0:
                continue

            centroid = neigh.mean(axis=0)
            degree = len(neigh)
            lam_eff = lam / (1.0 + degree * 0.1)

            new_pts[i] = pts[i] + lam_eff * (centroid - pts[i])

        pts = new_pts

    return pts

def tensor_vote_extrapolate(points, search_radius=5.0, vote_steps=5, step_size=1.0, max_nn=30, n_neighbors=10):
    """
    Uses Local PCA (Structure Tensor) to extrapolate 'tips' of lines.
    """
    points = np.asarray(points)
    num_points = len(points)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=max_nn))
    pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=search_radius, max_nn=max_nn))
    
    covariances = np.asarray(pcd.covariances) 
    new_points = []
    
    print("Analyzing Structure Tensors...")
    nbrs = NearestNeighbors(n_neighbors=n_neighbors, algorithm='kd_tree').fit(points)
    
    for i in range(num_points):
        cov = covariances[i]
        eigvals, eigvecs = np.linalg.eigh(cov) 
        
        idx = eigvals.argsort()[::-1]
        eigvals = eigvals[idx]
        eigvecs = eigvecs[:, idx]
        
        lambda1, lambda2, lambda3 = eigvals
        v1 = eigvecs[:, 0] 
        if lambda1 < 1e-6: continue 
        linearity = (lambda1 - lambda2) / lambda1
        if linearity > 0.8:
            indices = nbrs.kneighbors([points[i]], return_distance=False)[0]
            local_cloud = points[indices]
            centroid = np.mean(local_cloud, axis=0)
            
            vec_to_center = centroid - points[i]
            alignment = np.dot(vec_to_center, v1)
            
            if alignment > 0:
                shoot_dir = -v1
            else:
                shoot_dir = v1
                
            if np.linalg.norm(vec_to_center) > (search_radius * 0.2):
                current_pos = points[i].copy()
                for s in range(vote_steps):
                    current_pos += shoot_dir * step_size
                    new_points.append(current_pos)
 
    print(f"Generated {len(new_points)} extrapolated points.")
    return np.array(new_points)


# --- Main Pipeline ---
if __name__ == "__main__":
    # --- CLI Setup ---
    parser = argparse.ArgumentParser(description="Skeletonize and extrapolate a point cloud.")
    parser.add_argument("dataset_folder", type=str, help="Folder containing the point cloud dataset")
    parser.add_argument("pcd_name", type=str, help="Name of the point cloud file (e.g., file.pcd)")
    args = parser.parse_args()

    input_path = os.path.join("data", args.dataset_folder, args.pcd_name)

    if not os.path.exists(input_path):
        print(f"Error: Could not find file at '{input_path}'")
        sys.exit(1)

    # 1. Load point cloud 
    print(f"Loading point cloud from: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    points = np.asarray(pcd.points)

    # 2. Median nearest neighbor distance
    nbrs = NearestNeighbors(n_neighbors=2).fit(points)
    distances, _ = nbrs.kneighbors(points)
    median_dist = np.median(distances[:, 1])
    print(f"Median distance between points: {median_dist}")

    # 3. DBSCAN clustering
    eps = median_dist * 8
    clustering = DBSCAN(eps=eps, min_samples=5).fit(points)
    labels = clustering.labels_

    downsampled_points = []
    downsampled_labels = []

    # 4. Process each cluster (FPS -> Contract)
    print("\nSkeletonizing clusters...")
    for label in np.unique(labels):
        cluster_points = points[labels == label]
        
        sample_ratio = 0.02
        k = max(1, int(len(cluster_points) * sample_ratio))
        fps_indices = farthest_point_sampling(cluster_points, k)
        cluster_points_reduced = cluster_points[fps_indices]
        
        if len(cluster_points_reduced) < 15:
            downsampled_points.append(cluster_points_reduced)
            downsampled_labels.extend([label] * len(cluster_points_reduced))
            continue

        knn_idx = build_knn(cluster_points_reduced, k=8)
        contracted = laplacian_contraction(cluster_points_reduced, knn_idx, lam=0.4, iterations=30)

        downsampled_points.append(contracted)
        downsampled_labels.extend([label] * len(contracted))

    downsampled_points = np.vstack(downsampled_points)
    downsampled_labels = np.array(downsampled_labels)

    # Create Open3D point cloud for skeleton
    pcd_down = o3d.geometry.PointCloud()
    pcd_down.points = o3d.utility.Vector3dVector(downsampled_points)

    # Apply Random colors per cluster
    unique_labels = np.unique(downsampled_labels)
    cluster_colors = np.random.rand(len(unique_labels), 3)
    point_colors = np.array([cluster_colors[np.where(unique_labels == lbl)[0][0]] for lbl in downsampled_labels])
    pcd_down.colors = o3d.utility.Vector3dVector(point_colors)

    # 5. Crop the edges in X and Z AFTER skeletonizing
    print("\nApplying crop to XZ edges...")
    cropped_pcd, remainder_pcd = crop_edges_xz(pcd_down, fraction=0.15)

    # 6. Tensor Vote Extrapolation on the Cropped Skeleton (CLEAN VERSION)
    print("\nStarting Extrapolation...")
    base_points = np.asarray(cropped_pcd.points)
    base_colors = np.asarray(cropped_pcd.colors) 
    
    # Keep track of ONLY the newly generated points here
    accumulated_new_points = np.empty((0, 3))
    voxel_size = 1.1 # Note: You may need to tune this to match your skeleton density!

    for i in range(8):
        print(f"Growth Pass {i+1}...")
        
        # Create the context cloud (Old + New) so the algorithm can "see" the whole line
        if len(accumulated_new_points) > 0:
            context_points = np.vstack((base_points, accumulated_new_points))
        else:
            context_points = base_points

        new_growth = tensor_vote_extrapolate(
            context_points, 
            search_radius=20.0,  
            vote_steps=5, 
            step_size=2.5,       
            max_nn=30, 
            n_neighbors=10
        )
        if len(new_growth) == 0:
            break
            
        # Stack the brand new points with the previously grown points
        accumulated_new_points = np.vstack((accumulated_new_points, new_growth))
        
        # Downsample ONLY the new points. 
        # This prevents thickening/clumping WITHOUT touching your cropped skeleton
        pcd_temp = o3d.geometry.PointCloud()
        pcd_temp.points = o3d.utility.Vector3dVector(accumulated_new_points)
        pcd_temp = pcd_temp.voxel_down_sample(voxel_size=voxel_size)
        accumulated_new_points = np.asarray(pcd_temp.points)

    # 7. Compile Final Results (Adding back the remainder)
    num_inside = len(base_points)
    num_extrapolated = len(accumulated_new_points)
    
    # Generate green color array for the newly extrapolated points
    if num_extrapolated > 0:
        extrapolated_colors = np.zeros((num_extrapolated, 3)) + [0.0, 1.0, 0.0]
    else:
        extrapolated_colors = np.empty((0, 3))

    # Extract the points and colors from our saved remainder
    remainder_points = np.asarray(remainder_pcd.points)
    remainder_colors = np.asarray(remainder_pcd.colors)

    print("\n--- Final Tally ---")
    print(f"Original inner skeleton points: {num_inside}")
    print(f"Newly extrapolated points: {num_extrapolated}")
    print(f"Restored outer edge points: {len(remainder_points)}")

    # Stack everything together: Inside + Extrapolated + Outside
    if num_extrapolated > 0:
        final_points = np.vstack((base_points, accumulated_new_points, remainder_points))
        final_colors = np.vstack((base_colors, extrapolated_colors, remainder_colors))
    else:
        final_points = np.vstack((base_points, remainder_points))
        final_colors = np.vstack((base_colors, remainder_colors))

    final_pcd = o3d.geometry.PointCloud()
    final_pcd.points = o3d.utility.Vector3dVector(final_points)
    final_pcd.colors = o3d.utility.Vector3dVector(final_colors)
    
    # --- Formatting the Output Name and Saving ---
    base_name = os.path.splitext(args.pcd_name)[0] # Strips the .pcd extension
    output_filename = f"{base_name}-skeleton.pcd"
    output_path = os.path.join("data", args.dataset_folder, output_filename)

    print(f"\nSaving final skeletonized point cloud to: {output_path}")
    o3d.io.write_point_cloud(output_path, final_pcd)

    # 8. Visualize with custom background and point size
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(final_pcd)

    render_option = vis.get_render_option()
    render_option.point_size = 5 
    render_option.background_color = np.asarray([0, 0, 0])

    vis.run()
    vis.destroy_window()
