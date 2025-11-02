import numpy as np
import open3d as o3d
import os


def pcd_to_txt(pcd_object, output_filepath):
    """
    Converts an Open3D PointCloud object to a space-separated TXT file.

    Args:
        pcd_object (open3d.geometry.PointCloud): The input Open3D point cloud.
        output_filepath (str): The path where the output .txt file will be saved.
    """
    try:
        # Extract points from the point cloud object as a NumPy array
        points = np.asarray(pcd_object.points)

        # Save the NumPy array to a text file
        # The '%.6f' format ensures floating-point numbers are saved with precision.
        # The delimiter=' ' ensures that the coordinates are space-separated.
        np.savetxt(output_filepath, points, fmt='%.6f', delimiter=' ')

        print(f"Successfully converted and saved point cloud to {output_filepath}")
        print(f"Total points saved: {len(points)}")

    except Exception as e:
        print(f"An error occurred: {e}")

def cluster_and_reduce_pcd(pcd_object, voxel_size=5):
    """
    Reduces the number of points in a point cloud using voxel downsampling.
    This effectively clusters points that fall within the same 3D voxel.

    Args:
        pcd_object (open3d.geometry.PointCloud): The input Open3D point cloud.
        voxel_size (float): The size of the voxel grid. A larger size results
                            in a greater reduction of points.

    Returns:
        open3d.geometry.PointCloud: The downsampled point cloud.
    """
    downsampled_pcd = pcd_object.voxel_down_sample(voxel_size)
    return downsampled_pcd


if __name__ == "__main__":
    file_path = r"/Users/grantyang/Documents/python_projects/spiderweb/video_processing/point_clouds/tangle001 255 2025-09-18 05-01-48 T0.45.pcd"
    pcd = o3d.io.read_point_cloud(file_path)
    pcd = cluster_and_reduce_pcd(pcd)
    o3d.visualization.draw_geometries([pcd])
    pcd_to_txt(pcd, f"data/test/{os.path.basename(file_path).split('.pcd')[0]}.txt")