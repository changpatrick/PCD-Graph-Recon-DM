# WebMerge Pipeline

This tool provides a single entry point for merging two raw PCD scans of the same spider web taken from different angles.

## How it works
1. **Skeletonization**: Converts raw "fuzzy" scans into clean, single-pixel-width skeletons.
2. **Robust Registration**: Performs a brute-force search for the best rotation around the vertical axis (Z-axis).
3. **Super-Snap ICP**: Uses a large-radius Iterative Closest Point refinement to force threads to align.
4. **Thread Fusion**: Voxel-downsamples the combined cloud to merge parallel "ghost" threads into single lines.
5. **Color Coding**: Automatically paints Scan 1 **Red** and Scan 2 **Green** for easy verification.

## Usage
Run the script from the root of the repository:

```bash
python3 WebMerge/merge_web_scans.py <path_to_scan1.pcd> <path_to_scan2.pcd> --output <output_path.pcd>
```

The script will generate two files:
*   `<output_path>.pcd`: The final fused skeleton (Ready for Reconstruction).
*   `<output_path>_debug.pcd`: A color-coded version showing the red/green overlap.


## TopoWebMerge Usage
This script takes in two raw pcds and outputs the final graph after processing including degree-2 vertex collapse. 
Run the script from the root of the repository:

```bash
python3 WebMerge/TopoWebMerge.py <path_to_scan1.pcd> <path_to_scan2.pcd> --output <pajek_path.net> --pcd_output <pcd_path.pcd> <options>
```

The script will generate three files:
*   `<pcd_path>.pcd`: Vertices of the final graph.
*   `<pcd_path>_debug.pcd`: A color-coded version showing the red/green overlap.
*   `<pajek_path>.net`: The actual final graph Pajek file; visualize with WebMerge/pajek_graph_vis.py (edit .net file path within)


### Options
* `--fusion-radius`: (Default: 5.0) The voxel size used to merge close threads. If you still see "double threads", increase this value slightly (e.g., to 7.0).

## Example
```bash
python3 WebMerge/merge_web_scans.py \
    "data/tangle18/scan1.pcd" \
    "data/tangle18/scan2.pcd" \
    --output "data/tangle18/unified_skeleton.pcd"
```

## Viewing Results
To quickly inspect the merged skeleton, you can use the following Open3D one-liner:

```bash
python3 -c "import open3d as o3d; pcd = o3d.io.read_point_cloud('path/to/merged_web.pcd'); o3d.visualization.draw_geometries([pcd])"
```

Alternatively, you can open the output `.pcd` file in external tools like **MeshLab** or **CloudCompare**.
