import argparse
from pathlib import Path
import open3d as o3d


def pcd_to_net(pcd_path, output_path=None):
    pcd_path = Path(pcd_path)

    if output_path is None:
        output_path = pcd_path.with_suffix(".net")
    else:
        output_path = Path(output_path)

    # Read point cloud
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    points = list(pcd.points)

    if len(points) == 0:
        raise ValueError("No points found in the PCD file.")

    with open(output_path, "w") as f:
        f.write(f"*Vertices {len(points)}\n")

        for i, point in enumerate(points, start=1):
            x, y, z = point

            # Same style as:
            # 1 "1" 760.25192900 260.47209500  % z=917.38066400
            f.write(
                f'{i} "{i}" '
                f'{x:.8f} {y:.8f}  % z={z:.8f}\n'
            )

    print(f"Saved .net file to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert a PCD point cloud file to a Pajek .net file with vertex locations."
    )

    parser.add_argument("pcd_file", help="Path to input .pcd file")
    parser.add_argument(
        "-o",
        "--output",
        help="Optional output .net file path. Defaults to same name as input.",
        default=None,
    )

    args = parser.parse_args()

    pcd_to_net(args.pcd_file, args.output)



#to use: 
# python pcd_to_net.py input.pcd 
#       or
# python pcd_to_net.py input.pcd -o reconstructed.net