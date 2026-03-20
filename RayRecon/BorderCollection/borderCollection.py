import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import defaultdict, Counter
import alphashape
from scipy.spatial import ConvexHull
from shapely.geometry import Point, Polygon, MultiPolygon, GeometryCollection



def build_adjacency_list(edges, n_nodes=None):
    E = np.asarray(edges)

    if E.size == 0:
        return {i: [] for i in range(n_nodes or 0)}

    uv = E[:, :2].astype(int)

    adj = defaultdict(list)

    for u, v in uv:
        adj[u].append(v)
        adj[v].append(u)

    if n_nodes is not None:
        for i in range(n_nodes):
            adj[i] = adj[i]  

    return dict(adj)

def degree_dict_from_edges(edges):
    E = np.asarray(edges)
    if E.size == 0:
        return {}

    if E.ndim == 1:
        E = E.reshape(1, -1)

    uv = E[:, :2].astype(int)

    deg = Counter()
    for u, v in uv:
        deg[int(u)] += 1
        deg[int(v)] += 1

    return dict(deg)


def get_outer_boundary_geometry(hull):
    if isinstance(hull, Polygon):
        return hull.exterior

    if isinstance(hull, MultiPolygon):
        # keep only largest polygon
        largest = max(hull.geoms, key=lambda g: g.area)
        return largest.exterior

    if isinstance(hull, GeometryCollection):
        polys = [g for g in hull.geoms if isinstance(g, Polygon)]
        if len(polys) == 0:
            return None
        largest = max(polys, key=lambda g: g.area)
        return largest.exterior

    return None

def border_points_from_convex_hull(points, edges, epsilon, allowed_degrees=None):
    """
    points: (N,2) or (N,3)
    edges:  (M,2) or (M,3)
    epsilon: max XY distance from point to convex hull boundary
    allowed_degrees: optional set/list like {1,2}

    Returns:
        border_idx: indices of border points
        border_points: coordinates of those points
        deg: degree dict
        hull_polygon: shapely Polygon
    """
    P = np.asarray(points, dtype=float)
    if P.ndim != 2 or P.shape[1] < 2:
        raise ValueError("points must have shape (N,2) or (N,3)")

    XY = P[:, :2]

    if XY.shape[0] < 3:
        raise ValueError("need at least 3 points for a convex hull")

    hull = ConvexHull(XY)
    hull_xy = XY[hull.vertices]
    hull_polygon = Polygon(hull_xy)
    boundary = hull_polygon.exterior

    deg = degree_dict_from_edges(edges)

    border_idx = []
    for i, xy in enumerate(XY):
        d = Point(float(xy[0]), float(xy[1])).distance(boundary)
        if d <= epsilon:
            if allowed_degrees is None or deg.get(i, 0) in allowed_degrees:
                border_idx.append(i)

    border_idx = np.array(border_idx, dtype=int)
    return border_idx, P[border_idx], deg, hull_polygon


def plot_border_points(points, border_idx, hull=None):
    P = np.asarray(points)

    plt.figure(figsize=(7, 7))
    plt.scatter(P[:, 0], P[:, 1], s=8, label="all points")
    plt.scatter(P[border_idx, 0], P[border_idx, 1], s=20, label="border points")

    if hull is not None:
        boundary = get_outer_boundary_geometry(hull)
        if boundary is not None:
            x, y = boundary.xy
            plt.plot(x, y)

    plt.axis("equal")
    plt.legend()
    plt.show()



if __name__ == "__main__":
    data = np.load("C:/Users/samue/Downloads/Research/Spider/PCD-Graph-Recon-DM/RayRecon/BorderCollection/graph_data.npz")
    P = data["points"]
    E = data["edges"]
    print("Start Hull")
    border_idx, border_pts, deg, hull_poly = border_points_from_convex_hull(
        points=P,
        edges=E,
        epsilon=15.0,
        allowed_degrees={1, 2}
    )

    print("num border points:", len(border_idx))
    print(border_idx)

    plot_border_points(P, border_idx, hull_poly)