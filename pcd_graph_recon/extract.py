import sys
import re
import os

with open('/Users/grantyang/Documents/python_projects/PCD-Graph-Recon-DM/full_pipeline.py', 'r') as f:
    text = f.read()

start = text.find('def crop_edges_xz')
end = text.find('def parse_args()')

funcs = text[start:end]

funcs = funcs.replace(
    'def run_reconstruction(points, output_dir, args, prefix=""):',
    'def run_reconstruction(points, output_dir, k, metric, epsilon, persistence_threshold, tau_detour, keep_tau, prefix=""):',
)
funcs = funcs.replace('args.k', 'k')
funcs = funcs.replace('args.metric', 'metric')
funcs = funcs.replace('args.epsilon', 'epsilon')
funcs = funcs.replace('args.persistence_threshold', 'persistence_threshold')
funcs = funcs.replace('args.tau_detour', 'tau_detour')
funcs = funcs.replace('args.keep_tau', 'keep_tau')

header = '''import os
import shutil
import numpy as np
import open3d as o3d
import networkx as nx
from collections import Counter, defaultdict
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN
import dmpcd as dm
import dmpcd.pcd as pcd

try:
    from MomentumConnect import MomentumConnect
except ImportError:
    import sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from MomentumConnect import MomentumConnect

'''

with open('/Users/grantyang/Documents/python_projects/PCD-Graph-Recon-DM/pcd_graph_recon/utils.py', 'w') as f:
    f.write(header + funcs)
