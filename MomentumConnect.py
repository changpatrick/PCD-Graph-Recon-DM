import numpy as np
import open3d as o3d
from collections import defaultdict

# momentum based connection

#feed in bad edges
def MomentumConnect(edges, badedges, points, KeepTau, keepDist):
    adjDict= defaultdict(list)

    for u,v in edges:
        adjDict[u].append(v)
        adjDict[v].append(u)


    def cosine_similarity(v1, v2):
        v1 = np.array(v1)
        v2 = np.array(v2)
        return abs(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

    AddedBack = []

    for u,v in badedges:
        #go thru all edges connected to u in kept edges, and find cosine sim between those edges and the bad edge in question, keep the smallest cossin sim. If more than 1/2, add back
        #take min between u and v cossim.

        ucossim = 1
        vcossim = 1

        ucoord = points[u]
        vcoord = points[v]

        v2 = ucoord-vcoord

        for adjVert in adjDict[u]:
            adjcoord = points[adjVert]

            v1 = adjcoord-ucoord

            ucossim = min(ucossim, cosine_similarity(v1, v2))
        
        for adjVert in adjDict[v]:
            adjcoord = points[adjVert]

            v1 = adjcoord-vcoord

            vcossim = min(vcossim, cosine_similarity(v1, v2))

        if min(ucossim, vcossim)> KeepTau:
            AddedBack.append([u,v])
        
    return AddedBack

