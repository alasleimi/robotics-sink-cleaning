"""Offline distances to the actual triangle mesh, not its convex hull."""
from pathlib import Path
import numpy as np


class MeshSurface:
    def __init__(self, triangles):
        triangles = np.asarray(triangles, dtype=float)
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        ab, ac = b-a, c-a
        normal = np.cross(ab, ac)
        n2 = np.sum(normal**2, axis=1)
        good = n2 > 1e-20
        self.a, self.b, self.c, self.ab, self.ac, self.normal, self.n2 = (
            value[good] for value in (a, b, c, ab, ac, normal, n2))
        if not good.any():
            raise ValueError("Mesh has no nondegenerate triangles")

    @classmethod
    def from_obj(cls, path):
        vertices, faces = [], []
        for line in Path(path).read_text().splitlines():
            if line.startswith("v "):
                vertices.append([float(v) for v in line.split()[1:]])
            elif line.startswith("f "):
                faces.append([int(v.split("/")[0])-1 for v in line.split()[1:]])
        return cls(np.asarray(vertices)[np.asarray(faces)])

    def query(self, points):
        """Unsigned nearest-surface distance and topmost Z directly under XY.

        Height is NaN outside the mesh footprint (including holes). A signed
        height difference is not a collision/penetration test for a sponge.
        """
        a, b, c, ab, ac = self.a, self.b, self.c, self.ab, self.ac
        g00, g01, g11 = (np.sum(x*y, axis=1) for x, y in ((ab, ab), (ab, ac), (ac, ac)))
        determinant = g00*g11-g01*g01
        xy_det = ab[:, 0]*ac[:, 1]-ab[:, 1]*ac[:, 0]
        vertical = np.abs(xy_det) > 1e-12
        distances, heights = [], []
        for point in np.asarray(points):
            ap = point-a
            dot_ab, dot_ac = np.sum(ap*ab, axis=1), np.sum(ap*ac, axis=1)
            u = (dot_ab*g11-dot_ac*g01)/determinant
            v = (dot_ac*g00-dot_ab*g01)/determinant
            inside = (u >= 0) & (v >= 0) & (u+v <= 1)
            squared = np.where(inside, np.sum(ap*self.normal, axis=1)**2/self.n2, np.inf)
            for start, end in ((a, b), (b, c), (c, a)):
                edge = end-start
                t = np.clip(np.sum((point-start)*edge, axis=1)/np.sum(edge**2, axis=1), 0, 1)
                squared = np.minimum(squared, np.sum((point-start-t[:, None]*edge)**2, axis=1))
            distances.append(np.sqrt(squared.min()))
            u, v = np.full(len(a), np.nan), np.full(len(a), np.nan)
            u[vertical] = (ap[vertical, 0]*ac[vertical, 1]-ap[vertical, 1]*ac[vertical, 0])/xy_det[vertical]
            v[vertical] = (ab[vertical, 0]*ap[vertical, 1]-ab[vertical, 1]*ap[vertical, 0])/xy_det[vertical]
            hit = vertical & (u >= -1e-9) & (v >= -1e-9) & (u+v <= 1+1e-9)
            heights.append(np.max(a[hit, 2]+u[hit]*ab[hit, 2]+v[hit]*ac[hit, 2]) if hit.any() else np.nan)
        return np.asarray(distances), np.asarray(heights)
