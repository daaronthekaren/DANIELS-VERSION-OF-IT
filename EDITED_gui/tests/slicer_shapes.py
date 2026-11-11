import numpy as np


# -------------------shapes-----------------
def box(x0, y0, z0, x1, y1, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [[v[a], v[b], v[c]] for a, b, c in faces]


def ring_triangles():
    o, i, h = 0.05, 0.02, 0.03
    tris = box(-o, -o, 0, o, -i, h) + box(-o, i, 0, o, o, h) + box(-o, -i, 0, -i, i, h) + box(i, -i, 0, o, i, h)
    return np.array(tris, dtype=float)
