import numpy as np
from sinkcleaning.mesh_distance import MeshSurface


def test_nearest_triangle_interior_edge_vertex_and_vertical_height():
    surface = MeshSurface([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]])
    distance, height = surface.query([[.2, .2, .3], [.6, .6, 0], [-.1, -.2, 0], [.2, .2, -.3]])
    np.testing.assert_allclose(distance, [.3, np.sqrt(.02), np.sqrt(.05), .3])
    np.testing.assert_allclose(height[[0, 3]], [0, 0])
    assert np.isnan(height[[1, 2]]).all()


def test_topmost_surface_and_vertical_face():
    surface = MeshSurface([[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                           [[0, 0, 1], [1, 0, 1], [0, 1, 1]],
                           [[0, 0, 0], [0, 0, 1], [0, 1, 0]]])
    distance, height = surface.query([[.2, .2, .8], [-.1, .2, .2]])
    np.testing.assert_allclose(distance, [.2, .1])
    assert height[0] == 1 and np.isnan(height[1])
