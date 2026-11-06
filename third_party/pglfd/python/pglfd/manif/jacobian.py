# Copyright (c) 2026 Maximilian Mühlbauer
#
# This file is part of pglfd.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

""":cite:t:`muehlbauer2026unified`."""

import numpy as np

from pglfd.geom import s2, so3
from pglfd.manif.conversions import manifold_to_r3_so3
from pglfd.manif.manifold import Manifold


def manifold_jacobian(manifold: Manifold, point: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Jacobian mapping from Cartesian space into tangent space.

    Calculate the Jacobian to transform a velocity in the tangent space of ``R3*SO3`` to a velocity
    in tangent space of ``manifold`` at ``point``.

    :param manifold: The manifold.
    :param point: `(N, dim_man)` or `(dim_man)` The point at which to calculate the Jacobian.
    :param eps: Epsilon value for small radii.
    :return: The Jacobian.
    """
    # Initialize point & jacobian
    point_view = point.reshape(-1, manifold.dim_manifold)
    jacobian = np.repeat(np.eye(6).reshape(1, 6, 6), point_view.shape[0], axis=0)

    if "R3" in str(manifold):
        pass
    elif "SO2*R2" in str(manifold):
        point_r3_so3 = manifold_to_r3_so3(manifold, point_view)

        # Calculate radius
        r = np.linalg.norm(point_r3_so3[:, :2], axis=1)

        # for theta = atan2(y, x)
        jacobian[:, 0, 0] = -np.where(r > eps, point_r3_so3[:, 1], 0.0)
        jacobian[:, 0, 0] = jacobian[:, 0, 0] / np.where(r > eps, r**2, 1.0)
        jacobian[:, 0, 1] = np.where(r > eps, point_r3_so3[:, 0], 0.0)
        jacobian[:, 0, 1] = jacobian[:, 0, 1] / np.where(r > eps, r**2, 1.0)

        # for r = sqrt(x ^ 2 + y ^ 2)
        jacobian[:, 1, 0] = np.where(r > eps, point_r3_so3[:, 0], 0.0)
        jacobian[:, 1, 0] = jacobian[:, 1, 0] / np.where(r > eps, r, 1.0)
        jacobian[:, 1, 1] = np.where(r > eps, point_r3_so3[:, 1], 0.0)
        jacobian[:, 1, 1] = jacobian[:, 1, 1] / np.where(r > eps, r, 1.0)

        # for theta = atan2(y, x) -> this has an influence on the rotational
        # velocity around z!
        jacobian[:, 5, 0] = -jacobian[:, 0, 0]
        jacobian[:, 5, 1] = -jacobian[:, 0, 1]

        # rotate with the current orientation
        if point_view.shape[1] > 4:
            rotm = so3.quaternion_to_rotmat(point_view[:, 4:]).transpose((0, 2, 1))
            jacobian[:, 3:6, 0:3] = rotm @ jacobian[:, 3:6, 0:3]
    elif "S2*R1" in str(manifold):
        point_r3_so3 = manifold_to_r3_so3(manifold, point_view)

        # Calculate radius
        r = np.linalg.norm(point_r3_so3[:, :3], axis=1)

        # positional part, jacobian at origin -> angular part depends on radius
        jacobian[:, 0, 0] = np.where(r > eps, 1 / r, 0.0)
        jacobian[:, 1, 1] = np.where(r > eps, 1 / r, 0.0)
        # rotate x, y, z coordinates to the north pole (transpose of original rotmat)
        rotm = so3.axis_angle_to_rotmat(s2.vector2axis_angle(point_view[:, :3])).transpose((0, 2, 1))
        jacobian[:, :3, :3] = jacobian[:, :3, :3] @ rotm  # compose

        # first two angles also have an influence on the orientation
        jacobian[:, 3, 0] = jacobian[:, 1, 0]
        jacobian[:, 3, 1] = jacobian[:, 1, 1]
        jacobian[:, 3, 2] = jacobian[:, 1, 2]
        jacobian[:, 4, 0] = -jacobian[:, 0, 0]
        jacobian[:, 4, 1] = -jacobian[:, 0, 1]
        jacobian[:, 4, 2] = -jacobian[:, 0, 2]

        # rotate with the current orientation
        if point_view.shape[1] > 4:
            rotm = so3.quaternion_to_rotmat(point_view[:, 4:]).transpose((0, 2, 1))
            jacobian[:, 3:6, 0:3] = rotm @ jacobian[:, 3:6, 0:3]
    else:
        msg = f"Unknown manifold {manifold}"
        raise ValueError(msg)

    if len(point.shape) == 1:
        return jacobian.reshape(6, 6)
    return jacobian
