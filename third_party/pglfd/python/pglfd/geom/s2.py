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

"""
Implementations of the 2-dimensional unit sphere embedded in the 3-dimensional ambient space.

TODO understand our multiply, inverse operations
"""

import numpy as np

from pglfd.geom.so3 import axis_angle_to_rotmat


def vector2axis_angle(vectors: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Convert vectors to an axis angle representation.

    (axis, angle) describe the action needed to transform from [0, 0, 1] to vector.

    :param vectors: `(N, 3)` or `(3)` The vectors to transform.
    :param eps: Minimal rotation axis length to consider before reverting to the unit.
    :return: `(N, 3)` or `(3)` The axis-angle representation.
    """
    vectors_view = vectors.reshape((-1, 3))

    # preallocate results
    res_vecs = np.zeros(vectors_view.shape)
    res_angles = np.zeros(vectors_view.shape[0])

    # calculate norm and angle
    norm = np.linalg.norm(vectors_view[:, :2], axis=1)

    # where norm < eps and z ~ 1 don't care, with z ~ -1 rotate by 180° along x axis
    res_vecs[norm < eps, :] = np.array([1, 0, 0]).reshape(1, -1)
    res_angles[norm < eps] = np.where(vectors_view[norm < eps, 2] > 0, 0, np.pi)

    # cases with norm > eps
    res_vecs[norm >= eps, :2] = np.hstack(
        (-vectors_view[norm >= eps, 1].reshape(-1, 1), vectors_view[norm >= eps, 0].reshape(-1, 1))
    ) / norm[norm >= eps].reshape(-1, 1)
    res_angles[norm >= eps] = np.arccos(vectors_view[norm >= eps, 2])

    # return same shape as input
    if len(vectors.shape) == 1:
        return np.hstack((res_vecs.reshape(-1), res_angles.reshape(-1)))
    return np.hstack((res_vecs, res_angles.reshape(-1, 1)))


def normalize(vectors: np.ndarray) -> np.ndarray:
    """
    Normalize vectors to unit length.

    :param vectors: `(N, 3)` or `(3)` The vectors to normalize.
    :return: `(N, 3)` or `(3)` The normalized vectors.
    """
    vectors_view = vectors.reshape((-1, 3))
    return (vectors_view / np.linalg.norm(vectors_view, axis=1, keepdims=True)).reshape(vectors.shape)


def multiply(v0: np.ndarray, v1: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Multiply v0 and v1. This computes v1, defined relative to [0, 0, 1] relative to v0.

    :param v0: `(N, 3)` or `(3)` The first set of vectors.
    :param v1: `(N, 3)` or `(3)` The second set of vectors.
    :param eps: Minimal rotation axis length to consider before reverting to the unit.
    :return: `(N, 3)` or `(3)` The multiplication result.
    """
    v0_view = v0.reshape((-1, 3))
    v1_view = v1.reshape((-1, 3))

    # Multiplication
    rotmat = axis_angle_to_rotmat(vector2axis_angle(v0_view, eps=eps))
    mult = rotmat @ v1_view.reshape(-1, 3, 1)

    # return same shape as input
    if len(v0.shape) == 1 and len(v1.shape) == 1:
        return mult.reshape(-1)
    return mult.reshape(-1, 3)


def inverse(vectors: np.ndarray) -> np.ndarray:
    """
    Compute the inverse of vectors s.t. vectors relative to [0, 0, 1] is the same as [0, 0, 1] relative to the result.

    :param vectors: `(N, 3)` or `(3)` The vectors to calculate the inverse of.
    :return: `(N)` The inverse.
    """
    vectors_view = vectors.reshape((-1, 3))
    res = np.copy(vectors_view)

    # calculate the inverse: same z coordinate, invert x and y
    res[:, 0] = -res[:, 0]
    res[:, 1] = -res[:, 1]
    return res.reshape(vectors.shape)


def log(vectors: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Compute the tangent representation of vectors at the origin.

    :param vectors: `(N, 3)` or `(3)` The vectors to calculate the tangent representation of.
    :param eps: Minimal rotation axis length to consider before reverting to the unit.
    :return: `(N, 2)` The tangent representation.
    """
    vectors_view = vectors.reshape((-1, 3))
    res = np.zeros((vectors_view.shape[0], 2))

    # tangent for elems with vector size < eps is [0, 0]
    sel = np.abs(1 - vectors_view[:, 2]) > eps
    res[sel] = (
        np.arccos(vectors_view[sel, 2]).reshape(-1, 1)
        * vectors_view[sel, 0:2]
        / np.linalg.norm(vectors_view[sel, 0:2], axis=1, keepdims=True)
    )

    # return same shape as input
    if len(vectors.shape) == 1:
        return res.reshape(-1)
    return res


def exp(tangents: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Compute the exponential of tangents at the origin.

    :param tangents: `(N, 2)` or `(2)` The tangents to exponentiate.
    :param eps: Minimal rotation axis length to consider before reverting to the unit.
    :return: `(N, 3)` or `(3)` The exponential representation.
    """
    # Prepare outputs
    tangents_view = tangents.reshape((-1, 2))
    res = np.repeat(np.array([[0, 0, 1]], dtype=float), tangents_view.shape[0], axis=0)

    # Perform exponentiation
    norm = np.linalg.norm(tangents_view, axis=1)
    sel = norm > eps
    res[sel, :] = np.hstack(
        (np.sin(norm[sel]).reshape(-1, 1) * tangents_view[sel, :] / norm[sel].reshape(-1, 1), np.cos(norm[sel]).reshape(-1, 1))
    )

    # return same shape as input
    if len(tangents.shape) == 0:
        return res.reshape(-1)
    return res


def parallel_transport(g: np.ndarray, h: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Parallel transport matrix for vectors in g to h.

    :param g: `(N, 3)` origin vector.
    :param h: `(N, 3)` target vector.
    :param eps: Minimal rotation axis length to consider before reverting to the unit.
    :return: `(N, 2, 2)` rotation matrix to align vectors in g with h.
    """
    Reg = axis_angle_to_rotmat(vector2axis_angle(g, eps=eps))
    Reh = axis_angle_to_rotmat(vector2axis_angle(h, eps=eps))

    v = Reg @ np.hstack([log(multiply(inverse(g), h)), [0]])
    m = np.linalg.norm(v)
    if m < eps:
        Rgh = np.eye(3)
    else:
        u = v / m
        Rgh = (
            np.eye(3)
            - np.sin(m) * np.matmul(g.reshape(3, 1), u.reshape(1, 3))
            + (np.cos(m) - 1) * np.matmul(u.reshape(3, 1), u.reshape(1, 3))
        )

    B = np.eye(3)[:, 0:2]
    R = B.T @ Reh.T @ Rgh @ Reg @ B

    # ensure proper rotation
    if np.sign(np.trace(R)) == -1:
        R = -R

    return R
