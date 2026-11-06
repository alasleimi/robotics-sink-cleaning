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

"""Implementations of the SO(2) group / Riemannian manifold."""

import numpy as np


def rotmat2vector(rotmat: np.ndarray) -> np.ndarray:
    """
    Convert a 2x2 rotation matrix to a 2-dimensional rotation vector.

    :param rotmat: `(N, 2, 2)` or `(2, 2)` The rotation matrix to convert.
    :return: `(N, 2)` or `(2)` The rotation vector.
    """
    rotmat_view = rotmat.reshape((-1, 2, 2))

    # construct result vector
    vector = np.zeros((rotmat_view.shape[0], 2))
    vector[:, 0] = rotmat_view[:, 0, 0]
    vector[:, 1] = rotmat_view[:, 1, 0]

    # return same shape as input
    if len(rotmat.shape) == 2:
        return vector.reshape(-1)
    return vector


def vector2rotmat(vectors: np.ndarray) -> np.ndarray:
    """
    Convert a 2-dimensional rotation vector to a 2x2 rotation matrix.

    :param vectors: `(N, 2)` or `(2)` The rotation vector to convert.
    :return: `(N, 2, 2)` or `(2, 2)` The rotation matrix.
    """
    vectors_view = vectors.reshape((-1, 2))

    # construct rotation matrix
    rotmat = np.zeros((vectors_view.shape[0], 2, 2))
    rotmat[:, 0, 0] = vectors_view[:, 0]
    rotmat[:, 1, 0] = vectors_view[:, 1]
    rotmat[:, 0, 1] = -vectors_view[:, 1]
    rotmat[:, 1, 1] = vectors_view[:, 0]

    # return same shape as input
    if len(vectors.shape) == 1:
        return rotmat.reshape(2, 2)
    return rotmat


def normalize(vectors: np.ndarray) -> np.ndarray:
    """
    Normalize vectors to unit length.

    :param vectors: `(N, 2)` or `(2)` The vectors to normalize.
    :return: `(N, 2)` or `(2)` The normalized vectors.
    """
    vectors_view = vectors.reshape((-1, 2))
    return (vectors_view / np.linalg.norm(vectors_view, axis=1, keepdims=True)).reshape(vectors.shape)


def multiply(v0: np.ndarray, v1: np.ndarray) -> np.ndarray:
    """
    Multiply v0 and v1.

    :param v0: `(N, 2)` or `(2)` The first set of vectors.
    :param v1: `(N, 2)` or `(2)` The second set of vectors.
    :return: `(N, 2)` or `(2)` The multiplication result.
    """
    v0_view = v0.reshape((-1, 2))
    v1_view = v1.reshape((-1, 2))

    # Complex multiplication
    mult = np.hstack(
        (
            (v0_view[:, 0] * v1_view[:, 0] - v0_view[:, 1] * v1_view[:, 1]).reshape(-1, 1),
            (v0_view[:, 0] * v1_view[:, 1] + v0_view[:, 1] * v1_view[:, 0]).reshape(-1, 1),
        )
    )

    # return same shape as input
    if len(v0.shape) == 1 and len(v1.shape) == 1:
        return mult.reshape(-1)
    return mult


def inverse(vectors: np.ndarray) -> np.ndarray:
    """
    Compute the inverse of vectors.

    :param vectors: `(N, 2)` or `(2)` The vectors to calculate the inverse of.
    :return: `(N)` The inverse.
    """
    vectors_view = vectors.reshape((-1, 2))
    return np.hstack((vectors_view[:, 0].reshape(-1, 1), -vectors_view[:, 1].reshape(-1, 1))).reshape(vectors.shape)


def log(vectors: np.ndarray) -> np.ndarray:
    """
    Compute the tangent representation of vectors at the origin.

    :param vectors: `(N, 2)` or `(2)` The vectors to calculate the tangent representation of.
    :return: `(N)` The tangent representation.
    """
    vectors_view = vectors.reshape((-1, 2))
    tangents = np.arctan2(vectors_view[:, 1], vectors_view[:, 0])

    # return same shape as input
    if len(vectors.shape) == 1:
        return tangents[0]
    return tangents


def exp(tangents: np.ndarray) -> np.ndarray:
    """
    Compute the exponential of tangents at the origin.

    :param tangents: `(N)` The tangents to exponentiate.
    :return: `(N, 2)` or `(2)` The exponential representation.
    """
    tangents_view = tangents.reshape(-1)
    vectors = np.hstack((np.cos(tangents_view).reshape(-1, 1), np.sin(tangents_view).reshape(-1, 1)))

    # return same shape as input
    if len(tangents.shape) == 0:
        return vectors.reshape(-1)
    return vectors


def parallel_transport(g: np.ndarray, h: np.ndarray) -> np.ndarray:
    """
    Parallel transport matrix for vectors in g to h.

    :param g: `(N, 2)` origin vector.
    :param h: `(N, 2)` target vector.
    :return: `(N, 1, 1)` rotation matrix to align vectors in g with h.
    """
    _ = (g, h)  # unused
    return np.array([1])  # no tangent misalignment in SO(2)
