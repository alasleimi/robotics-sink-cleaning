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

import numpy as np


def quaternion_identity() -> np.ndarray:
    """
    Get the identity quaternion performing no rotation.

    :return: The quaternion performing no rotation.
    """
    return np.array([0, 0, 0, 1.0])


def quaternion_normalize(q: np.ndarray) -> np.ndarray:
    """
    Normalize a quaternion to unit length.

    :param q: Quaternion(s) to normalize.
    :return: The normalized quaternion(s).
    """
    res_shape = q.shape
    q = q.reshape(-1, 4)
    return (q / np.linalg.norm(q, axis=1).reshape(-1, 1)).reshape(res_shape)


def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    """
    Invert a (list of) quaternions.

    :param q: The quaternion(s) to invert.
    :return: The inverted quaternion(s).
    """
    q_view = q.reshape(-1, 4)
    return np.concatenate((-q_view[:, :3], q_view[:, 3].reshape(-1, 1)), axis=1).reshape(q.shape)


def quaternion_multiply(q0: np.ndarray, q1: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions.

    :param q0: Left quaternion(s).
    :param q1: Right quaternion(s).
    :return: The multiplication result.
    """
    input_shape = q1.shape if np.prod(q1.shape) > np.prod(q0.shape) else q0.shape
    x0, y0, z0, w0 = q0.reshape(-1, 4).transpose()
    x1, y1, z1, w1 = q1.reshape(-1, 4).transpose()
    return (
        np.array(
            [
                w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
                w0 * y1 + y0 * w1 + z0 * x1 - x0 * z1,
                w0 * z1 + z0 * w1 + x0 * y1 - y0 * x1,
                w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            ],
            dtype=np.float64,
        )
        .transpose()
        .reshape(input_shape)
    )


def quaternion_vector_product(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Multiply a quaternion with a vector.

    :param q: The rotation to apply.
    :param v: The vector to rotate.
    :return: The multiplication result.
    """
    return (quaternion_to_rotmat(q).reshape(-1, 3, 3) @ v.reshape((-1, 3, 1))).reshape(v.shape)


def quaternion_to_rotmat(q: np.ndarray) -> np.ndarray:
    """
    Converts quaternions in ``xyzw`` format to rotation matrices.

    :param q: `(4)` or `(N, 4)` quaternions.
    :return: `(3, 3)` or `(N, 3, 3)` rotation matrices.
    """
    out_shape_single = len(q.shape) == 1
    q = q.reshape((-1, 4))
    rotm = np.array(
        [
            [
                1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2),
                2 * (q[:, 0] * q[:, 1] - q[:, 2] * q[:, 3]),
                2 * (q[:, 0] * q[:, 2] + q[:, 1] * q[:, 3]),
            ],
            [
                2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]),
                1 - 2 * (q[:, 0] ** 2 + q[:, 2] ** 2),
                2 * (q[:, 1] * q[:, 2] - q[:, 0] * q[:, 3]),
            ],
            [
                2 * (q[:, 0] * q[:, 2] - q[:, 1] * q[:, 3]),
                2 * (q[:, 1] * q[:, 2] + q[:, 0] * q[:, 3]),
                1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2),
            ],
        ]
    ).transpose((2, 0, 1))
    if out_shape_single:
        return rotm[0]
    return rotm


def axis_angle_to_rotmat(axang: np.ndarray) -> np.ndarray:
    """
    Converts axis angle rotations (angle last) to rotation matrices.

    :param axang: `(4)` or `(N, 4)` axis angle rotations (angle last).
    :return: `(3, 3)` or `(N, 3, 3)` rotation matrices.
    """
    return quaternion_to_rotmat(axis_angle_to_quaternion(axang))


def axis_angle_to_quaternion(axang: np.ndarray) -> np.ndarray:
    """
    Converts axis angle rotations (angle last) to quaternions in ``xyzw`` format.

    :param axang: `(4)` or `(N, 4)` axis angle rotations (angle last).
    :return: `(4)` or `(N, 4)` quaternions.
    """
    return_shape = axang.shape
    axang = axang.reshape((-1, 4))
    return exp(axang[:, :3] * axang[:, 3].reshape((-1, 1))).reshape(return_shape)


def log(quats: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Compute the tangent representation of quats.

    The implementation is closely following https://github.com/strasdat/Sophus

    See "A micro Lie theory for state estimation in robotics" for details: https://arxiv.org/abs/1812.01537

    :param quats: `(4)` or `(N, 4)` Quaternion input.
    :param eps: "Small value" threshold to perform linear operation.
    :return: `(N, 3)` The tangent representation.
    """
    # determine output dimensions
    res_shape = list(quats.shape)
    res_shape[-1] = 3
    quats = quats.reshape(-1, 4)

    # Allocate data
    two_atan_nbyw_by_n = np.zeros((quats.shape[0],))
    theta = np.zeros((quats.shape[0],))

    # select w
    w = quats[:, 3]

    # Calculate n
    n = np.linalg.norm(quats[:, :3], axis=1)
    n_sq = n * n

    # Handle n_sq < eps*eps
    sel = n_sq < eps * eps
    w_sel = w[sel]
    w_sq = w_sel * w_sel
    two_atan_nbyw_by_n[sel] = 2.0 / w_sel - 2.0 / 3.0 * n_sq[sel] / (w_sel * w_sq)
    theta[sel] = 2.0 * n_sq[sel] / w_sel

    # Handle n_sq > eps*eps
    sel = ~sel
    sel3 = w > 0
    two_atan_nbyw_by_n[sel & sel3] = np.arctan2(n[sel & sel3], w[sel & sel3])
    two_atan_nbyw_by_n[sel & ~sel3] = np.arctan2(-n[sel & ~sel3], -w[sel & ~sel3])
    two_atan_nbyw_by_n[sel] = 2.0 * two_atan_nbyw_by_n[sel] / n[sel]
    theta[sel] = two_atan_nbyw_by_n[sel] * n[sel]

    # Calculate tangent
    tangent = two_atan_nbyw_by_n.reshape(-1, 1) * quats[:, :3]
    return tangent.reshape(res_shape)


def exp(tangents: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Compute quaternions from the tangent representation.

    The implementation is closely following https://github.com/strasdat/Sophus

    See "A micro Lie theory for state estimation in robotics" for details: https://arxiv.org/abs/1812.01537

    :param tangents: `(4)` or `(N, 3)` Tangent vectors.
    :param eps: "Small value" threshold to perform linear operation.
    :return: `(N, 4)` The resulting quaternions.
    """
    # determine output dimensions
    res_shape = list(tangents.shape)
    res_shape[-1] = 4
    tangents = tangents.reshape(-1, 3)

    # Allocate Quaternion
    quat = np.zeros((tangents.shape[0], 4))

    # Get magnitude of rotation
    theta_orig = np.linalg.norm(tangents, axis=1)
    theta_sq = theta_orig * theta_orig

    # Allocate storage
    imag_factor = np.zeros(tangents.shape[0])
    real_factor = np.zeros(tangents.shape[0])

    # Calculate factors for theta_sq < eps
    # copy theta to avoid in-place operations
    theta = np.zeros(theta_orig.shape)
    sel1 = theta_sq >= eps
    theta[sel1] = theta_orig[sel1]
    sel2 = ~sel1
    theta[sel2] = 0
    theta_po4 = theta_sq[sel2] * theta_sq[sel2]
    imag_factor[sel2] = 0.5 - 1.0 / 48.0 * theta_sq[sel2] + 1.0 / 3840.0 * theta_po4
    real_factor[sel2] = 1.0 - 1.0 / 8.0 * theta_sq[sel2] + 1.0 / 384.0 * theta_po4

    # Calculate factors for theta_sq > eps
    half_theta = 0.5 * theta[sel1]
    imag_factor[sel1] = np.sin(half_theta) / theta[sel1]
    real_factor[sel1] = np.cos(half_theta)

    # Construct Quaternion member
    quat[:, 0:3] = tangents * imag_factor.reshape(-1, 1)
    quat[:, 3] = real_factor

    return quaternion_normalize(quat.reshape(res_shape))


def parallel_transport(g: np.ndarray, h: np.ndarray) -> np.ndarray:
    """
    Parallel transport a vector vec in g to h.

    :param g: `(N, 4)` origin quaternion.
    :param h: `(N, 4)` target quaternion.
    :return: `(N, 3)` the parallel-transported vector.
    """
    a = log(quaternion_multiply(quaternion_inverse(g), h))
    return quaternion_to_rotmat(exp(a / 2))
