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

from pglfd.geom.so3 import quaternion_inverse, quaternion_multiply, quaternion_vector_product


def invert_transformation(pose: np.ndarray) -> np.ndarray:
    """
    Invert transformations.

    :param pose: `(7)` or `(N, 7)` the transformations to invert.
    :return: `(7)` or `(N, 7)` the inverted transformations.
    """
    # get the input shape
    ret_shape = pose.shape

    # reshape + allocate the result
    pose = pose.reshape((-1, 7))
    ret_pose = np.zeros(pose.shape)

    # first invert the quaternion (R^-1)
    ret_pose[:, 3:] = quaternion_inverse(pose[:, 3:])

    # then invert the position part (t^-1 = - R^-1 t)
    ret_pose[:, :3] = -quaternion_vector_product(ret_pose[:, 3:], pose[:, :3])

    return ret_pose.reshape(ret_shape)


def multiply_poses(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """
    Multiply poses, left*right. They can either be single poses or batches, if both are batches, their size has to match.

    :param left: `(N, 7)` or `(7)` Left pose(s).
    :param right: `(N, 7)` or `(7)` Right pose(s).
    :return: `(N, 7)` or `(7)` Multiplied poses.
    """
    # Check which inputs are batched
    left_batched = len(left.shape) > 1
    right_batched = len(right.shape) > 1

    # Check if left batched sizes match & determine output dim
    if left_batched or right_batched:
        if left.shape[0] != right.shape[0] and left_batched and right_batched:
            msg = "Left and right side batches must agree"
            raise ValueError(msg)
        n = left.shape[0] if left_batched else right.shape[0]
        out_shape = [n, 7]
    else:
        n = 1
        out_shape = [7]

    # Multiply
    left = left.reshape((-1, 7))
    right = right.reshape((-1, 7))
    res = np.zeros((n, 7))
    res[:, :3] = left[:, :3] + quaternion_vector_product(left[:, 3:], right[:, :3])
    res[:, 3:] = quaternion_multiply(left[:, 3:], right[:, 3:])

    return res.reshape(out_shape)
