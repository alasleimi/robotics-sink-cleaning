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
from pglfd.manif.manifold import Manifold
from pglfd.manif.product import ProductManifold
from pglfd.manif.s2 import S2Manifold
from pglfd.manif.so2 import SO2Manifold


def manifold_from_r3_so3(manifold: Manifold | str, data: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Transform R3*SO3 data to manifold data.

    :param manifold: The manifold to transform into.
    :param data: The data to transform.
    :param eps: Epsilon value for small radii.
    :return: The transformed data.
    """
    data_view = data.reshape(-1, data.shape[-1])

    if str(manifold) == "R3":
        converted_data = data_view[:, :3]
    elif "R3*SO3" in str(manifold):
        converted_data = data
    elif str(manifold) == "SO2*R2":
        #  Compute the radius, set to zero for < eps
        r = np.linalg.norm(data_view[:, 0:2], axis=1)
        r_zeros = np.where(r > eps, r, 0.0).reshape(-1, 1)

        # Calculate SO(2) data
        # Return the id element for r < eps
        data_so2 = np.where((r > eps).reshape(-1, 1), data_view[:, 0:2], SO2Manifold().id_elem)
        data_so2 = data_so2 / np.where(r > eps, r, 1.0).reshape(-1, 1)

        # construct element
        converted_data = np.hstack((data_so2, r_zeros.reshape(-1, 1), data_view[:, 2].reshape(-1, 1)))
    elif "SO2*R2*SO3" in str(manifold):
        # construct element
        converted_data = np.hstack(
            (
                manifold_from_r3_so3(ProductManifold.from_string("SO2*R2"), data, eps).reshape(-1, 4),
                data_view[:, 3:],
            )
        )

        # rotate SO(3) part
        so3_addtangent = np.zeros((data_view.shape[0], 3))
        so3_addtangent[:, 2] = -SO2Manifold().log(converted_data[:, 0:2], SO2Manifold().id_elem)
        so3_add = so3.exp(so3_addtangent)
        converted_data[:, 4:] = so3.quaternion_multiply(so3_add, data_view[:, 3:])
    elif str(manifold) == "S2*R1":
        #  Compute the radius, set to zero for < eps
        r = np.linalg.norm(data_view[:, 0:3], axis=1)
        r_zeros = np.where(r > eps, r, 0.0).reshape(-1, 1)

        # Calculate S2 data
        # Return the id element for r < eps
        data_s2 = np.where((r > eps).reshape(-1, 1), data_view[:, 0:3], S2Manifold().id_elem)
        data_s2 = data_s2 / np.where(r > eps, r, 1.0).reshape(-1, 1)

        # construct element
        converted_data = np.hstack((data_s2, r_zeros.reshape(-1, 1)))

        # fix for degenerate case with r < eps
        converted_data[r < eps, 0:3] = S2Manifold().id_elem
    elif "S2*R1*SO3" in str(manifold):
        # construct element
        converted_data = np.hstack(
            (
                manifold_from_r3_so3(ProductManifold.from_string("S2*R1"), data, eps).reshape(-1, 4),
                data_view[:, 3:],
            )
        )

        # rotate SO(3) part
        so3_add = so3.quaternion_inverse(so3.axis_angle_to_quaternion(s2.vector2axis_angle(converted_data[:, :3], eps=eps)))
        converted_data[:, 4:] = so3.quaternion_multiply(so3_add, data_view[:, 3:])
    else:
        msg = f"Unknown manifold {manifold}"
        raise ValueError(msg)

    if len(data.shape) == 1:
        return converted_data.reshape(-1)
    return converted_data


def manifold_to_r3_so3(manifold: Manifold | str, data: np.ndarray, eps: float = 1e-10) -> np.ndarray:
    """
    Transform manifold data to R3*SO3 data.

    :param manifold: The manifold to transform from.
    :param data: The data to transform.
    :param eps: Epsilon value for small radii.
    :return: The transformed data.
    """
    data_view = data.reshape(-1, data.shape[-1])

    if str(manifold) == "R3":
        converted_data = np.zeros((data_view.shape[0], 7))
        converted_data[:, :3] = data
        converted_data[:, -1] = 1
    elif "R3*SO3" in str(manifold):
        converted_data = data
    elif str(manifold) == "SO2*R2":
        # extract values
        cos_theta = data_view[:, 0]
        sin_theta = data_view[:, 1]
        r = data_view[:, 2]
        z = data_view[:, 3]

        # add a unit quaternion
        unit_quat = np.zeros((data_view.shape[0], 4))
        unit_quat[:, -1] = 1

        # stack!
        converted_data = np.hstack(
            ((r * cos_theta).reshape(-1, 1), (r * sin_theta).reshape(-1, 1), z.reshape(-1, 1), unit_quat)
        )
    elif "SO2*R2*SO3" in str(manifold):
        # construct element
        converted_data = np.hstack(
            (
                manifold_to_r3_so3(ProductManifold.from_string("SO2*R2"), data, eps).reshape(-1, 7)[:, :3],
                data_view[:, 4:],
            )
        )

        # rotate SO(3) part
        so3_addtangent = np.zeros((data_view.shape[0], 3))
        so3_addtangent[:, 2] = SO2Manifold().log(data_view[:, 0:2], SO2Manifold().id_elem)
        so3_add = so3.exp(so3_addtangent)
        converted_data[:, 3:] = so3.quaternion_multiply(so3_add, converted_data[:, 3:])
    elif str(manifold) == "S2*R1":
        # extract values
        r = data_view[:, 3]

        # add a unit quaternion
        unit_quat = np.zeros((data_view.shape[0], 4))
        unit_quat[:, -1] = 1

        # stack!
        converted_data = np.hstack(
            (
                r.reshape(-1, 1) * data_view[:, :3],
                unit_quat,
            )
        )
    elif "S2*R1*SO3" in str(manifold):
        # construct element
        converted_data = np.hstack(
            (
                manifold_to_r3_so3(ProductManifold.from_string("S2*R1"), data, eps).reshape(-1, 7)[:, :3],
                data_view[:, 4:],
            )
        )

        # rotate SO(3) part
        so3_add = so3.axis_angle_to_quaternion(s2.vector2axis_angle(data_view[:, :3], eps=eps))
        converted_data[:, 3:] = so3.quaternion_multiply(so3_add, converted_data[:, 3:])
    else:
        msg = f"Unknown manifold {manifold}"
        raise ValueError(msg)

    if len(data.shape) == 1:
        return converted_data.reshape(-1)
    return converted_data


def tangent_vector_from_r3_so3(manifold: Manifold, data: np.ndarray) -> np.ndarray:
    """
    Extract part of a R3*SO3 tangent vector.

    :param manifold: The manifold to transform into.
    :param data: The R3*S3 tangent vector.
    :return: The extracted part of the tangent vector.
    """
    data_view = data.reshape(-1, data.shape[-1])

    if str(manifold) == "R3":
        converted_data = data_view[:, :3]
    elif "R6" in str(manifold):
        converted_data = data
    elif str(manifold) == "SO3":
        converted_data = data_view[:, 3:]
    else:
        msg = f"Unknown manifold {manifold}"
        raise ValueError(msg)

    if len(data.shape) == 1:
        return converted_data.reshape(-1)
    return converted_data


def tangent_matrix_from_r3_so3(manifold: Manifold, data: np.ndarray) -> np.ndarray:
    """
    Extract part of a R3*SO3 tangent matrix.

    :param manifold: The manifold to transform into.
    :param data: The R3*S3 tangent vector.
    :return: The extracted part of the tangent vector.
    """
    data_view = data.reshape(-1, data.shape[-2], data.shape[-1])

    if str(manifold) == "R3" or str(manifold) == "SO2*R2" or str(manifold) == "S2*R1":
        converted_data = data_view[:, :3, :3]
    elif "R6" in str(manifold):
        converted_data = data
    elif str(manifold) == "SO3":
        converted_data = data_view[:, 3:, 3:]
    else:
        msg = f"Unknown manifold {manifold}"
        raise ValueError(msg)

    if len(data.shape) == 1:
        return converted_data.reshape(-1)
    return converted_data
