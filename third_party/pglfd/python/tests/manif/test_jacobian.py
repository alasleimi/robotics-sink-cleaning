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
import pytest

from pglfd.geom import s2, so3
from pglfd.manif.conversions import manifold_from_r3_so3
from pglfd.manif.jacobian import manifold_jacobian
from pglfd.manif.product import ProductManifold


@pytest.fixture()
def vec3():
    return np.array([1, 2, 3])


@pytest.fixture()
def q1():
    return np.array([1, 0, 0, 0])


@pytest.fixture()
def q2():
    return np.array([0, 0, 0, 1])


def test_manifold_jacobian(vec3, q1, q2) -> None:
    # prepare manifolds
    man_eucl = ProductManifold.from_string("R3*SO3")
    man_cyl = ProductManifold.from_string("SO2*R2*SO3")
    man_sph = ProductManifold.from_string("S2*R1*SO3")

    # prepare data
    data = np.concatenate((vec3, q1))
    data2 = np.concatenate((vec3, q2))
    data_cyl = manifold_from_r3_so3("SO2*R2*SO3", data)
    data2_cyl = manifold_from_r3_so3("SO2*R2*SO3", data2)
    data_sph = manifold_from_r3_so3("S2*R1*SO3", data)
    data2_sph = manifold_from_r3_so3("S2*R1*SO3", data2)
    data_rep = np.repeat(data.reshape(1, -1), 10, axis=0)
    data_rep_cyl = manifold_from_r3_so3("SO2*R2*SO3", data_rep)
    data_rep_sph = manifold_from_r3_so3("S2*R1*SO3", data_rep)
    data2_rep = np.repeat(data2.reshape(1, -1), 10, axis=0)
    data2_rep_cyl = manifold_from_r3_so3("SO2*R2*SO3", data2_rep)
    data2_rep_sph = manifold_from_r3_so3("S2*R1*SO3", data2_rep)

    # test jacobians - euclidean case
    assert np.allclose(manifold_jacobian(man_eucl, data), np.eye(6))
    assert np.allclose(manifold_jacobian(man_eucl, data_rep), np.repeat(np.eye(6).reshape(1, 6, 6), 10, axis=0))
    assert np.allclose(manifold_jacobian(man_eucl, data2), np.eye(6))
    assert np.allclose(manifold_jacobian(man_eucl, data2_rep), np.repeat(np.eye(6).reshape(1, 6, 6), 10, axis=0))

    # test jacobians - cylinder case
    jac_cyl = np.array(
        [
            [-0.4, 0.4472136, 0.0, 0.0, 0.0, -0.4],
            [0.2, 0.89442719, 0.0, 0.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]
    ).T
    jac2_cyl = np.array(
        [
            [-0.4, 0.4472136, 0.0, 0.0, 0.0, 0.4],
            [0.2, 0.89442719, 0.0, 0.0, 0.0, -0.2],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ]
    ).T
    assert np.allclose(manifold_jacobian(man_cyl, data_cyl), jac_cyl)
    assert np.allclose(manifold_jacobian(man_cyl, data_rep_cyl), np.repeat(jac_cyl.reshape(1, 6, 6), 10, axis=0))
    assert np.allclose(manifold_jacobian(man_cyl, data2_cyl), jac2_cyl)
    assert np.allclose(manifold_jacobian(man_cyl, data2_rep_cyl), np.repeat(jac2_cyl.reshape(1, 6, 6), 10, axis=0))

    # test with random data
    data = np.concatenate((np.random.rand(3) * np.random.rand(1) * 4, q1))
    data_cyl = manifold_from_r3_so3("SO2*R2*SO3", data)
    data_rep = np.repeat(data.reshape(1, -1), 10, axis=0)
    data_rep_cyl = manifold_from_r3_so3("SO2*R2*SO3", data_rep)

    # analytical jacobian
    angle = np.arctan2(data[1], data[0]) - np.pi / 2
    jac = np.eye(3)
    r = np.linalg.norm(data[:2])
    jac[0, 0] = -1 / r
    rotmat = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    jac[:2, :2] = jac[:2, :2] @ rotmat.T

    assert np.allclose(manifold_jacobian(man_cyl, data_cyl)[:3, :3], jac)
    assert np.allclose(manifold_jacobian(man_cyl, data_rep_cyl)[:, :3, :3], np.repeat(jac.reshape(1, 3, 3), 10, axis=0))

    # test jacobians - spherical case
    jac_sph = np.array(
        [
            [2.56666136e-01, -2.11902111e-02, -7.14285714e-02, 0.00000000e00, 0.00000000e00, 0.00000000e00],
            [-2.11902111e-02, 2.24880820e-01, -1.42857143e-01, 0.00000000e00, 0.00000000e00, 0.00000000e00],
            [2.67261242e-01, 5.34522484e-01, 8.01783726e-01, 0.00000000e00, 0.00000000e00, 0.00000000e00],
            [-1.93619966e-18, 2.14285714e-01, -1.42857143e-01, 1.00000000e00, 0.00000000e00, 0.00000000e00],
            [2.14285714e-01, 1.61560638e-18, -7.14285714e-02, 0.00000000e00, 1.00000000e00, 0.00000000e00],
            [-1.42857143e-01, 7.14285714e-02, -2.59852131e-18, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
    )
    jac2_sph = np.array(
        [
            [2.56666136e-01, -2.11902111e-02, -7.14285714e-02, 0.00000000e00, 0.00000000e00, 0.00000000e00],
            [-2.11902111e-02, 2.24880820e-01, -1.42857143e-01, 0.00000000e00, 0.00000000e00, 0.00000000e00],
            [2.67261242e-01, 5.34522484e-01, 8.01783726e-01, 0.00000000e00, 0.00000000e00, 0.00000000e00],
            [-1.93619966e-18, 2.14285714e-01, -1.42857143e-01, 1.00000000e00, 0.00000000e00, 0.00000000e00],
            [-2.14285714e-01, -1.61560638e-18, 7.14285714e-02, 0.00000000e00, 1.00000000e00, 0.00000000e00],
            [1.42857143e-01, -7.14285714e-02, 2.59852131e-18, 0.00000000e00, 0.00000000e00, 1.00000000e00],
        ]
    )
    print(manifold_jacobian(man_sph, data_sph))
    print(manifold_jacobian(man_sph, data2_sph))
    assert np.allclose(manifold_jacobian(man_sph, data_sph), jac_sph)
    assert np.allclose(manifold_jacobian(man_sph, data_rep_sph), np.repeat(jac_sph.reshape(1, 6, 6), 10, axis=0))
    assert np.allclose(manifold_jacobian(man_sph, data2_sph), jac2_sph)
    assert np.allclose(manifold_jacobian(man_sph, data2_rep_sph), np.repeat(jac2_sph.reshape(1, 6, 6), 10, axis=0))

    # test with method in paper
    pose2 = manifold_from_r3_so3(man_sph, np.concatenate((np.random.rand(3) * np.random.rand(1) * 4, q1)))
    pose2[4:8] = np.array([0, 0, 0, 1])
    r = pose2[3]
    r_align = so3.axis_angle_to_rotmat(s2.vector2axis_angle(pose2[:3]))
    jac = np.eye(6)
    jac[:3, :3] = np.diag([1 / r, 1 / r, 1]) @ r_align.T
    jac[3:6, :3] = np.array([[0, 1 / r, 0], [-1 / r, 0, 0], [0, 0, 0]]) @ r_align.T
    assert np.allclose(manifold_jacobian(man_sph, pose2), jac)
