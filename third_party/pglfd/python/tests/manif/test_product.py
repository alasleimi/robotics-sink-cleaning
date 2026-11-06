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

from pglfd.manif.euclidean import EuclideanManifold
from pglfd.manif.product import ProductManifold
from pglfd.manif.so3 import SO3Manifold


@pytest.fixture()
def vec1():
    return np.array([1])


@pytest.fixture()
def vec3():
    return np.array([1, 2, 3])


@pytest.fixture()
def q1():
    return np.array([1, 0, 0, 0])


@pytest.fixture()
def q2():
    return np.array([0, 0, 0, 1])


def test_product_manifold(q1, q2, vec1, vec3) -> None:
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    assert man.dim_tangent == 1 + 3 + 3
    assert man.dim_manifold == 1 + 3 + 4
    assert np.allclose(man.id_elem, np.array([0, 0, 0, 0, 0, 0, 0, 1]))
    assert np.allclose(man.exp(man.log(man.id_elem, man.id_elem), man.id_elem), man.id_elem)
    point1 = np.concatenate((vec1, vec3, q1))
    point2 = np.concatenate((vec1, vec3, q2))
    assert np.allclose(man.action(point1, point1, point1), point1)
    tanvec = np.array([1, 2, 3, 4, 5, 6, 7])
    assert np.allclose(man.parallel_transport_trafo(point1, point1), np.eye(man.dim_tangent))
    assert np.allclose(man.parallel_transport_trafo(point2, point2), np.eye(man.dim_tangent))
    assert np.allclose(
        man.parallel_transport_trafo(point1, point2) @ man.parallel_transport_trafo(point2, point1), np.eye(man.dim_tangent)
    )
    assert np.allclose(
        man.parallel_transport_vector(
            man.parallel_transport_vector(tanvec.reshape(1, man.dim_tangent), point1, point2), point2, point1
        ),
        tanvec.reshape(1, man.dim_tangent),
    )
