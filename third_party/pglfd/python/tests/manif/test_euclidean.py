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


@pytest.fixture()
def vec1():
    return np.array([1])


@pytest.fixture()
def vec1_2():
    return np.array([2])


@pytest.fixture()
def vec3():
    return np.array([1, 2, 3])


def test_euclidean_manifold(vec1, vec1_2, vec3) -> None:
    man = EuclideanManifold(1)
    assert man.dim_tangent == 1
    assert man.dim_manifold == 1
    assert np.allclose(man.id_elem, np.zeros(1))
    assert np.allclose(man.exp(man.log(man.id_elem, man.id_elem), man.id_elem), man.id_elem)
    assert np.allclose(man.action(vec1, vec1, vec1), vec1)
    assert np.allclose(man.action(vec1, vec1, -vec1), -vec1)
    assert np.allclose(man.parallel_transport_trafo(vec1, vec1_2), np.eye(1))
    assert np.allclose(man.parallel_transport_vector(vec1, vec1, vec1_2), vec1)

    man = EuclideanManifold(3)
    assert man.dim_tangent == 3
    assert man.dim_manifold == 3
    assert np.allclose(man.id_elem, np.zeros(3))
    assert np.allclose(man.exp(man.log(man.id_elem, man.id_elem), man.id_elem), man.id_elem)
    assert np.allclose(man.action(vec3, vec3, vec3), vec3)
    assert np.allclose(man.action(vec3, vec3, -vec3), -vec3)
    assert np.allclose(man.parallel_transport_trafo(vec1, vec1_2), np.eye(3))
    assert np.allclose(man.parallel_transport_vector(vec3, vec1, vec1_2), vec3)
