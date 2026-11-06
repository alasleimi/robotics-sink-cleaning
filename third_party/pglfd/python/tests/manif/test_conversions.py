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

from pglfd.manif.conversions import manifold_from_r3_so3, manifold_to_r3_so3
from pglfd.manif.product import ProductManifold


@pytest.fixture()
def vec3():
    return np.array([1, 2, 3])


@pytest.fixture()
def q1():
    return np.array([1, 0, 0, 0])


def test_manifold_conversions(vec3, q1) -> None:
    # Spherical manifold - S2 * R1
    # Used for pointing tasks
    # prepare manifold
    man_sph = ProductManifold.from_string("S2*R1*SO3")
    data = np.concatenate((vec3, q1))
    assert np.allclose(manifold_to_r3_so3(man_sph, manifold_from_r3_so3(man_sph, data)), data)

    data_rep = np.repeat(data.reshape(1, -1), 10, axis=0)
    assert np.allclose(manifold_to_r3_so3(man_sph, manifold_from_r3_so3(man_sph, data_rep)), data_rep)

    # Cylindrical manifold - S1 * R2
    # Used for cylindrical
    # prepare manifold
    man_cyl = ProductManifold.from_string("SO2*R2*SO3")
    data = np.concatenate((vec3, q1))
    assert np.allclose(manifold_to_r3_so3(man_cyl, manifold_from_r3_so3(man_cyl, data)), data)

    data_rep = np.repeat(data.reshape(1, -1), 10, axis=0)
    assert np.allclose(manifold_to_r3_so3(man_cyl, manifold_from_r3_so3(man_cyl, data_rep)), data_rep)
