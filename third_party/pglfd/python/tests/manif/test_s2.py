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

from pglfd.manif.s2 import S2Manifold


@pytest.fixture()
def vec2():
    return np.array([1, 2])


@pytest.fixture()
def rotvec3_1():
    return np.array([0, 0, 1])


@pytest.fixture()
def rotvec3_2():
    return np.array([1, 0, 0])


def test_s2_manifold(rotvec3_1, rotvec3_2, vec2) -> None:
    man = S2Manifold()
    assert man.dim_tangent == 2
    assert man.dim_manifold == 3
    assert np.allclose(man.id_elem, np.array([0, 0, 1]))
    assert np.allclose(man.exp(man.log(man.id_elem, man.id_elem), man.id_elem), man.id_elem)
    assert np.allclose(man.action(rotvec3_1, rotvec3_1, rotvec3_1), rotvec3_1)
    assert np.allclose(man.action(rotvec3_1, rotvec3_1, -rotvec3_1), -rotvec3_1)
    assert np.allclose(man.action(rotvec3_2, rotvec3_2, rotvec3_2), rotvec3_2)
    assert np.allclose(man.action(rotvec3_2, rotvec3_2, -rotvec3_2), rotvec3_2)
    assert np.allclose(man.action(rotvec3_1, rotvec3_2, rotvec3_1), -rotvec3_2)
    assert np.allclose(man.parallel_transport_trafo(rotvec3_1, rotvec3_1), np.eye(2))
    assert np.allclose(man.parallel_transport_trafo(rotvec3_2, rotvec3_2), np.eye(2))
    assert np.allclose(
        man.parallel_transport_trafo(rotvec3_1, rotvec3_2) @ man.parallel_transport_trafo(rotvec3_2, rotvec3_1), np.eye(2)
    )
    assert np.allclose(
        man.parallel_transport_vector(
            man.parallel_transport_vector(vec2.reshape(1, 2), rotvec3_1, rotvec3_2), rotvec3_2, rotvec3_1
        ),
        vec2.reshape(1, 2),
    )
