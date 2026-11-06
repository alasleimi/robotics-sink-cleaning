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

from typing import TYPE_CHECKING, cast

import numpy as np

from pglfd.manif.product import ProductManifold
from pglfd.model.kernel import MaternKernel, RBFKernel
from pglfd.model.nw import NadarayaWatson
from tests.model.test_data import generate_data_sine, have_cpp

if TYPE_CHECKING:
    from pglfd.model.gaussian import Gaussian


def test_nadaraya_watson() -> None:
    # Sample data
    data = generate_data_sine()

    # Fit model
    man_in = ProductManifold.from_string("R1")
    man_out = ProductManifold.from_string("R3*SO3")
    kernel = RBFKernel(man_in)
    nw = NadarayaWatson(man_in, man_out, data[:, 0], data[:, 1:], kernel)

    if have_cpp():
        assert nw.is_cpp

    # Check
    x = np.linspace(data[0, 0], data[-1, 0], 30)
    assert np.allclose(
        cast("Gaussian", nw(x[0])).mu,
        np.array([0.21584121, 0.43920962, 0.5347079, 0.02233803, 0.02950541, 0.01630118, 0.99918202]),
    )

    # Matern kernel
    kernel2 = MaternKernel(man_in)
    nw = NadarayaWatson(man_in, man_out, data[:, 0], data[:, 1:], kernel2)

    # Check
    x = np.linspace(data[0, 0], data[-1, 0], 30)
    assert np.allclose(
        cast("Gaussian", nw(x[0])).mu,
        np.array([0.21662262, 0.44098243, 0.53604717, 0.02311285, 0.0307003, 0.01681721, 0.99911985]),
    )
