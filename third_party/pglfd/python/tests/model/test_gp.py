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

from pglfd.manif.euclidean import EuclideanManifold
from pglfd.manif.product import ProductManifold
from pglfd.manif.so3 import SO3Manifold
from pglfd.model.gmm import GMM
from pglfd.model.gp import GaussianProcess
from pglfd.model.kernel import RBFKernel
from tests.model.test_data import generate_data_sine, have_cpp

if TYPE_CHECKING:
    from pglfd.model.gaussian import Gaussian


def test_gaussian_process() -> None:
    # Sample data
    data_gmm = generate_data_sine()

    # Create GMR with s as input
    gmm_man, _likelihood_man, _avg_loglikelihood_man = GMM.fit(
        ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), 3, data_gmm
    )
    gmr_s_origin = gmm_man.gmr([0], [1, 2])

    x = np.linspace(data_gmm[0, 0], data_gmm[-1, 0], 30)
    res = cast("list", gmr_s_origin(x))
    y = np.array([gauss.mu for gauss in res])
    kernel = RBFKernel(EuclideanManifold(1), length_scale=0.05)

    multi_skip = 50
    gmr_res = cast("Gaussian", gmr_s_origin(data_gmm[-1, 0]))
    multi_gmr_res = cast("list", gmr_s_origin(data_gmm[::multi_skip, 0]))

    # gp test
    gp = GaussianProcess(ProductManifold.from_string("R1"), ProductManifold.from_string("R3*SO3"), x, y, kernel, 0.01)

    if have_cpp():
        assert gp.is_cpp

    gp_res = cast("Gaussian", gp(data_gmm[-1, 0]))
    assert np.allclose(gmr_res.mu, gp_res.mu, atol=1e-3, rtol=1e-3)

    multi_gp_res = cast("list", gp(data_gmm[::multi_skip, 0]))
    assert len(multi_gp_res) == data_gmm.shape[0] / multi_skip
    for one_gmr_res, one_gp_res in zip(multi_gmr_res, multi_gp_res, strict=True):
        assert np.allclose(one_gmr_res.mu, one_gp_res.mu, atol=1e-1, rtol=1e-1)
