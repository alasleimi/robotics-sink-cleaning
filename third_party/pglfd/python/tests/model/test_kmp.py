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
from pglfd.model.kernel import RBFKernel
from pglfd.model.kmp import KMP
from tests.model.test_data import generate_data_sine, have_cpp

if TYPE_CHECKING:
    from pglfd.model.gaussian import Gaussian


def test_kmp() -> None:
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
    sigma = np.array([gauss.sigma for gauss in res])
    kernel = RBFKernel(EuclideanManifold(1), length_scale=0.05)

    # kmp test
    kmp = KMP(EuclideanManifold(1), ProductManifold.from_string("R3*SO3"), x, y, kernel, sigma)

    if have_cpp():
        assert kmp.is_cpp

    gmr_res = cast("Gaussian", gmr_s_origin(data_gmm[-1, 0]))
    kmp_res = cast("Gaussian", kmp(data_gmm[-1, 0]))

    assert np.allclose(gmr_res.mu, kmp_res.mu, atol=1e-3, rtol=1e-3)
    # TODO compare sigma's

    multi_skip = 50

    multi_gmr_res = cast("list", gmr_s_origin(data_gmm[::multi_skip, 0]))
    multi_kmp_res = cast("list", kmp(data_gmm[::multi_skip, 0]))
    assert len(multi_gmr_res) == data_gmm.shape[0] / multi_skip
    assert len(multi_kmp_res) == data_gmm.shape[0] / multi_skip
    for one_gmr_res, one_kmp_res in zip(multi_gmr_res, multi_kmp_res, strict=True):
        assert np.allclose(one_gmr_res.mu, one_kmp_res.mu, atol=1e-1, rtol=1e-1)

    # kmp append measurements test
    kmp.append_measurements(x[-2:] + 10, y[-2:, :], sigma[-2:, :, :])

    # kmp remove measurement test
    kmp.remove_measurements([0, 10, 29])


def test_kmp_soft_nullspace() -> None:
    x = np.linspace(0, 1, 10)
    y = np.zeros((10, 3))
    y[:, 0] = np.linspace(0, 1, 10)
    sigma = 0.1
    cov = np.repeat(np.eye(3).reshape((1, 3, 3)) * sigma, x.shape[0], axis=0)

    man_in = ProductManifold.from_string("R1")
    man_out = ProductManifold.from_string("R3")
    kernel = RBFKernel(man_in, length_scale=0.1)
    kmp = KMP(man_in, man_out, x, y, kernel, cov)
    assert np.allclose(cast("Gaussian", kmp(np.array([0]))).mu, np.array([0.00050609, 0.0, 0.0]))

    kmp.append_ns_action(np.array([0]), np.array([1, 0, 0]))
    assert np.allclose(cast("Gaussian", kmp(np.array([0]))).mu, np.array([0.01035104, 0.0, 0.0]))

    kmp.append_ns_action(np.array([0.2]), np.array([0, 1, 0]))
    assert np.allclose(cast("Gaussian", kmp(np.array([0.2]))).mu, np.array([0.19753701, 0.01533117, 0.0]))

    kmp.remove_ns_action([1])
    assert np.allclose(cast("Gaussian", kmp(np.array([0]))).mu, np.array([0.01035104, 0.0, 0.0]))

    # decorrelate
    kmp.set_ns_params(use_k_underbar=False, decorrelate=True)
    kmp.remove_ns_action([0])
    kmp.append_ns_action(np.array([0]), np.array([1, 0, 0]))
    assert np.allclose(cast("Gaussian", kmp(np.array([0]))).mu, np.array([0.01035104, 0.0, 0.0]))

    kmp.append_ns_action(np.array([0.2]), np.array([0, 1, 0]))
    assert np.allclose(cast("Gaussian", kmp(np.array([0.2]))).mu, np.array([0.19961185, 0.01496983, 0.0]))

    kmp.remove_ns_action([1])
    assert np.allclose(cast("Gaussian", kmp(np.array([0]))).mu, np.array([0.01035104, 0.0, 0.0]))
