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
from pglfd.model.gaussian import Gaussian
from pglfd.model.gmm import GMM
from tests.model.test_data import have_cpp, test_gaussian_data, test_gmm_data  # noqa: F401


@pytest.fixture()
def data(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["data"]


@pytest.fixture()
def data_s2(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["data_s2"]


@pytest.fixture()
def mu(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["mu"]


@pytest.fixture()
def sigma(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["sigma"]


@pytest.fixture()
def data_gmm(test_gmm_data):  # noqa: F811
    return test_gmm_data["data"]


@pytest.fixture()
def data_gmm_eucl(test_gmm_data):  # noqa: F811
    return test_gmm_data["gmm_eucl"]


@pytest.fixture()
def data_gmm_man(test_gmm_data):  # noqa: F811
    return test_gmm_data["gmm_man"]


def test_gmm(data, mu, sigma) -> None:
    gaussian = Gaussian.mle_from_data(ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), data)
    gmm = GMM([gaussian, gaussian, gaussian, gaussian, gaussian])
    gaussian_approx = gmm.approximate_single_gaussian(np.ones(5))
    assert np.allclose(gaussian_approx.mu, mu)
    assert np.allclose(gaussian_approx.sigma, sigma, rtol=1e-3, atol=1e-3)

    if have_cpp():
        assert gmm.is_cpp
        assert gaussian_approx.is_cpp


def test_gmm_expectation(data, mu, sigma) -> None:
    gaussian = Gaussian.mle_from_data(ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), data)
    gmm = GMM([gaussian, gaussian, gaussian, gaussian, gaussian])

    peak_val = 1 / np.sqrt((2 * np.pi) ** gaussian.manifold.dim_tangent * np.linalg.det(sigma))
    assert np.allclose(gmm.expectation(mu), np.ones(5) * 0.2 * peak_val)
    assert np.allclose(gmm.expectation(np.array([mu, mu, mu])), np.ones((3, 5)) * 0.2 * peak_val)


def test_gmm_sample(data) -> None:
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian.mle_from_data(man, data)
    gaussian.mu = gaussian.mu + np.array([1, 2, 3, 4, 0, 0, 0, 0])
    gmm = GMM([gaussian, gaussian, gaussian, gaussian, gaussian])
    assert gmm.sample(10).shape == (10, 8)

    vecs = gmm.sample(1000)
    sample_gaussian = Gaussian.mle_from_data(man, vecs)

    assert np.allclose(gaussian.mu, sample_gaussian.mu, atol=1e-3, rtol=1)
    assert np.allclose(gaussian.sigma, sample_gaussian.sigma, atol=1e-2, rtol=1)


def test_gmm_fit(data_gmm, data_gmm_eucl, data_gmm_man) -> None:
    gmm_eucl, _likelihood_eucl, _avg_loglikelihood_eucl = GMM.fit(EuclideanManifold(3), 5, data_gmm[:, 1:4])
    for i in range(5):
        found = -1
        diff = 1000.0
        for j in range(5):
            new_diff = float(np.linalg.norm(gmm_eucl.gaussians[i].mu - data_gmm_eucl.gaussians[j].mu))
            if new_diff < diff:
                diff = new_diff
                found = j
        assert found >= 0
        assert abs(gmm_eucl.priors[i] - data_gmm_eucl.priors[i]) < 0.2
        assert np.allclose(gmm_eucl.gaussians[i].mu, data_gmm_eucl.gaussians[found].mu, rtol=1, atol=1e-2)
        assert np.allclose(gmm_eucl.gaussians[i].sigma, data_gmm_eucl.gaussians[found].sigma, rtol=1e-1, atol=1e-2)

    gmm_man, _likelihood_man, _avg_loglikelihood_man = GMM.fit(
        ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), 5, data_gmm
    )
    for i in range(5):
        found = -1
        diff = 1000.0
        for j in range(5):
            if abs(gmm_man.priors[i] - data_gmm_man.priors[j]) < diff:
                diff = abs(gmm_man.priors[i] - data_gmm_man.priors[j])
                found = j
        assert found >= 0
        assert np.allclose(gmm_man.gaussians[i].mu, data_gmm_man.gaussians[found].mu, rtol=1e-2, atol=1e-3)
        assert np.allclose(gmm_man.gaussians[i].sigma, data_gmm_man.gaussians[found].sigma, rtol=1e-2, atol=1e-3)
