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


def test_gmm_gmr(data_gmm_man) -> None:
    # Create GMR with s as input
    gmr_s = data_gmm_man.gmr([0], [1, 2])

    if have_cpp():
        assert gmr_s.is_cpp

    gmr_result = [
        1.17200068e00,
        6.43068198e-03,
        -7.27453326e-04,
        1.06913523e-03,
        1.00761153e-03,
        -8.36800293e-04,
        9.99998571e-01,
    ]
    assert np.allclose(
        gmr_s(np.array(0)).mu,
        np.array(gmr_result),
    )
    multi_res = gmr_s(np.array([0, 0, 0]))
    assert len(multi_res) == 3
    for res in multi_res:
        assert np.allclose(res.mu, np.array(gmr_result))

    # Create GMR with position as input
    gmr_position = data_gmm_man.gmr([1], [2])
    assert np.allclose(
        gmr_position(np.array([1.1, 0.1, 0.2]), eps_marg=1e-3, eps_cond=1e-4).mu,
        np.array([1.97060692e-03, 1.28006942e-03, -3.07283189e-04, 9.99997192e-01]),
        rtol=1e-5,
        atol=1e-4,
    )

    # Create GMR with orientation as input
    gmr_position = data_gmm_man.gmr([2], [1])
    assert np.allclose(
        gmr_position(np.array([0.2574188, 0.1716125, 0.0429031, 0.9499706]), eps_marg=1e-3, eps_cond=1e-4).mu,
        np.array([2.53102369, -0.07047645, 0.00640427]),
        rtol=1,
        atol=1e-2,
    )
