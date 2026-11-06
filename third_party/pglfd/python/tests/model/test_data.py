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

import os
import pickle

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def have_cpp() -> bool:
    return bool(os.getenv("HAVE_CPP"))


def generate_data(num=1000, s_offset=0, pos_offset=None, euler_scaler=0.1, euler_start=None):
    if pos_offset is None:
        pos_offset = np.array([0, 0, 0])
    if euler_start is None:
        euler_start = np.array([0, 0, 0])
    data = 0.3 * np.random.randn(num, 8)
    data[:, 0] *= 0.1 + s_offset
    data[:, 1:4] += pos_offset
    data[:, 4:9] = Rotation.from_euler(
        "xyz", np.random.randn(num, 3) * euler_scaler + euler_start.reshape(1, 3), degrees=False
    ).as_quat()
    return data


def generate_data_s2(num=1000):
    data = 0.05 * np.random.randn(num, 3)
    data += np.array([[0.5, 0.5, 0]])
    data *= 1 / np.linalg.norm(data, axis=1, keepdims=True)
    return data


def generate_data_sine(num=1000):
    data = np.zeros((num, 8))
    data[:, 0] = np.linspace(0, 1, num)
    data[:, 1:4] = np.array([0.2, 0.25, 0.15]).reshape(1, -1) * np.sin(
        np.array([1, 2, 3]).reshape(1, -1) * data[:, 0].reshape(-1, 1)
    ) + np.array([0.2, 0.4, 0.5]).reshape(1, -1)
    data[:, 4:9] = Rotation.from_euler(
        "xyz", np.array([0.2, 0.25, 0.15]).reshape(1, -1) * np.sin(3 * data[:, 0]).reshape(-1, 1), degrees=False
    ).as_quat()
    return data


@pytest.fixture()
def test_gaussian_data():
    # Same random seed = same result
    np.random.seed(4)

    with open("tests/model/test_gaussian.pickle", "rb") as f:
        return pickle.load(f)

    # uncomment to generate new data
    # data = {}
    # data["data"] = generate_data(pos_offset=np.array([5, 0.2, 0.3]), euler_start=np.array([0.1, 1, 0.0001]))
    # data["data_s2"] = generate_data_s2()
    # gauss = Gaussian.mle_from_data(ProductManifold.from_string("R1*R3*SO3"), data["data"])
    # data["mu"] = gauss.mu
    # data["sigma"] = gauss.sigma
    # with open("tests/model/test_gaussian.pickle", "wb") as f:
    #    pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)


@pytest.fixture()
def test_gmm_data():
    # Same random seed = same result
    np.random.seed(4)

    with open("tests/model/test_gaussian_gmm.pickle", "rb") as f:
        data = pickle.load(f)

    # uncomment to generate new data
    # a = generate_data(num=500)
    # b = generate_data(num=500, s_offset=0.2, pos_offset=np.array([1, 0, 0]))
    # c = generate_data(s_offset=0.4, pos_offset=np.array([2, 0, 0]))
    # d = generate_data(s_offset=0.6, pos_offset=np.array([3, 0, 0]))
    # e = generate_data(num=700, s_offset=0.8, pos_offset=np.array([4, 0, 0]))
    # data = {}
    # data["data"] = np.concatenate((a, b, c, d, e), axis=0)
    # data["gmm_eucl"], _, __ = GMM.fit(EuclideanManifold(3), 5, data["data"][:, 1:4])
    # data["gmm_man"], _, __ = GMM.fit(
    #    ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), 5, data["data"]
    # )
    # with open("tests/model/test_gaussian_gmm.pickle", "wb") as f:
    #    pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

    return data
