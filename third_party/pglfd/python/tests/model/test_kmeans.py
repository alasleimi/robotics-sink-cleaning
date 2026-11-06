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

import pickle

import numpy as np
import pytest

from pglfd.manif.euclidean import EuclideanManifold
from pglfd.manif.product import ProductManifold
from pglfd.manif.so3 import SO3Manifold
from pglfd.model.kmeans import KMeans
from tests.model.test_data import have_cpp


@pytest.fixture()
def test_file_data():
    with open("tests/model/test_kmeans.pickle", "rb") as f:
        return pickle.load(f)


@pytest.fixture()
def data(test_file_data):
    return test_file_data["data"]


@pytest.fixture()
def clusters(test_file_data):
    return test_file_data["clusters"]


@pytest.fixture()
def cluster_centers(test_file_data):
    return test_file_data["cluster_centers"]


def test_kmeans_fit(data, cluster_centers) -> None:
    kmeans = KMeans.fit(ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), 5, data)
    for i in range(len(cluster_centers)):
        closest_cluster = np.argmin(np.linalg.norm(kmeans.cluster_centers - cluster_centers[i].reshape(-1, 8), axis=1))
        assert np.allclose(kmeans.cluster_centers[closest_cluster], cluster_centers[i], rtol=1e-2, atol=0.2)

    if have_cpp():
        assert kmeans.is_cpp
