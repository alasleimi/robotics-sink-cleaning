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

"""
KMeans model on a manifold.

:cite:t:`zeestraten2017manifold`
:cite:t:`calinon2020gaussians`
"""

import random
from typing import Any

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.manif.util import frechet_mean

try:
    from pglfd_cpp.kmeans_wrapper import cpp_kmeans_fit, get_cpp_kmeans

    have_cpp = True
except ImportError:
    have_cpp = False


class KMeans:
    """KMeans model on a manifold, inspired by riepybdlib. Please use the ``fit()`` method to fit data."""

    def __new__(cls, manifold: Manifold, clusters: list[np.ndarray]) -> Any:
        """
        Initialize the KMeans object.

        :param manifold: The manifold to work on.
        :param clusters: The clustered data.
        """
        if have_cpp and cls == KMeans:
            kmeans = get_cpp_kmeans(manifold, clusters)
            if kmeans is not None:
                return kmeans
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: manifold, clusters
        """
        return self.manifold, self.clusters

    def __init__(self, manifold: Manifold, clusters: list[np.ndarray]) -> None:
        """
        Initialize KMeans clustering representation.

        :param manifold: The manifold to work on.
        :param clusters: The clustered data.
        """
        #: The manifold of the data fitted to this KMeans algorithm.
        self.manifold: Manifold = manifold
        #: The number of clusters.
        self.k: int = len(clusters)
        #: Clusters containing the original data.
        self.clusters: list[np.ndarray] = clusters
        #: The cluster centers.
        self.cluster_centers = np.array([frechet_mean(manifold, clusters[i]) for i in range(self.k)])
        #: Indicates whether this KMeans is using a C++ implementation.
        self.is_cpp: bool = False

    @classmethod
    def fit(cls, manifold: Manifold, k: int, data: np.ndarray, maxit: int = 100) -> "KMeans":
        """
        Fit `data` on `manifold` using the K-means algorithm into `k` clusters.

        :param manifold: The manifold to work on.
        :param k: The number of clusters.
        :param data: The data to cluster.
        :param maxit: Maximum number of steps to perform.
        :return: Fitted kmeans.
        """
        # Fit C++ kmeans if it exists
        if have_cpp:
            kmeans = cpp_kmeans_fit(manifold, k, data, maxit)
            if kmeans is not None:
                return kmeans

        # Initialize means
        rng = np.random.default_rng(seed=42)
        data_shuffled = rng.permutation(data)
        means = np.copy(data_shuffled[:k])

        id_old = np.zeros(data.shape[0]) + k
        it = 0
        while True:
            # Calculate distances to cluster centers
            dist = np.zeros((k, data.shape[0]))
            for i in range(k):
                dist[i, :] = (manifold.log(data, means[i]) ** 2).sum(axis=1)
            id_min = np.argmin(dist, axis=0)

            # Update cluster centers
            for i in range(k):
                if np.sum(id_min == i):
                    means[i] = frechet_mean(manifold, data[id_min == i])
                else:
                    print("[WARNING] k-means replaced empty cluster by random value")
                    means[i] = data[random.randint(0, data.shape[0] - 1)]

            # Check for convergence
            if (id_min == id_old).all():
                break
            id_old = id_min.copy()

            # Iteration counter
            it += 1
            if it > maxit:
                msg = f"K-Means did not converge after {it} steps"
                raise RuntimeWarning(msg)

        clusters = [data[id_min == i] for i in range(k)]

        return KMeans(manifold, clusters)
