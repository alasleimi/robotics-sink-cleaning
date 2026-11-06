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

from pglfd.manif.manifold import Manifold


def frechet_mean(
    manifold: Manifold, data: np.ndarray, weights: np.ndarray | None = None, eps: float = 1e-10, maxit: int = 50
) -> np.ndarray:
    """
    Calculate the frechet mean of data.

    :param manifold: The manifold of the data.
    :param data: `(N, dim_man)`` Data to compute the frechet mean of.
    :param weights: `(N)` Weights for individual data points, have to sum to 1.
    :param eps: Stopping criterion.
    :param maxit: Maximum number of iterations to perform before exiting.
    :return: `(dim_man)` The mean value.
    """
    if weights is None:
        weights = np.ones((data.shape[0], 1)) * (1.0 / data.shape[0])

    mu = data[np.argmax(weights)]
    diff = 1
    it = 0
    while diff > eps:
        weighted_distance = weights.T @ manifold.log(data, mu)  # (1 x N) @ (N, dim_tan) = (1, dim_tan) vector of distances
        mu = manifold.exp(weighted_distance.reshape(1, -1), mu).reshape(-1)
        diff = weighted_distance @ weighted_distance.T  # (1 x N) @ (N x 1)

        it += 1
        if it > maxit:
            msg = f"Weighted averaging did not converge in {maxit} steps"
            raise RuntimeWarning(msg)
    return mu
