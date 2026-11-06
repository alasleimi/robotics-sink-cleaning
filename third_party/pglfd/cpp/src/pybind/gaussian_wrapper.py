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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Generator

    from pglfd.manif.manifold import Manifold
    from pglfd_cpp.conditioned_gaussian_wrapper import ConditionedGaussianWrapper


class GaussianWrapper:
    def __init__(self, manifold: Manifold, gauss: Any) -> None:
        self.manifold = manifold
        self.gauss = gauss
        self.is_cpp = True

    @property
    def mu(self) -> np.ndarray:
        return self.gauss.mu

    @mu.setter
    def mu(self, new_mu: np.ndarray) -> None:
        self.gauss.mu = new_mu

    @property
    def sigma(self) -> np.ndarray:
        # ensure that even a scalar value is a proper 1x1 matrix
        sigma = self.gauss.sigma
        return sigma.reshape((sigma.shape[-1], sigma.shape[-1]))

    @sigma.setter
    def sigma(self, new_sigma: np.ndarray) -> None:
        self.gauss.sigma = new_sigma

    def __repr__(self) -> str:
        return self.gauss.__repr__()

    def tangent_action(self, A: np.ndarray) -> None:
        self.gauss.tangent_action(A)

    def action(self, trafo: np.ndarray) -> None:
        self.gauss.action(trafo)

    def probability(self, samples: np.ndarray) -> np.ndarray:
        return self.gauss.probability(samples)

    def sample(self, n: int) -> np.ndarray:
        return np.array(self.gauss.sample(n))

    def margin(self, idx_man: list) -> GaussianWrapper:
        args = "".join([str(i) for i in idx_man])
        man, _, _ = self.manifold.submanifold(idx_man)
        return GaussianWrapper(man, getattr(self.gauss, f"margin{args}")())

    def condition(self, idx_in: list, idx_out: list) -> ConditionedGaussianWrapper:
        from pglfd_cpp.conditioned_gaussian_wrapper import ConditionedGaussianWrapper

        args_in = "".join([str(i) for i in idx_in])
        args_out = "".join([str(i) for i in idx_out])
        man_in, _, _ = self.manifold.submanifold(idx_in)
        man_out, _, _ = self.manifold.submanifold(idx_out)
        return ConditionedGaussianWrapper(man_in, man_out, getattr(self.gauss, f"condition{args_in}_{args_out}")())


class GaussiansListWrapper:
    def __init__(self, manifold: Manifold, gaussians: Any) -> None:
        self._manifold = manifold
        self._gaussians = gaussians

    def __iter__(self) -> Generator:
        for i in range(len(self)):
            yield self[i]

    def __len__(self) -> int:
        return len(self._gaussians)

    def __getitem__(self, key: int) -> GaussiansListWrapper | GaussianWrapper:
        if isinstance(key, slice):
            return GaussiansListWrapper(self._manifold, self._gaussians[key])
        return GaussianWrapper(self._manifold, self._gaussians[key])

    def __setitem__(self, key: int, value: GaussianWrapper) -> None:
        # this method only supports updating mu and sigma,
        # but not manipulating cpp / other stuff
        self._gaussians[key].mu = value.gauss.mu
        self._gaussians[key].sigma = value.gauss.sigma


def get_cpp_gaussian(manifold: Manifold, mu: np.ndarray, sigma: np.ndarray) -> GaussianWrapper | None:
    """
    Get a gaussian if it exists.

    :param manifold: The Gaussian's manifold.
    :param mu: The Gaussian's mean.
    :param sigma: The Gaussian's covariance.
    :return: The Gaussian object or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        conditioned_gaussian,  # noqa: F401
        gaussian,
        gaussian_regressor,  # noqa: F401
    )

    man = "".join(str(manifold).split("*"))
    name = f"Gaussian{man}"
    if hasattr(gaussian, name):
        obj = getattr(gaussian, name)(mu, sigma)
        return GaussianWrapper(manifold, obj)
    return None


def cpp_gaussian_mle_from_data(
    manifold: Manifold,
    data: np.ndarray,
    weights: np.ndarray | None = None,
    reg: float = 1e-3,
    eps: float = 1e-8,
    maxit: int = 50,
) -> GaussianWrapper | None:
    """
    Fit Gaussian if it exists.

    :param manifold: The manifold of the Gaussian.
    :param data: (N, dim_man) Data to perform MLE on.
    :param weights: (N) Weights for individual data points.
    :param reg: Diagonal regularization.
    :param eps: Stopping criterion.
    :param maxit: Maximum number of iterations to perform before exiting.
    :return: The estimated gaussian.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        conditioned_gaussian,  # noqa: F401
        gaussian,
        gaussian_regressor,  # noqa: F401
    )

    man = "".join(str(manifold).split("*"))
    name = f"Gaussian{man}"
    if hasattr(gaussian, name):
        if weights is None:
            weights = np.array([])
        obj = getattr(gaussian, name).mle_from_data(data, weights, reg, eps, maxit)
        return GaussianWrapper(manifold, obj)
    return None
