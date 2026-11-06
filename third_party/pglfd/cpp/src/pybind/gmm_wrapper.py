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

import inspect
from typing import TYPE_CHECKING, Any

import numpy as np
from pglfd_cpp.gaussian_wrapper import GaussiansListWrapper, GaussianWrapper, get_cpp_gaussian
from pglfd_cpp.gmr_wrapper import GMRWrapper

if TYPE_CHECKING:
    from pglfd.manif.manifold import Manifold


class GMMWrapper:
    def __init__(self, manifold: Manifold, gmm: Any) -> None:
        self.manifold = manifold
        self.gmm = gmm
        self.is_cpp = True

    @property
    def gaussians(self) -> GaussiansListWrapper:
        return GaussiansListWrapper(self.manifold, self.gmm.gaussians)

    @property
    def priors(self) -> np.ndarray:
        return self.gmm.priors

    @priors.setter
    def priors(self, priors: np.ndarray) -> None:
        self.gmm.priors = priors

    def tangent_action(self, A: np.ndarray) -> None:
        self.gmm.tangent_action(A)

    def action(self, trafo: np.ndarray) -> None:
        self.gmm.action(trafo)

    def expectation(self, samples: np.ndarray) -> np.ndarray:
        return self.gmm.expectation(samples)

    def margin(self, idx_man: list) -> GMMWrapper:
        args = "".join([str(i) for i in idx_man])
        man, _, _ = self.manifold.submanifold(idx_man)
        return GMMWrapper(man, getattr(self.gmm, f"margin{args}")())

    def sample(self, n: int) -> np.ndarray:
        return np.array(self.gmm.sample(n))

    def gmr(self, idx_in: list, idx_out: list) -> GMRWrapper:
        args_in = "".join([str(i) for i in idx_in])
        args_out = "".join([str(i) for i in idx_out])
        man_in, _, _ = self.manifold.submanifold(idx_in)
        man_out, _, _ = self.manifold.submanifold(idx_out)
        return GMRWrapper(man_in, man_out, getattr(self.gmm, f"gmr{args_in}_{args_out}")())

    def approximate_single_gaussian(self, weights: list, eps: float = 1e-8, maxit: int = 50) -> GaussianWrapper:
        return GaussianWrapper(self.manifold, self.gmm.approximate_single_gaussian(weights, eps, maxit))


def get_cpp_gmm(manifold: Manifold, gaussians: Any, priors: np.ndarray | None = None) -> GMMWrapper | None:
    """
    Get a gaussian if it exists.

    :param manifold: The Gaussian's manifold.
    :param mu: The Gaussian's mean.
    :param sigma: The Gaussian's covariance.
    :return: The Gaussian object or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        conditioned_gaussian,  # noqa: F401
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        gmm,
        gmr,  # noqa: F401
    )

    man = "".join(str(manifold).split("*"))
    name = f"GMM{man}"
    if hasattr(gmm, name):
        # prepare gaussians
        if "pglfd_cpp" not in str(inspect.getmodule(gaussians[0])):
            gaussians = [get_cpp_gaussian(gaussian.manifold, gaussian.mu, gaussian.sigma) for gaussian in gaussians]
        if priors is None:
            priors = np.array([])
        obj = getattr(gmm, name)([gaussian.gauss for gaussian in gaussians], priors)
        return GMMWrapper(manifold, obj)
    return None


def fit_cpp_gmm(
    manifold: Manifold, k: int, data: np.ndarray, init_algorithm: str = "kmeans", eps: float = 1e-5, maxit: int = 100
) -> tuple[GMMWrapper, np.ndarray, list] | None:
    """
    Fit the GMM using some initially fitted gaussians (e.g. using k-means) and some data if it exists.

    :param manifold: The manifold to use.
    :param k: The number of components to fit.
    :param data: The data to fit.
    :param init_algorithm: The initialization algorithm to use. Currently only "kmeans".
    :param eps: Stopping criterion.
    :param maxit: Maximum number of iterations to perform before exiting.
    :return: tuple The fitted GMM, likelihood values for the data, list of average likelihoods during optimization or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        conditioned_gaussian,  # noqa: F401
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        gmm,
        gmr,  # noqa: F401
    )

    man = "".join(str(manifold).split("*"))
    name = f"GMM{man}"
    if hasattr(gmm, name):
        obj = getattr(gmm, name).fit(k, data, init_algorithm, eps, maxit)
        return GMMWrapper(manifold, obj[0]), obj[1], obj[2]
    return None
