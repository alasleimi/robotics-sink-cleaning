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
GMM implementation.

:cite:t:`zeestraten2017manifold`
:cite:t:`calinon2020gaussians`.
"""

from typing import TYPE_CHECKING, Any

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.manif.util import frechet_mean
from pglfd.model.gaussian import Gaussian
from pglfd.model.kmeans import KMeans

try:
    from pglfd_cpp.gmm_wrapper import fit_cpp_gmm, get_cpp_gmm

    have_cpp = True
except ImportError:
    have_cpp = False

if TYPE_CHECKING:
    from pglfd.model.gmr import GMR


class GMM:
    """Gaussian Mixture Model on a manifold, inspired by riepybdlib."""

    def __new__(cls, gaussians: list[Gaussian], priors: np.ndarray | None = None) -> Any:
        """
        Initialize the GMM.

        :param gaussians: The gaussians this GMM consists of
        :param priors: The priors.
        """
        if have_cpp and gaussians is not None:
            gmm_cpp = get_cpp_gmm(gaussians[0].manifold, gaussians, priors)
            if gmm_cpp is not None:
                return gmm_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: gaussians, priors
        """
        return self.gaussians, self.priors

    def __init__(self, gaussians: list[Gaussian], priors: np.ndarray | None = None) -> None:
        """
        Initialize the GMM.

        :param gaussians: The gaussians this GMM consists of
        :param priors: The priors.
        """
        #: The manifold of the data fitted to this KMeans algorithm.
        self.manifold: Manifold = gaussians[0].manifold
        #: The Gaussians this GMM consists of.
        self.gaussians: list[Gaussian] = gaussians
        #: The priors of each Gaussian in the GMM.
        self.priors: np.ndarray = priors if priors is not None else np.ones(len(gaussians)) / len(gaussians)
        #: Indicates whether this GMM is using a C++ implementation.
        self.is_cpp: bool = False

    def __repr__(self) -> str:
        """Information about this GMM."""
        return f"<pglfd.model.gmm.GMM with {len(self.gaussians)} Gaussians on {self.manifold!s}>"

    def tangent_action(self, A: np.ndarray) -> None:
        """
        Rotate our tangent space by A (pure rotation of the tangent space at the origin, no translation on the manifold).

        :param A: Rotation to perform in tangent space.
        """
        for gaussian in self.gaussians:
            gaussian.tangent_action(A)

    def action(self, trafo: np.ndarray) -> None:
        """
        Perform the action function on the Gaussian (pure translation, no rotation of the tangent space).

        :param trafo: `(dim_man)` Transformation to apply.
        """
        for gaussian in self.gaussians:
            gaussian.action(trafo)

    def expectation(self, data: np.ndarray) -> np.ndarray:
        """
        Calculate the expectation for data points.

        :param data: `(N, dim_man)` The datapoints.
        :return: `(N, K)` The resulting likelihood where K = number of gaussians.
        """
        # Return shape
        ret_shape = list(data.shape)
        ret_shape[-1] = len(self.gaussians)

        # Calculate likelihood
        data = data.reshape(-1, self.gaussians[0].manifold.dim_manifold)
        likelihood = np.zeros((data.shape[0], len(self.gaussians)))
        for i in range(len(self.gaussians)):
            likelihood[:, i] = self.gaussians[i].probability(data) * self.priors[i]
        return likelihood.reshape(tuple(ret_shape))

    def sample(self, n: int) -> np.ndarray:
        """
        Draw n samples from the GMM.

        :param n: The number of samples to draw.
        :return: `(n, dim_man)` The samples.
        """
        # Draw from the priors and reorganize in a list of n elements
        # with the id of the Gaussian to sample from
        rng = np.random.default_rng()
        nums = rng.multinomial(n, self.priors)
        pis = [i for i in range(len(self.priors)) for _ in range(nums[i])]

        # Preallocate the result
        res = np.zeros((n, self.gaussians[0].manifold.dim_manifold))

        # Calculate the results
        for i, pi in zip(range(len(pis)), pis, strict=True):
            res[i, :] = self.gaussians[pi].sample(1)

        return res

    def margin(self, idx_man: list) -> "GMM":
        """
        Compute the marginal distribution.

        :param idx_man: `(N)` list of input dimensions.
        :return: The marginal distribution of the submanifold GMM.
        """
        marg_gaussians = [gauss.margin(idx_man) for gauss in self.gaussians]

        return GMM(marg_gaussians, self.priors)

    def gmr(self, idx_in: list, idx_out: list) -> "GMR":
        """
        Create a Gaussian Mixture Regression class.

        :param idx_in: List of input manifold idx's.
        :param idx_out: List of output manifold idx's.
        :return: GMR which can be queried with input values.
        """
        from pglfd.model.gmr import GMR

        return GMR(self, idx_in, idx_out)

    @classmethod
    def fit(
        cls,
        manifold: Manifold,
        k: int,
        data: np.ndarray,
        init_algorithm: str = "kmeans",
        eps: float = 1e-5,
        maxit: int = 100,
    ) -> tuple["GMM", np.ndarray, list]:
        """
        Fit the GMM using some initially fitted gaussians (e.g. using k-means) and some data.

        :param manifold: The manifold to use.
        :param k: The number of components to fit.
        :param data: The data to fit.
        :param init_algorithm: The initialization algorithm to use. Currently only "kmeans".
        :param eps: Stopping criterion.
        :param maxit: Maximum number of iterations to perform before exiting.
        :return: tuple The fitted GMM, likelihood values for the data, list of average likelihoods during optimization.
        """
        # Fit C++ GMM if it exists
        if have_cpp:
            gmm_cpp = fit_cpp_gmm(manifold, k, data, init_algorithm, eps, maxit)
            if gmm_cpp is not None:
                return gmm_cpp

        # Initial clustering
        if init_algorithm == "kmeans":
            kmeans = KMeans.fit(manifold, k, data)
            priors = np.array([kmeans.clusters[i].shape[0] / data.shape[0] for i in range(k)])
            gaussians = [Gaussian.mle_from_data(manifold, kmeans.clusters[i]) for i in range(k)]
        else:
            msg = f"Do not support init algorithm {init_algorithm}"
            raise ValueError(msg)

        # Initialize GMM
        gmm = GMM(gaussians, priors=priors)

        # Optimization loop
        it = 0
        cur_likelihood = 2 * eps
        prev_likelihood = 0.0
        avg_loglikelihood = []
        while abs(cur_likelihood - prev_likelihood) > eps:
            # Storage
            prev_likelihood = cur_likelihood

            # Expectation Step
            # Variable names after https://www.ics.uci.edu/~smyth/courses/cs274/notes/EMnotes.pdf
            likelihood = gmm.expectation(data)
            w = likelihood / (np.sum(likelihood, axis=1, keepdims=True) + 1e-200)

            # Maximization Step
            # - update gaussian
            for i in range(len(gmm.gaussians)):
                gmm.gaussians[i] = Gaussian.mle_from_data(manifold, data, w[:, i])

            # - update priors
            gmm.priors = np.sum(w, axis=0)
            gmm.priors /= np.sum(gmm.priors)

            # Convergence check
            cur_likelihood = -np.log(np.sum(likelihood, axis=1) + 1e-200).mean()
            avg_loglikelihood.append(cur_likelihood)

            # Maximum number of iterations
            it += 1
            if it > maxit:
                msg = f"GMM Expectation Maximization did not converge in {maxit} steps"
                raise RuntimeWarning(msg)

        return gmm, likelihood, avg_loglikelihood

    def approximate_single_gaussian(self, weights: np.ndarray, eps: float = 1e-8, maxit: int = 50) -> Gaussian:
        """
        Approximate GMM with a single gaussian using moment matching :cite:p:`muehlbauer2024probabilistic`.

        :param weights: `(N)` weight for each gaussian.
        :param eps: Stopping criterion.
        :param maxit: Maximum number of iterations to perform before exiting.
        :return: `(N, dim_man)` or `(dim_man)` The calculated output.
        :return: The resulting gaussian.
        """
        # Normalize weights
        weight_sum = np.sum(weights)
        if weight_sum > 0:
            weights /= weight_sum
        else:
            weights[:] = 1 / weights.shape[0]

        # Extract means
        mus = np.zeros((len(self.gaussians), self.manifold.dim_manifold))
        for i, gaussian in enumerate(self.gaussians):
            mus[i, :] = gaussian.mu

        # Mean is the weighted average
        mu = frechet_mean(self.manifold, mus, weights, eps=eps, maxit=maxit)

        # Covariance is the weighted average + moments
        sigma = np.zeros((self.manifold.dim_tangent, self.manifold.dim_tangent))
        for i, gaussian in zip(range(len(self.gaussians)), self.gaussians, strict=True):
            logval = gaussian.manifold.log(gaussian.mu.reshape(1, -1), mu).reshape(self.manifold.dim_tangent, 1)
            moment = logval @ logval.T
            sigma_transp = self.manifold.parallel_transport_matrix(gaussian.sigma, gaussian.mu, mu)
            sigma += weights[i] * (sigma_transp + moment)

        return Gaussian(self.manifold, mu, sigma)
