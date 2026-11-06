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
Gaussian on manifold implementation.

Literature:

:cite:t:`zeestraten2017manifold`
:cite:t:`calinon2020gaussians`

#. https://calinon.ch/misc/EE613/EE613-nonlinearRegression.pdf
"""

import typing
from typing import Any, cast

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.manif.product import ProductManifold
from pglfd.manif.util import frechet_mean
from pglfd.model.gaussian_regressor import GaussianRegressor

try:
    from pglfd_cpp.gaussian_wrapper import cpp_gaussian_mle_from_data, get_cpp_gaussian

    have_cpp = True
except ImportError:
    have_cpp = False


class Gaussian:
    """Implementation of Gaussian on a manifold [1]."""

    def __new__(cls, manifold: Manifold | ProductManifold, mu: np.ndarray, sigma: np.ndarray) -> Any:
        """
        Initialize the gaussian.

        :param manifold: The manifold to work on.
        :param mu: The mean.
        :param sigma: The covariance.
        """
        # Check inputs
        if len(mu.shape) != 1 or mu.shape[0] != manifold.dim_manifold:
            msg = f"Incorrect mean shape {mu.shape}, expected {manifold.dim_manifold}"
            raise ValueError(msg)
        if sigma.shape != (manifold.dim_tangent, manifold.dim_tangent):
            msg = f"Incorrect covariance shape {sigma.shape}, expected ({manifold.dim_tangent}, {manifold.dim_tangent})"
            raise ValueError(msg)

        if have_cpp and cls == Gaussian:
            gaussian_cpp = get_cpp_gaussian(manifold, mu, sigma)
            if gaussian_cpp is not None:
                return gaussian_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: manifold, mu, sigma
        """
        return self.manifold, self.mu, self.sigma

    def __init__(self, manifold: Manifold, mu: np.ndarray, sigma: np.ndarray) -> None:
        """
        Initialize the gaussian.

        :param manifold: The manifold to work on.
        :param mu: The mean.
        :param sigma: The covariance.
        """
        #: The manifold of the Gaussian.
        self.manifold: Manifold = manifold
        #: The mean value of the Gaussian, in manifold coordinates.
        self.mu: np.ndarray = mu
        #: The covariance of the Gaussian expressed in tangent space.
        self.sigma: np.ndarray = sigma
        #: Indicates whether this Gaussian is using a C++ implementation.
        self.is_cpp: bool = False

    def __repr__(self) -> str:
        """Return type of Gaussian."""
        return f"<pglfd.model.gaussian.Gaussian at {self.mu}>"

    def tangent_action(self, A: np.ndarray) -> None:
        """
        Rotate our tangent space by A (pure rotation of the tangent space at the origin, no translation on the manifold).

        This implements the rotation by A of eq. 26 & eq. 27 in [1].

        :param A: `(dim_tan, dim_tan)` rotation to perform in tangent space.
        """
        self.mu = self.manifold.exp(A @ self.manifold.log(self.mu, self.manifold.id_elem), self.manifold.id_elem)
        self.sigma = A @ self.sigma @ A.T

    def action(self, trafo: np.ndarray) -> None:
        """
        Perform the action function on the Gaussian (pure translation, no rotation of the tangent space).

        :param trafo: `(dim_man)` The transformation to apply to the gaussian.
        """
        man_desc = str(self.manifold)

        if "*" in man_desc and ("S2" in man_desc or "SO2" in man_desc):
            msg = f"Moving a Gaussian on {man_desc} is not implemented as it does not make sense."
            raise NotImplementedError(msg)

        self.mu = self.manifold.action(self.mu, self.manifold.id_elem, trafo)

        # Modify the covariance if required
        if "S2" in man_desc:
            # TODO maybe we could define a "new" manifold function when it's needed,
            # e.g. for S2
            # TODO really only apply to manifolds where it's needed
            self.sigma = self.manifold.parallel_transport_matrix(
                self.sigma, trafo, self.mu
            )  # transport sigma from origin -> new mu

    @classmethod
    def weighted_covariance_from_data(
        cls, manifold: Manifold, mu: np.ndarray, data: np.ndarray, weights: np.ndarray, reg: float = 1e-8
    ) -> np.ndarray:
        """
        Calculate the covariance of data.

        :param manifold: The manifold of the Gaussian.
        :param mu: The mean value of the data.
        :param data: `(N, dim_man)` Data to compute the covariance.
        :param weights: `(N)` Weights for individual data points. Have to sum to 1.
        :param reg: Diagonal regularization.
        :return: `(dim_man, dim_man)` The covariance.
        """
        logval = manifold.log(data, mu)
        # The covariance is the elementwise sum of the outer product log @ log.T, each entry weighted by the weights.
        # As dim(logval) = N, dim_tangent, we can write it more efficiently as follows:
        return logval.T @ np.diag(weights) @ logval + reg * np.eye(manifold.dim_tangent)

    @classmethod
    def mle_from_data(
        cls,
        manifold: Manifold,
        data: np.ndarray,
        weights: np.ndarray | None = None,
        reg: float = 1e-8,
        eps: float = 1e-8,
        maxit: int = 50,
    ) -> "Gaussian":
        """
        Perform Maximum Likelihood Estimation on the manifold for getting a gaussian.

        :param manifold: The manifold of the Gaussian.
        :param data: `(N, dim_man)` Data to perform MLE on.
        :param weights: `(N)` Weights for individual data points.
        :param reg: Diagonal regularization.
        :param eps: Stopping criterion.
        :param maxit: Maximum number of iterations to perform before exiting.
        :return: The estimated gaussian.
        """
        # Fit C++ Gaussian if it exists
        if have_cpp:
            gaussian_cpp = cpp_gaussian_mle_from_data(manifold, data, weights, reg, eps, maxit)
            if gaussian_cpp is not None:
                return gaussian_cpp

        # Check weights input
        if weights is None:
            weights = np.ones(data.shape[0])
        weights = weights.copy() / np.sum(weights)  # normalize to also "fix" unnormalized input

        # Calculate mean and covariance
        mu = frechet_mean(manifold, data, weights, eps, maxit)
        sigma = Gaussian.weighted_covariance_from_data(manifold, mu, data, weights, reg)

        # Construct & return gaussian
        return Gaussian(manifold, mu, sigma)

    def probability(self, samples: np.ndarray) -> np.ndarray:
        """
        Calculate the probability density function of individual samples.

        :param samples: `(N, dim_man)` or `(dim_man)` the samples to evaluate
        :return: The probability density function evaluated at samples.
        """
        samples = samples.reshape(-1, self.manifold.dim_manifold)

        # Calculate distance
        logval = self.manifold.log(samples, self.mu)
        sigma_solve = self.sigma.reshape(-1, self.manifold.dim_tangent, self.manifold.dim_tangent)  # reshape sigma
        dist = logval.reshape(-1, 1, self.manifold.dim_tangent) @ np.linalg.solve(sigma_solve, logval).reshape(
            -1, self.manifold.dim_tangent, 1
        )  # logval.T @ Sigma^-1 @ logval

        return np.exp(-0.5 * dist).reshape(-1) / (
            np.sqrt((2 * np.pi) ** self.manifold.dim_tangent * np.linalg.det(self.sigma)) + 1e-200
        )

    def sample(self, n: int) -> np.ndarray:
        """
        Draw n samples from the Gaussian distribution.

        :param n: The number of samples to draw.
        :return: `(N, dim_man)` The samples
        """
        # Sample in tangent space
        samples = np.random.default_rng().multivariate_normal(np.zeros((self.manifold.dim_tangent,)), self.sigma, n)

        return self.manifold.exp(samples, self.mu)

    def margin(self, idx_man: list) -> "Gaussian":
        """
        Compute the marginal distribution.

        :param idx_man: `(N)` list of input dimensions.
        :return: The marginal distribution of the submanifold.
        """
        if type(self.manifold).__name__ != "ProductManifold":
            msg = ".margin only works for product manifolds!"
            raise TypeError(msg)
        submanifold, idx_manifold, idx_tangent = cast(ProductManifold, self.manifold).submanifold(idx_man)

        mu = self.mu[idx_manifold]
        sigma = self.sigma[np.ix_(idx_tangent, idx_tangent)]

        return Gaussian(submanifold, mu, sigma)

    def condition(self, idx_in: list, idx_out: list) -> "ConditionedGaussian":
        """
        Gaussian conditioning.

        Returns a ConditionedGaussian which can be called with the concrete input values.

        :param idx_in: `(N)` list of input dimensions.
        :param idx_out: `(N)` list of output dimensions.
        :return: The ConditionedGaussian.
        """
        if type(self.manifold).__name__ != "ProductManifold":
            msg = ".condition only works for product manifolds!"
            raise TypeError(msg)
        return ConditionedGaussian(self, idx_in, idx_out)


def gaussian_product(*args: Gaussian, eps: float = 1e-8, maxit: int = 50) -> Gaussian:
    """
    Calculate the gaussian product over gaussians.

    :param eps: Threshold parameter.
    :param maxit: Maximum number of iterations
    :return: The resulting gaussian.
    """
    mu = args[0].mu
    sigma = np.eye(args[0].manifold.dim_tangent)

    # prepare inverse matrices
    sigma_invs = np.zeros((len(args), args[0].manifold.dim_tangent, args[0].manifold.dim_tangent))
    for i, gaussian in zip(range(len(args)), args, strict=True):
        sigma_invs[i] = np.linalg.inv(gaussian.sigma)

    # loop
    it = 0
    diff = 1.0
    while diff > eps:
        # transport covariances
        sigma_transp = np.zeros((len(args), args[0].manifold.dim_tangent, args[0].manifold.dim_tangent))
        for i, gaussian in zip(range(len(args)), args, strict=True):
            sigma_transp[i] = gaussian.manifold.parallel_transport_matrix(sigma_invs[i], gaussian.mu, mu)

        # Compute new covariance
        sigma = np.linalg.inv(np.sum(sigma_transp, axis=0))

        # Update mu
        delta_inv_sum = np.zeros(args[0].manifold.dim_tangent)
        for i, gaussian in zip(range(len(args)), args, strict=True):
            delta_inv_sum += sigma_transp[i] @ gaussian.manifold.log(gaussian.mu.reshape(1, -1), mu).reshape(-1)
        delta = sigma @ delta_inv_sum
        mu = args[0].manifold.exp(delta.reshape(1, -1), mu).reshape(-1)

        # Check convergence
        diff = (delta.T @ delta).item()
        it += 1
        if it > maxit:
            msg = f"Gaussian product did not converge in {maxit} steps"
            raise RuntimeWarning(msg)

    return Gaussian(args[0].manifold, mu, sigma)


class ConditionedGaussian(GaussianRegressor):
    """Conditioned Gaussian that can be queried for new outputs."""

    def __init__(self, gauss_in: Gaussian, idx_in: list, idx_out: list) -> None:
        """
        Initialize the conditioned Gaussian.

        :param gauss_in: The Gaussian to be conditioned.
        :param idx_in: Submanifolds that will be used as input.
        :param idx_out: Submanifolds that will be used as output.
        """
        # Check for product manifold
        if type(gauss_in.manifold).__name__ != "ProductManifold":
            msg = "ConditionedGaussian only works for product manifolds!"
            raise TypeError(msg)

        # Marginalize full Gaussian to involved manifolds
        self.marg_tot = gauss_in.margin(idx_in + idx_out)

        # Marginalize input and output
        self.marg_in = gauss_in.margin(idx_in)
        self.marg_out = gauss_in.margin(idx_out)

        # Input and output tangent idx
        self.manifold_in, _idx_manifold_in, idx_tangent_in = cast(ProductManifold, gauss_in.manifold).submanifold(idx_in)
        self.manifold_out, _idx_manifold_out, idx_tangent_out = cast(ProductManifold, gauss_in.manifold).submanifold(idx_out)

        # Calculate lambda
        self.sigma_ii = gauss_in.sigma[np.ix_(idx_tangent_in, idx_tangent_in)]
        self.sigma_oi = gauss_in.sigma[np.ix_(idx_tangent_out, idx_tangent_in)]
        self.sigma_oo = gauss_in.sigma[np.ix_(idx_tangent_out, idx_tangent_out)]

        # Calculate inverses
        self.sigma_ii_inv = np.linalg.pinv(self.sigma_ii)

    @property
    @typing.override
    def input_manifold(self) -> Manifold:
        return self.manifold_in

    @property
    @typing.override
    def output_manifold(self) -> Manifold:
        return self.manifold_out

    def __call__(self, x: np.ndarray, eps: float = 1e-8, maxit: int = 50) -> Gaussian | list[Gaussian]:
        """
        Calculate the conditioned Gaussian for input x. The implementation follows [2] and [3] (slide 8).

        :param x: Input vectors.
        :param eps: Threshold parameter.
        :param maxit: Maximum number of iterations.
        :return: Gaussians resulting from the conditioning operation.
        """
        # reshape
        shape_in = x.shape
        x = x.reshape(-1, self.marg_in.manifold.dim_manifold)

        res = []
        for x_i in x:
            # Prepare mu and log of the input
            mu = self.marg_out.mu
            log_x_in = self.marg_in.manifold.log(x_i, self.marg_in.mu)

            # Loop variables
            it = 0
            diff = 1
            while diff > eps:
                # Get parallel transport
                R_out = self.marg_out.manifold.parallel_transport_trafo(mu, self.marg_out.mu)

                # Calculate update
                delta = (
                    -R_out.T @ self.marg_out.manifold.log(mu, self.marg_out.mu)
                    + R_out.T @ self.sigma_oi @ self.sigma_ii_inv @ log_x_in
                )
                mu = self.marg_out.manifold.exp(delta, mu)

                # Check convergence
                diff = delta.T @ delta
                it += 1
                if it > maxit:
                    msg = f"Gaussian conditioning did not converge in {maxit} steps, diff: {diff}, eps: {eps}"
                    raise RuntimeWarning(msg)

            # Calculate sigma
            R_out = self.marg_out.manifold.parallel_transport_trafo(mu, self.marg_out.mu)
            sigma = R_out.T @ (self.sigma_oo - self.sigma_oi @ self.sigma_ii_inv @ self.sigma_oi.T) @ R_out

            # Append
            res.append(Gaussian(self.marg_out.manifold, mu, sigma))

        # reshape
        if len(shape_in) <= 1 and len(res) == 1:
            return res[0]
        return res
