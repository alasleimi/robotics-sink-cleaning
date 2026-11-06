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
Kernels on Riemannian Manifolds.

TODO add more docs
"""

import math
import typing
from abc import ABC, abstractmethod

import numpy as np

from pglfd.manif.manifold import Manifold
from pglfd.manif.product import ProductManifold


class Kernel(ABC):
    """Abstract base class for a Kernel."""

    @property
    @abstractmethod
    def is_vectorfield(self) -> bool:
        """Whether this kernel returns a vector field."""

    @abstractmethod
    def __call__(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        """
        Query the kernel for the distance between ``x1`` and ``x2``.

        :param x1: `(N, dim_man)`  First input.
        :param x2: `(N, dim_man)` Second input.
        :return: `(N)` Distance between ``x1`` and ``x2``.
        """


class RBFKernel(Kernel):
    """Radial basis function kernel."""

    def __init__(self, manifold: Manifold, length_scale: float | np.ndarray = 0.1, height_scale: float = 1.0) -> None:
        """
        Initialize the radial basis function.

        :param manifold: The manifold of this kernel.
        :param length_scale: Length scale of the kernel.
        :param height_scale: Maximum value of th kernel.
        """
        self.manifold = manifold
        self.length_scale = length_scale  # save for c++ access
        self.length_scaler = np.ones(self.manifold.dim_tangent) / (2.0 * length_scale * length_scale)
        self.height_scale = height_scale

    @property
    @typing.override
    def is_vectorfield(self) -> bool:
        return False

    @typing.override
    def __call__(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        dist = self.manifold.log(x2, x1).reshape((-1, self.manifold.dim_tangent, 1))
        return self.height_scale * np.exp(
            -dist.transpose((0, 2, 1))
            @ np.diag(self.length_scaler).reshape((1, self.manifold.dim_tangent, self.manifold.dim_tangent))
            @ dist
        ).reshape(-1)


class MaternKernel(Kernel):
    """Matern kernel."""

    def __init__(
        self, manifold: Manifold, length_scale: float | np.ndarray = 0.1, height_scale: float = 1.0, p: int = 2
    ) -> None:
        """
        Initialize the radial basis function.

        :param manifold: The manifold of this kernel.
        :param length_scale: Length scale of the kernel.
        :param height_scale: Maximum value of th kernel.
        :param p: P parameter.
        """
        self.manifold = manifold
        self.p = p
        self.nu = p + 1 / 2
        self.length_scale = length_scale  # save for c++ access
        self.length_scaler = np.ones(self.manifold.dim_tangent) / (length_scale * length_scale)
        self.height_scale = height_scale
        self.gamma_val = math.factorial(p) / math.factorial(2 * p)

    @property
    @typing.override
    def is_vectorfield(self) -> bool:
        return False

    def _polynomial(self, r: np.ndarray) -> float:
        res = 0.0
        for i in range(self.p + 1):
            res += (
                math.factorial(self.p + i)
                / (math.factorial(i) * math.factorial(self.p - i))
                * np.power(np.sqrt(8 * self.nu) * r, self.p - i)
            )
        return res

    @typing.override
    def __call__(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        dist = self.manifold.log(x2, x1).reshape((-1, self.manifold.dim_tangent, 1))
        r = np.sqrt(
            dist.transpose((0, 2, 1))
            @ np.diag(self.length_scaler).reshape((1, self.manifold.dim_tangent, self.manifold.dim_tangent))
            @ dist
        ).reshape(-1)
        return self.height_scale * np.exp(-np.sqrt(2 * self.nu) * r) * self.gamma_val * self._polynomial(r)


class RBFKernelDiagonal(Kernel):
    """RBF Kernel with additional component-wise RBF kernels on the diagonals."""

    def __init__(
        self,
        manifold: Manifold,
        length_scale_rbf: float | np.ndarray = 0.1,
        length_scale_diag: float | np.ndarray = 0.1,
        height_scale: float = 1.0,
    ) -> None:
        """
        Initialize the diagonal radial basis function.

        :param manifold: The manifold of this kernel.
        :param length_scale_rbf: Length scale of the kernel.
        :param length_scale_diagonal: Length scale of the diagonals.
        :param height_scale: Maximum value of th kernel.
        """
        self.manifold = manifold
        self.rbf_kernel = RBFKernel(manifold, length_scale_rbf, height_scale)
        self.length_scaler = np.ones(self.manifold.dim_tangent) / (2.0 * length_scale_diag * length_scale_diag)

        self.length_scale_rbf = length_scale_rbf
        self.length_scale_diag = length_scale_diag

    @property
    @typing.override
    def is_vectorfield(self) -> bool:
        return True

    @typing.override
    def __call__(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        dist = self.manifold.log(x2, x1).reshape((-1, self.manifold.dim_tangent))
        return (
            self.rbf_kernel(x1, x2).reshape((-1, 1, 1))
            * np.exp(-dist * self.length_scaler.reshape((1, -1)) * dist).reshape((-1, 1, self.manifold.dim_tangent))
            * np.eye(self.manifold.dim_tangent).reshape((1, self.manifold.dim_tangent, self.manifold.dim_tangent))
        )


class MaternKernelDiagonal(Kernel):
    """Matern Kernel with additional component-wise Matern kernels on the diagonals."""

    def __init__(
        self,
        manifold: Manifold,
        length_scale_matern: float | np.ndarray = 0.1,
        length_scale_diag: float | np.ndarray = 0.1,
        height_scale: float = 1.0,
        p: int = 2,
    ) -> None:
        """
        Initialize the diagonal radial basis function.

        :param manifold: The manifold of this kernel.
        :param length_scale_matern: Length scale of the kernel.
        :param length_scale_diagonal: Length scale of the diagonals.
        :param height_scale: Maximum value of th kernel.
        :param p: P parameter.
        """
        self.manifold = manifold
        self.matern_kernel = MaternKernel(manifold, length_scale_matern, height_scale, p)

        self.length_scale_matern = length_scale_matern
        self.length_scales_diag = np.ones(self.manifold.dim_tangent) * length_scale_diag

        self.diag_kernels = []
        self.man_diag = ProductManifold.from_string("R1")
        for i in range(self.manifold.dim_tangent):
            self.diag_kernels.append(MaternKernel(self.man_diag, length_scale=self.length_scales_diag[i], p=p))

    @property
    @typing.override
    def is_vectorfield(self) -> bool:
        return True

    @typing.override
    def __call__(self, x1: np.ndarray, x2: np.ndarray) -> np.ndarray:
        dist = self.manifold.log(x2, x1).reshape((-1, self.manifold.dim_tangent))
        for i in range(self.manifold.dim_tangent):
            dist[:, i] = self.diag_kernels[i](self.man_diag.id_elem.reshape((-1, 1)), dist[:, i].reshape((-1, 1)))
        return (
            self.matern_kernel(x1, x2).reshape((-1, 1, 1))
            * dist.reshape((-1, 1, self.manifold.dim_tangent))
            * np.eye(self.manifold.dim_tangent).reshape((1, self.manifold.dim_tangent, self.manifold.dim_tangent))
        )


# TODO: add curl- / divergence-free kernels?
