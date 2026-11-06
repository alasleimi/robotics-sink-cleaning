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

from abc import ABC, abstractmethod

import numpy as np


class Manifold(ABC):
    """Abstract base class defining the required manifold operations and properties.

    Extend this class to implement your custom manifold.
    """

    @property
    @abstractmethod
    def dim_manifold(self) -> int:
        """Get the dimension of a manifold element."""

    @property
    @abstractmethod
    def dim_tangent(self) -> int:
        """Get the dimension of a tangent space element."""

    @property
    @abstractmethod
    def id_elem(self) -> np.ndarray:
        """Get the identity element on the manifold."""

    @abstractmethod
    def exp(self, tangents: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        """
        Exponential map.

        :param tangents: `(N, dim_tan)` or `(dim_tan)` The tangent vector to exponentiate, dim_tan = tangent dimension.
        :param base: `(N, dim_man)` or `(dim_man)` The point on the manifold at which to exponentiate,
                        dim_man = manifold dimension.
        :param eps: (float) Threshold for exponential map.
        :return: `(N, dim_man)` Points on the manifold, dim_tan = manifold dimension.
        """

    @abstractmethod
    def log(self, points: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        """
        Logarithm map.

        :param points: `(N, dim_man)` or `(dim_man)` The point on the manifold to take the logarithm of,
                           dim_tan = tangent dimension.
        :param base: `(N, dim_man)` or `(dim_man)` The point on the manifold at which to take the logarithm,
                        dim_man = manifold dimension.
        :param eps: (float) Threshold for exponential map.
        :return: `(N, dim_tan)` Tangents, dim_tan = tangent dimension.
        """

    @abstractmethod
    def continuous_log(
        self, logval: np.ndarray, vias: dict | None = None, *, start_low: bool = True
    ) -> tuple[np.ndarray, list]:
        """
        Try to move discontinuities away on compact manifolds.

        :param logval: `(N, dim_tan)` Tangents to make continous.
        :param vias: Via points, just order them closest to the other points.
        :param start_low: Start with low (vs high) values for the log.
        :return: Modified log values and locations of discontinuities.
        """

    @abstractmethod
    def action(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        """
        Perform an action on manifold elements: move elems relative to g to elems relative to h.

        :param elems: `(N, dim_man)` The manifold elements to transform, dim_tan = manifold dimension.
        :param g: `(N, dim_man)` or `(dim_man)` Point in the manifold from which to transform, dim_man = manifold dimension.
        :param h: `(N, dim_man)` or `(dim_man)` Point in the manifold to which to transform, dim_man = manifold dimension.
        :return: `(N, dim_man)` Transformed manifold elements, dim_man = manifold dimension.
        """

    @abstractmethod
    def parallel_transport_trafo(self, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        """
        Parallel transport transformation matrix for tangent elements from manifold point g to h.

        :param g: `(dim_man)` The point in the manifold from which to transform, dim_man = manifold dimension.
        :param h: `(dim_man)` The point in the manifold to which to transform, dim_man = manifold dimension.
        :return: (dim_tan, dim_tan)` Transformation matrix, dim_tan = tangent dimension.
        """

    @abstractmethod
    def parallel_transport_vector(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        """
        Parallel transport of tangent vectors from manifold point g to h.

        :param elems: `(dim_tan)` or `(N, dim_tan)` The tangent vectors to transform, dim_tan = tangent dimension.
        :param g: `(dim_man)` The point in the manifold from which to transform, dim_man = manifold dimension.
        :param h: `(dim_man)` The point in the manifold to which to transform, dim_man = manifold dimension.
        :return: `(dim_tan)` or `(N, dim_tan)` Transformed tangent vectors, dim_tan = tangent dimension.
        """

    @abstractmethod
    def parallel_transport_matrix(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        """
        Parallel transport of tangent matrices from manifold point g to h.

        :param elems: (dim_tan, dim_tan)` or `(N, dim_tan, dim_tan)` The tangent matrices to transform, dim_tan = tangent dim.
        :param g: `(dim_man)` The point in the manifold from which to transform, dim_man = manifold dimension.
        :param h: `(dim_man)` The point in the manifold to which to transform, dim_man = manifold dimension.
        :return: (dim_tan, dim_tan)` or `(N, dim_tan, dim_tan)` Transformed tangent matrices, dim_tan = tangent dimension.
        """
