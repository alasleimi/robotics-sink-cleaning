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

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np

    from pglfd.manif.manifold import Manifold
    from pglfd.model.gaussian import Gaussian


class GaussianRegressor(ABC):
    """Abstract base class for a Gaussian regressor method."""

    is_cpp = False  #: Indicates whether this GaussianRegressor is using a C++ implementation.

    @property
    @abstractmethod
    def input_manifold(self) -> Manifold:
        """Manifold for the input data."""

    @property
    @abstractmethod
    def output_manifold(self) -> Manifold:
        """Manifold for the output data."""

    @abstractmethod
    def __call__(self, x_query: np.ndarray) -> Gaussian | list[Gaussian]:
        """
        Query the regressor with `x_query` to calculate an output.

        :param x_query: `(D)` or `(N,D)` The query value, may be batched.
        :return: A Gaussian or a list of Gaussians containing the result.
        """
