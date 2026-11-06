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

import typing

import numpy as np

from pglfd.geom.so2 import exp, inverse, log, multiply, parallel_transport
from pglfd.manif.manifold import Manifold


class SO2Manifold(Manifold):
    """The SO2 manifold."""

    def __init__(self) -> None:
        """Initialize the SO2 manifold."""

    @typing.override
    def __repr__(self) -> str:
        return "SO2"

    @property
    @typing.override
    def dim_manifold(self) -> int:
        return 2

    @property
    @typing.override
    def dim_tangent(self) -> int:
        return 1

    @property
    @typing.override
    def id_elem(self) -> np.ndarray:
        return np.array([0, 1])

    @typing.override
    def exp(self, tangents: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        return multiply(base, exp(tangents))

    @typing.override
    def log(self, points: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        return log(multiply(inverse(base), points))

    @typing.override
    def continuous_log(
        self, logval: np.ndarray, vias: dict | None = None, *, start_low: bool = True
    ) -> tuple[np.ndarray, list]:
        add_glob = 0.0
        for i in range(1, logval.shape[0]):
            logval[i] += add_glob
            if abs(logval[i] + 2 * np.pi - logval[i - 1]) < abs(logval[i] - logval[i - 1]):
                logval[i] += 2 * np.pi
                add_glob += 2 * np.pi
            elif abs(logval[i] - 2 * np.pi - logval[i - 1]) < abs(logval[i] - logval[i - 1]):
                logval[i] -= 2 * np.pi
                add_glob -= 2 * np.pi
        return logval, []

    @typing.override
    def action(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        return multiply(multiply(h, inverse(g)), elems)

    @typing.override
    def parallel_transport_trafo(self, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        return parallel_transport(g, h)

    @typing.override
    def parallel_transport_vector(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        ret_shape = elems.shape
        rotmat = self.parallel_transport_trafo(g, h).reshape(-1, self.dim_tangent, self.dim_tangent)
        return (rotmat @ elems.reshape(-1, self.dim_tangent, 1)).reshape(ret_shape)

    @typing.override
    def parallel_transport_matrix(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        ret_shape = elems.shape
        rotmat = self.parallel_transport_trafo(g, h).reshape(-1, self.dim_tangent, self.dim_tangent)
        return (rotmat @ elems.reshape(-1, self.dim_tangent, self.dim_tangent) @ rotmat.transpose(0, 2, 1)).reshape(ret_shape)
