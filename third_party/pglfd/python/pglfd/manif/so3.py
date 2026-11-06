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

from pglfd.geom.so3 import exp, log, parallel_transport, quaternion_inverse, quaternion_multiply
from pglfd.manif.manifold import Manifold


class SO3Manifold(Manifold):
    """The SO(3) manifold. Note that we assume quaternions instead of rotation matrices for the representation."""

    def __init__(self) -> None:
        """Initialize the SO(3) manifold."""

    @typing.override
    def __repr__(self) -> str:
        return "SO3"

    @property
    @typing.override
    def dim_manifold(self) -> int:
        return 4

    @property
    @typing.override
    def dim_tangent(self) -> int:
        return 3

    @property
    @typing.override
    def id_elem(self) -> np.ndarray:
        return np.array([0, 0, 0, 1])

    @typing.override
    def exp(self, tangents: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        return quaternion_multiply(base, exp(tangents, eps=eps))

    @typing.override
    def log(self, points: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        return log(quaternion_multiply(quaternion_inverse(base), points), eps=eps)

    @typing.override
    def continuous_log(  # noqa: C901, PLR0912
        self, logval: np.ndarray, vias: dict | None = None, *, start_low: bool = True
    ) -> tuple[np.ndarray, list]:
        # handle vias
        first_via = logval.shape[0]
        if vias is not None and len(vias.keys()) > 0:
            first_via = min(vias.keys())

            # check vias
            for i in range(first_via, logval.shape[0]):
                if i not in vias:
                    msg = "Got normal point after a via point!"
                    raise ValueError(msg)

        discontinuities = []
        for i in range(3):
            discont = False
            for j in range(1, first_via):
                testval = logval[j, i] + 2 * np.pi
                if logval[j, i] > 0:
                    testval = logval[j, i] - 2 * np.pi
                if abs(testval - logval[j - 1, i]) < abs(logval[j, i] - logval[j - 1, i]):
                    discont = True

            if discont:
                if start_low and logval[0, i] > 0:
                    logval[0, i] -= 2 * np.pi
                elif not start_low and logval[0, i] < 0:
                    logval[0, i] += 2 * np.pi
                for j in range(1, first_via):
                    oldval = logval[j, i]
                    testval = logval[j, i] + 2 * np.pi
                    if logval[j, i] > 0:
                        testval = logval[j, i] - 2 * np.pi
                    if abs(testval - logval[j - 1, i]) < abs(logval[j, i] - logval[j - 1, i]):
                        logval[j, i] = testval

                    testval2 = logval[j - 1, i] + 2 * np.pi
                    if logval[j - 1, i] > 0:
                        testval2 = logval[j - 1, i] - 2 * np.pi
                    if abs(testval2 - logval[j, i]) < abs(logval[j, i] - logval[j - 1, i]):
                        discontinuities.append(j)
                        if abs(oldval - logval[j - 1, i]) < abs(testval - logval[j - 1, i]):
                            logval[j, i] = testval
                        else:
                            logval[j, i] = oldval

            # handle vias, move them close to their closest point
            if vias is not None:
                for j in vias:
                    testval = logval[j, i] + 2 * np.pi
                    if logval[j, i] > 0:
                        testval = logval[j, i] - 2 * np.pi
                    if abs(testval - logval[vias[j], i]) < abs(logval[j, i] - logval[vias[j], i]):
                        logval[j, i] = testval

        return logval, discontinuities

    @typing.override
    def action(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        return quaternion_multiply(quaternion_multiply(h, quaternion_inverse(g)), elems)

    @typing.override
    def parallel_transport_trafo(self, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        return parallel_transport(g, h).T

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
