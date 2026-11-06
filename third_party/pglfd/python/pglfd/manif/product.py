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

from pglfd.manif.euclidean import EuclideanManifold
from pglfd.manif.manifold import Manifold
from pglfd.manif.s2 import S2Manifold
from pglfd.manif.so2 import SO2Manifold
from pglfd.manif.so3 import SO3Manifold


def _flatten_list(list_in: list[list]) -> list:
    return [item for sublist in list_in for item in sublist]


class ProductManifold(Manifold):
    """A manifold resulting from the product of other manifolds."""

    def __init__(self, manifolds: list[Manifold]) -> None:
        """
        Initialize the product manifold from a list of manifolds.

        :param manifolds: The manifolds we want to use in product form.
        """
        self.manifolds = manifolds

    @classmethod
    def from_string(cls, description: str) -> "ProductManifold":
        """
        Create a product manifold from a description string.

        :param description: Manifold description, e.g. "R3*SO3".
        :return: The corresponding product manifold
        """
        manifolds: list[Manifold] = []
        for manifold in description.split("*"):
            manifold_desc = manifold.lower().strip()
            if "so3" in manifold_desc:
                manifolds.append(SO3Manifold())
            elif "s2" in manifold_desc:
                manifolds.append(S2Manifold())
            elif "so2" in manifold_desc:
                manifolds.append(SO2Manifold())
            elif "r" in manifold_desc:
                manifolds.append(EuclideanManifold(int(manifold_desc[1:])))
            else:
                msg = f"Unknown submanifold {manifold_desc}"
                raise ValueError(msg)

        return ProductManifold(manifolds)

    @typing.override
    def __repr__(self) -> str:
        descr = ""
        for manifold in self.manifolds:
            descr += str(manifold) + "*"
        return descr[:-1]

    def submanifold(self, idx_man: list) -> tuple["ProductManifold", np.ndarray, np.ndarray]:
        """
        Get the submanifold.

        :param idx_man: Submanifold elements.
        :return: The submanifold, on-manifold idx and tangent idx.
        """
        idx_manifold = np.array(_flatten_list([self.idx_manifold[i] for i in idx_man]), dtype=np.int64)
        idx_tangent = np.array(_flatten_list([self.idx_tangent[i] for i in idx_man]), dtype=np.int64)
        submanifold = ProductManifold([self.manifolds[i] for i in idx_man])

        return submanifold, idx_manifold, idx_tangent

    @property
    @typing.override
    def dim_manifold(self) -> int:
        return np.sum([manifold.dim_manifold for manifold in self.manifolds])

    @property
    @typing.override
    def dim_tangent(self) -> int:
        return np.sum([manifold.dim_tangent for manifold in self.manifolds])

    @property
    def idx_manifold(self) -> list:
        """
        Get all manifold indices.

        :return: List of arrays containing the indices for all submanifolds.
        """
        idxs = []
        offset = 0
        for manifold in self.manifolds:
            idxs.append(np.array(range(offset, offset + manifold.dim_manifold), dtype=int))
            offset += manifold.dim_manifold
        return idxs

    @property
    def idx_tangent(self) -> list:
        """
        Get all tangent indices.

        :return: List of arrays containing the indices for all submanifolds.
        """
        idxs = []
        offset = 0
        for manifold in self.manifolds:
            idxs.append(np.array(range(offset, offset + manifold.dim_tangent), dtype=int))
            offset += manifold.dim_tangent
        return idxs

    @property
    @typing.override
    def id_elem(self) -> np.ndarray:
        elem = np.zeros(self.dim_manifold)
        for manifold, idx_manifold in zip(self.manifolds, self.idx_manifold, strict=True):
            elem[idx_manifold] = manifold.id_elem
        return elem

    @typing.override
    def exp(self, tangents: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        # check inputs
        if tangents.shape[-1] != self.dim_tangent:
            msg = f"Invalid shape {tangents.shape[-1]} of tangents, expected {self.dim_tangent}"
            raise ValueError(msg)
        if base.shape[-1] != self.dim_manifold:
            msg = f"Invalid shape {base.shape[-1]} of basis, expected {self.dim_manifold}"
            raise ValueError(msg)

        # result shape
        res_shape = list(tangents.shape)
        res_shape[-1] = self.dim_manifold

        # calculate log
        tangents = tangents.reshape(-1, self.dim_tangent)
        res = np.zeros((tangents.shape[0], self.dim_manifold))
        for manifold, idx_manifold, idx_tangent in zip(self.manifolds, self.idx_manifold, self.idx_tangent, strict=True):
            res[:, idx_manifold] = manifold.exp(tangents[:, idx_tangent], base[idx_manifold], eps=eps)
        return res.reshape(res_shape)

    @typing.override
    def log(self, points: np.ndarray, base: np.ndarray, eps: float = 1e-10) -> np.ndarray:
        # check inputs
        if points.shape[-1] != self.dim_manifold:
            msg = f"Invalid shape {points.shape[-1]} of points, expected {self.dim_manifold}"
            raise ValueError(msg)
        if base.shape[-1] != self.dim_manifold:
            msg = f"Invalid shape {base.shape[-1]} of basis, expected {self.dim_manifold}"
            raise ValueError(msg)

        # result shape
        res_shape = list(points.shape) if len(points.shape) >= len(base.shape) else list(base.shape)
        res_shape[-1] = self.dim_tangent

        # reshape inputs
        points = points.reshape(-1, self.dim_manifold)
        base = base.reshape(-1, self.dim_manifold)
        num_pts = max(points.shape[0], base.shape[0])

        # calculate log
        res = np.zeros((num_pts, self.dim_tangent))
        for manifold, idx_manifold, idx_tangent in zip(self.manifolds, self.idx_manifold, self.idx_tangent, strict=True):
            res[:, idx_tangent] = manifold.log(points[:, idx_manifold], base[:, idx_manifold], eps=eps).reshape(
                (-1, manifold.dim_tangent)
            )
        return res.reshape(res_shape)

    @typing.override
    def continuous_log(
        self, logval: np.ndarray, vias: dict | None = None, *, start_low: bool = True
    ) -> tuple[np.ndarray, list]:
        ret_shape = logval.shape
        logval = logval.reshape(-1, self.dim_tangent)
        discontinuities = []
        for manifold, idx_tangent in zip(self.manifolds, self.idx_tangent, strict=True):
            logval[:, idx_tangent], discontinuity = manifold.continuous_log(logval[:, idx_tangent], vias, start_low=start_low)
            discontinuities += discontinuity
        return logval.reshape(ret_shape), list(set(discontinuities))

    @typing.override
    def action(self, elems: np.ndarray, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        # Allocate result
        res_shape = elems.shape
        elems = elems.reshape(-1, self.dim_manifold)
        res = np.zeros(elems.shape)

        # Perform parallel transport
        for manifold, idx_manifold in zip(self.manifolds, self.idx_manifold, strict=True):
            res[:, idx_manifold] = manifold.action(elems[:, idx_manifold], g[idx_manifold], h[idx_manifold])
        return res.reshape(res_shape)

    @typing.override
    def parallel_transport_trafo(self, g: np.ndarray, h: np.ndarray) -> np.ndarray:
        res = np.zeros((self.dim_tangent, self.dim_tangent))
        for manifold, idx_manifold, idx_tangent in zip(self.manifolds, self.idx_manifold, self.idx_tangent, strict=True):
            res[np.ix_(idx_tangent, idx_tangent)] = manifold.parallel_transport_trafo(g[idx_manifold], h[idx_manifold])
        return res

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
