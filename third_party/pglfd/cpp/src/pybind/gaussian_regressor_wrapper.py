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

from typing import Any

import numpy as np
from pglfd.manif.manifold import Manifold
from pglfd_cpp.gaussian_wrapper import GaussiansListWrapper, GaussianWrapper


class GaussianRegressorWrapper:
    def __init__(self, manifold_in: Manifold, manifold_out: Manifold, model: Any) -> None:
        self.manifold_in = manifold_in
        self.manifold_out = manifold_out
        self.model = model
        self.is_cpp = True

    def __repr__(self) -> str:
        return self.model.__repr__()

    @property
    def input_manifold(self) -> Manifold:
        return self.manifold_in

    @property
    def output_manifold(self) -> Manifold:
        return self.manifold_out

    def __call__(self, x: np.ndarray, *args: Any) -> GaussiansListWrapper | GaussianWrapper:
        x_arr = np.array(x).reshape(-1, self.manifold_in.dim_manifold)
        res = GaussiansListWrapper(self.manifold_out, self.model(x_arr, *args))
        if len(x.shape) == 0 or (len(x.shape) == 1 and x.shape[0] == self.manifold_in.dim_manifold):
            return res[0]
        return res
