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

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np
    from pglfd.manif.manifold import Manifold


def get_cpp_kmeans(manifold: Manifold, clusters: list[np.ndarray]) -> Any | None:
    from pglfd_cpp import (
        kmeans,  # protected import, otherwise might have a partially initialized module
    )

    man = "".join(str(manifold).split("*"))
    name = f"KMeans{man}"
    if hasattr(kmeans, name):
        return getattr(kmeans, name)(clusters)
    return None


def cpp_kmeans_fit(
    manifold: Manifold,
    k: int,
    data: np.ndarray,
    maxit: int,
) -> Any | None:
    from pglfd_cpp import (
        kmeans,  # protected import, otherwise might have a partially initialized module
    )

    man = "".join(str(manifold).split("*"))
    name = f"KMeans{man}"
    if hasattr(kmeans, name):
        return getattr(kmeans, name).fit(k, data, maxit)
    return None
