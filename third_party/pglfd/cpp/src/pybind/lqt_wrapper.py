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

import numpy as np
from pglfd_cpp.gaussian_wrapper import GaussiansListWrapper, GaussianWrapper

if TYPE_CHECKING:
    from pglfd.manif.manifold import Manifold


class LQTWrapper:
    def __init__(self, manifold: Manifold, lqt: Any) -> None:
        self.manifold = manifold
        self.model = lqt
        self.is_cpp = True

    def __repr__(self) -> str:
        return self.model.__repr__()

    def fit(
        self,
        eps: float = 1e-8,
        maxit: int = 50,
    ) -> int:
        return self.model.fit(eps, maxit)

    @property
    def dt(self) -> float:
        return self.model.get_dt()

    @property
    def states(self) -> GaussiansListWrapper:
        return GaussiansListWrapper(self.manifold, self.model.get_states())

    @property
    def poses(self) -> np.ndarray:
        return self.model.get_poses()

    @property
    def velocities(self) -> np.ndarray:
        return self.model.get_velocities()

    @property
    def commands(self) -> np.ndarray:
        return self.model.commands.T

    @commands.setter
    def commands(self, value: np.ndarray) -> None:
        self.model.commands = value.T


def get_cpp_lqt(
    manifold: Manifold,
    initial_state: np.ndarray,
    t_vias: np.ndarray,
    vias: list[GaussianWrapper],
    tend: float,
    dt: float,
    rfactor: float | np.ndarray,
    initial_velocity: np.ndarray | None,
) -> LQTWrapper | None:
    """
    Get the LQT if it exists.

    :param manifold: The output manifold.
    :param initial_state: (dim) The initial state.
    :param t_vias: (N) Times of via points.
    :param vias: (N) Gaussian via points.
    :param tend: End time point.
    :param dt: Time between steps.
    :param rfactor: The r factor.
    :param initial_velocity: Initial velocity.
    :return: A wrapped C++ LQT regressor or None.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        lqt,
    )

    man = "".join(str(manifold).split("*"))

    name_lqt = f"LQT{man}"
    if hasattr(lqt, name_lqt):
        initial_velocity = initial_velocity if initial_velocity is not None else np.zeros(manifold.dim_tangent)
        obj = getattr(lqt, name_lqt)(
            initial_state,
            t_vias,
            [via.gauss for via in vias],
            tend,
            dt,
            rfactor * np.ones(manifold.dim_tangent),
            initial_velocity,
        )
        return LQTWrapper(manifold, obj)
    return None
