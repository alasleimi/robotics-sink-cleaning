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

from pglfd.manif.product import ProductManifold
from pglfd_cpp.gaussian_regressor_wrapper import GaussianRegressorWrapper

if TYPE_CHECKING:
    import numpy as np
    from pglfd.gaussian.gaussian_regressor import GaussianRegressor
    from pglfd.manif.manifold import Manifold


class TaskParameterizedModelWrapper(GaussianRegressorWrapper):
    def __init__(self, manifold_in: Manifold, manifold_out: Manifold, model: Any, models: Any) -> None:
        super().__init__(manifold_in, manifold_out, model)
        self.models = models

    def update_origin(self, new_origin: tuple[str, np.ndarray]) -> None:
        self.model.update_origin(new_origin[0], new_origin[1])

    @property
    def model_origins(self) -> list:
        origin_names, origin_poses = self.model.get_origins()
        origins = []
        for name, pose in zip(origin_names, origin_poses, strict=True):
            origins.append((name, pose.reshape(-1)))
        return origins


def get_cpp_tp(
    model_origins: list[tuple[str, np.ndarray]], models: list[GaussianRegressor]
) -> TaskParameterizedModelWrapper | None:
    """
    Initialize the model with origins and corresponding models.

    :param model_origins: SE(3) / position / orientation coordinate origins
                            of the respective models in the "global origin".
    :param models: The corresponding models.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        tp,
    )

    manifold_in = models[0].manifold_in
    manifold_out = ProductManifold.from_string("R3*SO3")

    man_in = "".join(str(manifold_in).split("*"))

    name_tp = f"TaskParameterized{man_in}"
    if hasattr(tp, name_tp):
        # assign origins
        origin_names = [origin[0] for origin in model_origins]
        origins = [origin[1] for origin in model_origins]

        # get the actual c++ object
        obj = getattr(tp, name_tp)(origin_names, origins, [model.model for model in models])
        return TaskParameterizedModelWrapper(manifold_in, manifold_out, obj, models)
    return None


def get_cpp_tp_manif(
    model_origins: list[tuple[str, np.ndarray]], models: list[GaussianRegressor]
) -> TaskParameterizedModelWrapper | None:
    """
    Initialize the model with origins and corresponding models.

    :param model_origins: SE(3) / position / orientation coordinate origins
                            of the respective models in the "global origin".
    :param models: The corresponding models.
    """
    from pglfd_cpp import (  # protected import, otherwise might have a partially initialized module
        gaussian,  # noqa: F401
        gaussian_regressor,  # noqa: F401
        tp,
    )

    manifold_in = models[0].manifold_in
    manifold_out = models[0].manifold_out

    man_in = "".join(str(manifold_in).split("*"))
    man_out = "".join(str(manifold_out).split("*"))

    name_tp = f"TaskParameterizedManif{man_in}_{man_out}"
    if hasattr(tp, name_tp):
        # assign origins
        origin_names = [origin[0] for origin in model_origins]
        origins = [origin[1] for origin in model_origins]

        # get the actual c++ object
        obj = getattr(tp, name_tp)(models[0].model, origin_names, origins, [model.model for model in models[1:]])
        return TaskParameterizedModelWrapper(manifold_in, manifold_out, obj, models)
    return None
