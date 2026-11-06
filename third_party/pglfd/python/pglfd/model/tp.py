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

""":cite:t:`muehlbauer2026nonparametric`."""

import typing
from collections.abc import Sequence
from typing import Any, cast

import numpy as np

from pglfd.geom.so3 import quaternion_to_rotmat
from pglfd.manif.conversions import manifold_from_r3_so3, manifold_to_r3_so3
from pglfd.manif.jacobian import manifold_jacobian
from pglfd.manif.manifold import Manifold
from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian, gaussian_product
from pglfd.model.gaussian_regressor import GaussianRegressor

try:
    from pglfd_cpp.tp_wrapper import get_cpp_tp, get_cpp_tp_manif

    have_cpp = True
except ImportError:
    have_cpp = False


class TaskParameterizedModel(GaussianRegressor):
    """
    Task-parametrized model supporting models on different manifolds.

    Data input is accepted as Euclidean; output data will be on R3*SO3.
    """

    def __new__(cls, model_origins: list[tuple[str, np.ndarray]], models: Sequence[GaussianRegressor]) -> Any:
        """
        Initialize the model with origins and corresponding models.

        :param model_origins: SE(3) coordinate origins of the respective models in the "global origin".
        :param models: The corresponding models.
        """
        # Check origins
        if len(model_origins) != len(models):
            msg = "Number of origins and models must match."
            raise ValueError(msg)

        # Check inputs
        for i in range(len(model_origins)):
            if len(model_origins[i]) != 2:
                msg = f"Origin format at {i} is invalid."
                raise ValueError(msg)

        if have_cpp and cls == TaskParameterizedModel:
            all_cpp = True
            for model in models:
                if not model.is_cpp:
                    all_cpp = False
            if all_cpp:
                tp_cpp = get_cpp_tp(model_origins, models)
                if tp_cpp is not None:
                    return tp_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: model_origins, models
        """
        return self.model_origins, self.models

    def __init__(self, model_origins: list[tuple[str, np.ndarray]], models: Sequence[GaussianRegressor]) -> None:
        """
        Initialize the model with origins and corresponding models.

        :param model_origins: SE(3) coordinate origins of the respective models in the "global origin".
        :param models: The corresponding models.
        """
        self.model_origins = model_origins

        #: The models used under the hood.
        self.models = list(models)

    @property
    @typing.override
    def input_manifold(self) -> Manifold:
        return self.models[0].input_manifold

    @property
    @typing.override
    def output_manifold(self) -> Manifold:
        return self.models[0].output_manifold

    def update_origin(self, new_origin: tuple[str, np.ndarray]) -> None:
        """
        Update an origin of the task-parameterized model.

        :param new_origin: The new origin.
        """
        for i in range(len(self.model_origins)):
            if self.model_origins[i][0] == new_origin[0]:
                self.model_origins[i] = new_origin
                return
        msg = f"Did not find model origin {new_origin[0]}"
        raise RuntimeError(msg)

    def __call__(self, x_query: np.ndarray) -> Gaussian | list[Gaussian]:
        """
        Query the task-parametrized model for some output.

        :param x_query: Query value in the "global origin".
        :return: Gaussian in the "global origin".
        """
        # Reshape input
        inp_shape = x_query.shape
        x_query = x_query.reshape(-1, x_query.shape[-1])

        # Query all our models
        model_gaussians = []
        for origin, model in zip(self.model_origins, self.models, strict=True):
            # Query the model
            model_result = cast("list[Gaussian]", model(x_query))

            # Transform the output
            # Get manifold
            man = model_result[0].manifold

            # Transform the Gaussian to R3*SO(3)
            model_result_r3_so3 = []
            for gaussian in model_result:
                mu_r3_so3 = manifold_to_r3_so3(gaussian.manifold, gaussian.mu)
                jacobian = np.linalg.inv(manifold_jacobian(gaussian.manifold, gaussian.mu))
                sigma_r3_so3 = jacobian @ gaussian.sigma @ jacobian.T
                model_result_r3_so3.append(Gaussian(ProductManifold.from_string("R3*SO3"), mu_r3_so3, sigma_r3_so3))

            # Move the Gaussians to the global origin
            # First get the tangent space rotation
            A = np.eye(man.dim_tangent)
            A[:3, :3] = quaternion_to_rotmat(origin[1][3:])

            # Then move the Gaussians
            for gaussian in model_result_r3_so3:
                gaussian.tangent_action(A)
                gaussian.action(origin[1][: man.dim_manifold])

            model_gaussians.append(model_result_r3_so3)

        # Calculate the Gaussian product
        result_gaussians = []
        for i in range(len(model_gaussians[0])):
            gaussian_sel = [model_gaussian[i] for model_gaussian in model_gaussians]
            result_gaussians.append(gaussian_product(*gaussian_sel))

        # reshape
        if len(inp_shape) <= 1 and len(result_gaussians) == 1:
            return result_gaussians[0]
        return result_gaussians


class TaskParameterizedModelManif(GaussianRegressor):
    """
    Task-parametrized model supporting models on different manifolds.

    Data input is accepted as Euclidean; output data will be on the manifold of the first model.
    """

    def __new__(cls, model_origins: list[tuple[str, np.ndarray]], models: list[GaussianRegressor]) -> Any:
        """
        Initialize the model with origins and corresponding models.

        :param model_origins: SE(3) coordinate origins of the models 1-n in the "global origin".
        :param models: The corresponding models.
        """
        # Check origins
        if len(model_origins) != len(models) - 1:
            msg = "Number of origins and models must match."
            raise ValueError(msg)

        # Check inputs
        for i in range(len(model_origins)):
            if len(model_origins[i]) != 2:
                msg = f"Origin format at {i} is invalid."
                raise ValueError(msg)

        if have_cpp and cls == TaskParameterizedModelManif:
            all_cpp = True
            for model in models:
                if not model.is_cpp:
                    all_cpp = False
            if all_cpp:
                tp_cpp = get_cpp_tp_manif(model_origins, models)
                if tp_cpp is not None:
                    return tp_cpp
        return object.__new__(cls)

    def __getnewargs__(self) -> tuple:
        """
        Get args for pickling.

        :return: model_origins, models
        """
        return self.model_origins, self.models

    def __init__(self, model_origins: list[tuple[str, np.ndarray]], models: list[GaussianRegressor]) -> None:
        """
        Initialize the model with origins and corresponding models.

        :param model_origins: SE(3) coordinate origins of the respective models in the "global origin".
        :param models: The corresponding models.
        """
        self.model_origins = model_origins

        #: The models used under the hood.
        self.models = models

        self.tp_model = TaskParameterizedModel(model_origins, models[1:])

    @property
    @typing.override
    def input_manifold(self) -> Manifold:
        return self.models[0].input_manifold

    @property
    @typing.override
    def output_manifold(self) -> Manifold:
        return self.models[0].output_manifold

    def update_origin(self, new_origin: tuple[str, np.ndarray]) -> None:
        """
        Update an origin of the task-parameterized model.

        :param new_origin: The new origin.
        """
        self.tp_model.update_origin(new_origin)

    def __call__(self, x_query: np.ndarray) -> Gaussian | list[Gaussian]:
        """
        Query the task-parametrized model for some output.

        :param x_query: Query value in the "global origin".
        :return: Gaussian in the "global origin".
        """
        # Reshape input
        inp_shape = x_query.shape
        x_query = x_query.reshape(-1, x_query.shape[-1])

        # Get the manifold result
        model_gaussians = [cast("list[Gaussian]", self.models[0](x_query))]

        # Get the other model results + transform
        tp_res = cast("list[Gaussian]", self.tp_model(x_query))
        out_man = self.models[0].output_manifold
        model_result_man = []
        for gaussian in tp_res:
            mu_man = manifold_from_r3_so3(out_man, gaussian.mu)
            jacobian = manifold_jacobian(out_man, mu_man)
            sigma_man = jacobian @ gaussian.sigma @ jacobian.T
            model_result_man.append(Gaussian(out_man, mu_man, sigma_man))
        model_gaussians.append(model_result_man)

        # Calculate the Gaussian product
        result_gaussians = []
        for i in range(len(model_gaussians[0])):
            gaussian_sel = [model_gaussian[i] for model_gaussian in model_gaussians]
            result_gaussians.append(gaussian_product(*gaussian_sel))

        # reshape
        if len(inp_shape) <= 1 and len(result_gaussians) == 1:
            return result_gaussians[0]
        return result_gaussians
