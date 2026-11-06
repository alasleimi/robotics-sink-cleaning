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

from typing import TYPE_CHECKING, cast

import numpy as np
import pytest

from pglfd.geom.se3 import invert_transformation
from pglfd.geom.so3 import quaternion_inverse, quaternion_to_rotmat
from pglfd.manif.conversions import manifold_from_r3_so3
from pglfd.manif.product import ProductManifold
from pglfd.model.gmm import GMM
from pglfd.model.tp import TaskParameterizedModel, TaskParameterizedModelManif
from tests.model.test_data import (
    have_cpp,
    test_gmm_data,  # noqa: F401
)

if TYPE_CHECKING:
    from pglfd.model.gaussian import Gaussian


@pytest.fixture()
def data_gmm(test_gmm_data):  # noqa: F811
    return test_gmm_data["data"]


def test_task_parametrized_model(data_gmm) -> None:
    # Create GMR with s as input
    gmm_man, _likelihood_man, _avg_loglikelihood_man = GMM.fit(ProductManifold.from_string("R1*R3*SO3"), 5, data_gmm)
    gmr_s_origin = gmm_man.gmr([0], [1, 2])

    # Create task-parametrized model from that
    tp_model = TaskParameterizedModel([("origin", np.array([0, 0, 0, 0, 0, 0, 1]))], [gmr_s_origin])
    assert np.allclose(
        cast("Gaussian", tp_model(np.array([0]))).mu,
        np.array(
            [1.17200068e00, 6.43068198e-03, -7.27453326e-04, 1.06913523e-03, 1.00761153e-03, -8.36800293e-04, 9.99998571e-01]
        ),
        atol=1e-4,
    )

    # check if it is cpp
    if have_cpp():
        assert tp_model.is_cpp

    # Create a GMR with new origin
    new_origin = np.array([0, 0.1, 0.5, 0.3, 0.4132863, -0.545638, -0.3537703, 0.6374325])
    A = np.eye(7)
    A[1:4, 1:4] = quaternion_to_rotmat(quaternion_inverse(new_origin[4:]))
    gmm_man.tangent_action(A)
    new_origin_inverse = new_origin.copy()
    new_origin_inverse[1:] = invert_transformation(new_origin[1:])
    gmm_man.action(new_origin_inverse)
    gmr_s_new_origin = gmm_man.gmr([0], [1, 2])

    # Create task-parametrized model from that
    tp_model = TaskParameterizedModel([("origin", new_origin[1:])], [gmr_s_new_origin])
    assert np.allclose(
        cast("Gaussian", tp_model(np.array([0]))).mu,
        np.array(
            [1.17200068e00, 6.43068198e-03, -7.27453326e-04, 1.06913523e-03, 1.00761153e-03, -8.36800293e-04, 9.99998571e-01]
        ),
        atol=1e-4,
    )

    # Create a task-parametrized model from both
    tp_model = TaskParameterizedModel(
        [("identity", np.array([0, 0, 0, 0, 0, 0, 1])), ("other_origin", np.array([0, 0, 0, 0, 0, 0, 1]))],
        [gmr_s_origin, gmr_s_new_origin],
    )
    tp_model.update_origin(("other_origin", new_origin[1:]))
    res_vec = [1.17200068e00, 6.43068198e-03, -7.27453326e-04, 1.06913523e-03, 1.00761153e-03, -8.36800294e-04, 9.99998571e-01]
    assert np.allclose(cast("Gaussian", tp_model(np.array([0]))).mu, np.array(res_vec), atol=1e-4)
    # the same for multiple outputs
    assert np.allclose(
        np.array([res.mu for res in cast("list", tp_model(np.array([0, 0, 0])))]),
        np.array([res_vec, res_vec, res_vec]),
        atol=1e-4,
    )


def test_task_parametrized_model_manif(data_gmm) -> None:
    # Prepare data
    man = ProductManifold.from_string("R1*SO2*R2*SO3")
    data_gmm_man = np.zeros((data_gmm.shape[0], 9))
    data_gmm_man[:, 0] = data_gmm[:, 0]
    data_gmm_man[:, 1:] = manifold_from_r3_so3(man, data_gmm[:, 1:])

    # Create GMR with s as input
    gmm_man, _likelihood_man, _avg_loglikelihood_man = GMM.fit(man, 10, data_gmm_man, eps=1e-2)
    gmr_s_origin = gmm_man.gmr([0], [1, 2, 3])

    # Create task-parametrized model from that
    tp_model = TaskParameterizedModelManif([("origin", np.array([0, 0, 0, 0, 0, 0, 1]))], [gmr_s_origin, gmr_s_origin])
    assert np.allclose(
        cast("Gaussian", tp_model(np.array([0]))).mu,
        cast("Gaussian", gmr_s_origin(np.array([0]))).mu,
    )

    # check if it is cpp
    if have_cpp():
        assert tp_model.is_cpp
