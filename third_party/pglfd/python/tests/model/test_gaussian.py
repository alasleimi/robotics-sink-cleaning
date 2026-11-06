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

from typing import cast

import numpy as np
import pytest

from pglfd.geom.se3 import multiply_poses
from pglfd.geom.so3 import quaternion_normalize, quaternion_to_rotmat
from pglfd.manif.euclidean import EuclideanManifold
from pglfd.manif.product import ProductManifold
from pglfd.manif.s2 import S2Manifold
from pglfd.manif.so3 import SO3Manifold
from pglfd.model.gaussian import Gaussian, gaussian_product
from tests.model.test_data import generate_data, have_cpp, test_gaussian_data  # noqa: F401


@pytest.fixture()
def data(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["data"]


@pytest.fixture()
def data_s2(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["data_s2"]


@pytest.fixture()
def mu(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["mu"]


@pytest.fixture()
def sigma(test_gaussian_data):  # noqa: F811
    return test_gaussian_data["sigma"]


def test_gaussian(data, mu, sigma) -> None:
    gaussian = Gaussian.mle_from_data(ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), data)
    assert np.allclose(gaussian.mu, mu)
    assert np.allclose(gaussian.sigma, sigma)

    if have_cpp():
        assert gaussian.is_cpp

    with pytest.raises(ValueError, match=r"Incorrect mean shape \(4,\), expected 3") as v_info:
        gaussian = Gaussian(ProductManifold.from_string("R3"), np.array([0, 0, 0, 0]), np.eye(3))
    assert str(v_info.value) == "Incorrect mean shape (4,), expected 3"

    with pytest.raises(ValueError, match=r"Incorrect covariance shape \(4, 4\), expected \(3, 3\)") as v_info:
        gaussian = Gaussian(ProductManifold.from_string("R3"), np.array([0, 0, 0]), np.eye(4))
    assert str(v_info.value) == "Incorrect covariance shape (4, 4), expected (3, 3)"


def test_gaussian_probability(mu, sigma) -> None:
    manifold = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian(manifold, mu, sigma)

    peak_val = 1 / np.sqrt((2 * np.pi) ** manifold.dim_tangent * np.linalg.det(sigma))
    assert np.allclose(gaussian.probability(mu), peak_val)
    assert np.allclose(gaussian.probability(np.array([mu, mu, mu])), np.array([peak_val, peak_val, peak_val]))

    logval = np.array([1, 1, 1, 1, 1, 1, 1])
    mu_shifted = manifold.exp(logval, mu)
    dist = logval.T @ np.linalg.inv(sigma) @ logval
    expected_val = np.exp(-0.5 * dist) / (np.sqrt((2 * np.pi) ** manifold.dim_tangent * np.linalg.det(sigma)) + 1e-200)
    assert np.allclose(gaussian.probability(mu_shifted), expected_val)
    assert np.allclose(gaussian.probability(np.array([mu_shifted, mu, mu])), np.array([expected_val, peak_val, peak_val]))


def test_gaussian_sample(data) -> None:
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian.mle_from_data(man, data)
    gaussian.mu = gaussian.mu + np.array([1, 2, 3, 4, 0, 0, 0, 0])
    assert gaussian.sample(10).shape == (10, 8)

    vecs = gaussian.sample(1000)
    sample_gaussian = Gaussian.mle_from_data(man, vecs)

    assert np.allclose(gaussian.mu, sample_gaussian.mu, atol=1e-5, rtol=np.inf)
    assert np.allclose(gaussian.sigma, sample_gaussian.sigma, atol=1e-5, rtol=np.inf)


def test_gaussian_margin(mu, sigma) -> None:
    manifold = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian(manifold, mu, sigma)

    assert np.allclose(gaussian.margin([0]).mu, mu[0])
    assert np.allclose(gaussian.margin([0]).sigma, sigma[0, 0])

    assert np.allclose(gaussian.margin([2]).mu, mu[4:8])
    assert np.allclose(gaussian.margin([2]).sigma, sigma[4:7, 4:7])

    new_range = np.array([0, 4, 5, 6, 7], dtype=int)
    assert np.allclose(gaussian.margin([0, 2]).mu, mu[new_range])
    assert np.allclose(gaussian.margin([0, 2]).sigma, sigma[np.ix_(new_range[:-1], new_range[:-1])])


def test_gaussian_condition(mu, sigma, data, data_s2) -> None:
    manifold = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian(manifold, mu, sigma)

    # condition on scalar input
    conditioned_gaussian = gaussian.condition([0], [1, 2])
    sigma_res = np.array(
        [
            [9.40976288e-02, 2.27675989e-04, -2.37544918e-04, -5.28989970e-04, 1.14811598e-04, 1.01954410e-04],
            [2.27675989e-04, 8.49339357e-02, -1.75725866e-04, -7.86373962e-04, -1.09421734e-03, 1.57350584e-05],
            [-2.37544918e-04, -1.75725866e-04, 8.53249991e-02, 2.18221772e-03, 3.18036836e-04, -1.04670060e-03],
            [-5.28989970e-04, -7.86373962e-04, 2.18221772e-03, 1.87626104e-02, -5.13471286e-04, -4.98703607e-03],
            [1.14811598e-04, -1.09421734e-03, 3.18036836e-04, -5.13471286e-04, 1.04368122e-02, -8.29641233e-04],
            [1.01954410e-04, 1.57350584e-05, -1.04670060e-03, -4.98703607e-03, -8.29641233e-04, 3.14032177e-03],
        ]
    )
    res = cast("Gaussian", conditioned_gaussian(np.array([mu[0]])))
    assert np.allclose(res.mu, mu[1:])
    assert np.allclose(res.sigma, sigma_res)
    condres = cast("Gaussian", conditioned_gaussian(np.array(np.array([0.1]))))
    assert np.allclose(
        condres.mu,
        np.array([4.97495847, 0.15972542, 0.28895078, 0.04362065, 0.48137352, -0.02179696, 0.87515808]),
    )
    assert np.allclose(
        condres.sigma,
        np.array(
            [
                [9.40976288e-02, 2.27675989e-04, -2.37544918e-04, -5.28981927e-04, 1.15795261e-04, 1.00878007e-04],
                [2.27675989e-04, 8.49339357e-02, -1.75725866e-04, -7.89268275e-04, -1.09220650e-03, 9.15006119e-06],
                [-2.37544918e-04, -1.75725866e-04, 8.53249991e-02, 2.18606465e-03, 3.16433673e-04, -1.03913178e-03],
                [-5.28981927e-04, -7.89268275e-04, 2.18606465e-03, 1.87886649e-02, -5.13359679e-04, -4.94588575e-03],
                [1.15795261e-04, -1.09220650e-03, 3.16433673e-04, -5.13359679e-04, 1.04458730e-02, -7.89317059e-04],
                [1.00878007e-04, 9.15006119e-06, -1.03913178e-03, -4.94588575e-03, -7.89317059e-04, 3.10520640e-03],
            ]
        ),
        rtol=1e-3,
        atol=1e-4,
    )
    multi_res = cast("list", conditioned_gaussian(np.array([mu[0], mu[0], mu[0]])))
    assert len(multi_res) == 3
    for res in multi_res:
        assert np.allclose(res.mu, mu[1:])
        assert np.allclose(res.sigma, sigma_res)

    # condition on position, get orientation
    conditioned_gaussian = gaussian.condition([1], [2])
    sigma_res = np.array(
        [
            [0.01870228, -0.00053449, -0.00496305],
            [-0.00053449, 0.01042357, -0.00082353],
            [-0.00496305, -0.00082353, 0.00312953],
        ]
    )
    res = cast("Gaussian", conditioned_gaussian(mu[1:4]))
    assert np.allclose(res.mu, mu[4:])
    assert np.allclose(res.sigma, sigma_res)
    condres = cast("Gaussian", conditioned_gaussian(np.array(np.array([0.1, 0.1, 0.2]))))
    assert np.allclose(
        condres.mu,
        np.array([0.05564852, 0.47665574, -0.03350728, 0.87668684]),
    )
    assert np.allclose(
        condres.sigma,
        np.array(
            [
                [0.01868226, -0.00058391, -0.00498888],
                [-0.00058391, 0.01040037, -0.00091946],
                [-0.00498888, -0.00091946, 0.00317275],
            ]
        ),
        rtol=1e-3,
        atol=1e-3,
    )
    multi_res = cast("list", conditioned_gaussian(np.array([mu[1:4], mu[1:4], mu[1:4]])))
    assert len(multi_res) == 3
    for res in multi_res:
        assert np.allclose(res.mu, mu[4:])
        assert np.allclose(res.sigma, sigma_res)

    # condition on orientation, get position
    conditioned_gaussian = gaussian.condition([2], [1])
    sigma_res = np.array(
        [[0.09417585, 0.00033878, -0.00017725], [0.00033878, 0.08490256, -0.00010491], [-0.00017725, -0.00010491, 0.0849505]]
    )
    res = cast("Gaussian", conditioned_gaussian(mu[4:]))
    assert np.allclose(res.mu, mu[1:4])
    assert np.allclose(res.sigma, sigma_res)
    condres = cast("Gaussian", conditioned_gaussian(np.array([0.0248371, 0.5298574, 0, 0.847723])))
    assert np.allclose(
        condres.mu,
        np.array([5.01154582, 0.19442862, 0.28764536]),
    )
    assert np.allclose(
        condres.sigma,
        np.array(
            [
                [0.09417585, 0.00033878, -0.00017725],
                [0.00033878, 0.08490256, -0.00010491],
                [-0.00017725, -0.00010491, 0.0849505],
            ]
        ),
        rtol=1e-3,
        atol=1e-4,
    )
    multi_res = cast("list", conditioned_gaussian(np.array([mu[4:], mu[4:], mu[4:]])))
    assert len(multi_res) == 3
    for res in multi_res:
        assert np.allclose(res.mu, mu[1:4])
        assert np.allclose(res.sigma, sigma_res)

    # condition on scalar, get position
    # check with https://calinon.ch/misc/EE613/EE613-nonlinearRegression.pdf (slide 9)
    conditioned_gaussian = gaussian.condition([0], [1])
    x = mu[0] + 0.5
    condres = cast("Gaussian", conditioned_gaussian(x))
    assert np.allclose(condres.mu, mu[1:4] + sigma[1:4, 0] * 1 / sigma[0, 0] * (x - mu[0]))
    assert np.allclose(
        condres.sigma,
        sigma[np.ix_(range(1, 4), range(1, 4))]
        - sigma[1:4, 0].reshape((3, 1)) * 1 / sigma[0, 0] @ sigma[0, 1:4].reshape(1, 3),
    )

    # TODO check conditioning with e.g. pos - pos

    # condition on position, get scalar
    # check with https://calinon.ch/misc/EE613/EE613-nonlinearRegression.pdf (slide 9)
    conditioned_gaussian = gaussian.condition([1], [0])
    x = mu[1:4] + np.array([0.1, 0.2, 0.3])
    res = cast("Gaussian", conditioned_gaussian(x))
    assert np.allclose(res.mu, mu[0] + sigma[0, 1:4] @ np.linalg.pinv(sigma[1:4, 1:4]) @ (x - mu[1:4]))
    assert np.allclose(
        res.sigma,
        sigma[0, 0] - sigma[0, 1:4] @ np.linalg.pinv(sigma[1:4, 1:4]) @ sigma[1:4, 0],
    )

    # check with https://calinon.ch/misc/EE613/EE613-nonlinearRegression.pdf (slide 9)
    conditioned_gaussian = gaussian.condition([1], [0])
    x = mu[1:4] + np.array([1.1, 0.2, 0.3])
    res = cast("Gaussian", conditioned_gaussian(x))
    assert np.allclose(res.mu, mu[0] + sigma[0, 1:4] @ np.linalg.pinv(sigma[1:4, 1:4]) @ (x - mu[1:4]))
    assert np.allclose(
        res.sigma,
        sigma[0, 0] - sigma[0, 1:4] @ np.linalg.pinv(sigma[1:4, 1:4]) @ sigma[1:4, 0],
    )

    # conditioning on S2
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold(), S2Manifold()])
    gaussian = Gaussian.mle_from_data(man, np.hstack((data, data_s2)))
    conditioned_gaussian = gaussian.condition([0], [3])  # S2 output
    res = cast("Gaussian", conditioned_gaussian(np.array([0])))
    assert np.allclose(res.mu, np.array([0.70533761, 0.70886747, -0.00239947]))
    assert np.allclose(res.sigma, np.array([[0.00511537, 0.00022221], [0.00022221, 0.00489699]]))
    conditioned_gaussian = gaussian.condition([3], [2])  # S2 input
    res = cast("Gaussian", conditioned_gaussian(np.array([0, 1, 0])))
    assert np.allclose(res.mu, np.array([0.03729734, 0.46320968, -0.01721741, 0.88529614]))
    assert np.allclose(
        res.sigma,
        np.array(
            [
                [0.01852318, -0.00050174, -0.00524412],
                [-0.00050174, 0.01044831, -0.00071519],
                [-0.00524412, -0.00071519, 0.00329015],
            ]
        ),
    )


def test_gaussian_product(data, mu, sigma) -> None:
    gaussian = Gaussian.mle_from_data(ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()]), data)
    gaussian_prod = gaussian_product(gaussian, gaussian, gaussian, gaussian, gaussian)
    assert np.allclose(gaussian_prod.mu, mu)
    assert np.allclose(gaussian_prod.sigma, 1 / 5 * sigma)


def test_gaussian_tangent_action(data, mu, sigma) -> None:
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian.mle_from_data(man, data)

    # first test: identity transform should not change anything
    A = np.eye(7)
    gaussian.tangent_action(A)
    assert np.allclose(gaussian.mu, mu)
    assert np.allclose(gaussian.sigma, sigma)

    # second test: transforming back and forth should again yield the input
    # the q matrix of the qr decomposition yields a rotation matrix
    # generated from:
    # from scipy.stats import special_ortho_group  # noqa: ERA001
    # A = special_ortho_group.rvs(7)  # noqa: ERA001
    A = np.array(
        [
            [-0.02300445, -0.36224019, -0.21680596, 0.67960942, 0.38710765, -0.22572265, -0.39821603],
            [-0.22746674, -0.4704678, 0.69217458, -0.17220418, -0.11610535, -0.44309821, -0.09133743],
            [0.45311642, -0.05598789, 0.05170849, -0.40630012, 0.76121509, -0.1617461, 0.13485952],
            [-0.3155719, 0.63574553, 0.09067067, -0.18182749, 0.19084012, -0.22574096, -0.6062843],
            [0.19031789, -0.33562253, 0.08639568, -0.26590591, -0.04342746, 0.63364729, -0.60792403],
            [0.72094742, 0.31259981, 0.41380094, 0.35226111, -0.2446973, -0.10149368, -0.13045817],
            [0.29469218, -0.17255126, -0.53319711, -0.33991454, -0.39879334, -0.51359773, -0.24641991],
        ]
    )
    A_inv = A.T
    gaussian.tangent_action(A)
    gaussian.tangent_action(A_inv)
    assert np.allclose(gaussian.mu, mu)
    assert np.allclose(gaussian.sigma, sigma)


def test_gaussian_action(data, mu, sigma) -> None:
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian.mle_from_data(man, data)

    # first test: identity transform should not change anything
    trafo = man.id_elem
    gaussian.action(trafo)
    assert np.allclose(gaussian.mu, mu)
    assert np.allclose(gaussian.sigma, sigma)

    # second test: transforming back and forth should again yield the input
    trafo = np.array([1, 2, 3, 4, 0, 1, 0, 0])
    trafo_inv = man.action(man.id_elem, trafo, man.id_elem)
    gaussian.action(trafo)
    gaussian.action(trafo_inv)
    assert np.allclose(gaussian.mu, mu)
    assert np.allclose(gaussian.sigma, sigma)


def test_gaussian_se3_transport(data, mu, sigma) -> None:
    man = ProductManifold([EuclideanManifold(1), EuclideanManifold(3), SO3Manifold()])
    gaussian = Gaussian.mle_from_data(man, data)
    data2 = generate_data(pos_offset=np.array([5, 0.2, 0.3]), euler_start=np.array([0.1, 1, 0.0001]))
    gaussian_no_reg = Gaussian.mle_from_data(man, data2, reg=0)  # no regularization as this would add diagonal elements
    init_probability = gaussian_no_reg.probability(data2)

    data2_at_mu = man.action(data2.copy(), gaussian_no_reg.mu, man.id_elem)
    gaussian_at_mu = Gaussian.mle_from_data(man, data2_at_mu, reg=0)  # no regularization as this would add diagonal elements
    mu_probability = gaussian_at_mu.probability(data2_at_mu)
    assert np.allclose(init_probability, mu_probability)
    assert np.allclose(gaussian_no_reg.sigma, gaussian_at_mu.sigma)

    # combination of tangent action & parallel transport to perform Ax + b
    g = np.array([1, 2, 3, 4, 0.258819, 0, 0, 0.9659258])
    g[4:] = quaternion_normalize(g[4:])
    g_inv = man.action(man.id_elem, g, man.id_elem)
    A_g = np.eye(7)
    A_g[1:4, 1:4] = quaternion_to_rotmat(g[4:])
    A_g_inv = np.linalg.inv(A_g)

    # transform forward
    gaussian.tangent_action(A_g)
    gaussian.action(g)
    gaussian_no_reg.tangent_action(A_g)
    gaussian_no_reg.action(g)

    # forward-transform the data
    data_forward = data2.copy()
    data_forward[:, 0] += g[0]
    data_forward[:, 1:] = multiply_poses(g[1:], data2[:, 1:])

    # double-check forward-transformed data
    data_forward2 = data2.copy()
    data_forward2 = man.exp(
        (A_g.reshape(1, 7, 7) @ man.log(data_forward2, man.id_elem).reshape(-1, 7, 1)).reshape(-1, 7), g
    ).reshape(-1, 8)
    assert np.allclose(data_forward2, data_forward)

    # check for equivalent probability
    forward_probability = gaussian_no_reg.probability(data_forward)
    assert np.allclose(init_probability, forward_probability)

    # create a Gaussian with SE(3) transformed data
    gaussian_forward = Gaussian.mle_from_data(man, data_forward, reg=0)
    gaussian_forward_probability = gaussian_forward.probability(data_forward)

    # check transformed Gaussian and Gaussian on transformed data for equivalence
    assert np.allclose(gaussian_no_reg.mu, gaussian_forward.mu)
    assert np.allclose(gaussian_no_reg.sigma, gaussian_forward.sigma)
    assert np.allclose(init_probability, gaussian_forward_probability)

    # transform backward
    gaussian.action(g_inv)
    gaussian.tangent_action(A_g_inv)
    assert np.allclose(man.log(gaussian.mu, mu), np.zeros((man.dim_tangent, 1)), atol=1e-6)
    assert np.allclose(gaussian.sigma, sigma)

    # test transformation by h
    h = np.array([1, 2, 3, 4, 0.258819, 0, 0.9659258, 0])
    h[4:] = quaternion_normalize(h[4:])
    A_h = np.eye(7)
    A_h[1:4, 1:4] = quaternion_to_rotmat(h[4:])

    # transform data from g -> h
    data_forward_h = man.exp(
        (A_h.reshape(1, 7, 7) @ man.log(data_forward, man.id_elem).reshape(-1, 7, 1)).reshape(-1, 7), h
    ).reshape(-1, 8)

    # forward-transform forward gaussian
    gaussian_forward.tangent_action(A_h)
    gaussian_forward.action(h)

    # create new gaussian
    gaussian_forward_h = Gaussian.mle_from_data(man, data_forward_h, reg=0, maxit=1000)
    assert np.allclose(
        np.zeros((man.dim_tangent, 1)), man.log(gaussian_forward.mu, gaussian_forward_h.mu), rtol=1e-3, atol=1e-3
    )
    assert np.allclose(gaussian_forward.sigma, gaussian_forward_h.sigma)


def test_gaussian_s2_transport(data_s2) -> None:
    man = ProductManifold([S2Manifold()])
    gaussian = Gaussian.mle_from_data(man, data_s2, maxit=1000)
    mu = gaussian.mu.copy()
    sigma = gaussian.sigma.copy()
    gaussian_no_reg = Gaussian.mle_from_data(
        man, data_s2, reg=0, maxit=1000
    )  # no regularization as this would add diagonal elements
    init_probability = gaussian_no_reg.probability(data_s2)

    # combination of tangent action & parallel transport to perform Ax + b
    g = man.id_elem + np.array([0.5, 0.2, 0.1], dtype=float)
    g *= 1 / np.linalg.norm(g)
    g_inv = man.action(man.id_elem, g, man.id_elem)
    A = np.array([[0, -1], [1, 0]])
    A_inv = np.linalg.inv(A)

    # transform forward
    gaussian.tangent_action(A)
    gaussian.action(g)
    gaussian_no_reg.tangent_action(A)
    gaussian_no_reg.action(g)

    # forward-transform the data
    data_forward = data_s2.copy()
    data_forward = man.exp(
        (A.reshape(1, 2, 2) @ man.log(data_forward, man.id_elem).reshape(-1, 2, 1)).reshape(-1, 2), g
    ).reshape(-1, 3)

    # backward-transform the data and check if it is equivalent
    data_backward = data_forward.copy()
    data_backward = man.exp(
        (A_inv.reshape(1, 2, 2) @ man.log(data_backward, g).reshape(-1, 2, 1)).reshape(-1, 2), man.id_elem
    ).reshape(-1, 3)
    assert np.allclose(data_s2, man.exp(man.log(data_s2, man.id_elem), man.id_elem))
    assert np.allclose(data_s2, data_backward)

    # check for equivalent probability
    forward_probability = gaussian_no_reg.probability(data_forward)
    assert np.allclose(init_probability, forward_probability)

    # create a Gaussian with transformed data
    gaussian_forward = Gaussian.mle_from_data(man, data_forward, reg=0, maxit=1000)
    gaussian_forward_probability = gaussian_forward.probability(data_forward)

    # check transformed Gaussian and Gaussian on transformed data for equivalence
    assert np.allclose(gaussian_no_reg.mu, gaussian_forward.mu)
    assert np.allclose(gaussian_no_reg.sigma, gaussian_forward.sigma)
    assert np.allclose(init_probability, gaussian_forward_probability)

    # transform backward
    # ATTENTION: the order of action and tangent action is important as on S2,
    # the orientation of the base vectors depends on the position on the manifold
    # and thus rotating first yields a different parallel transport
    gaussian.action(g_inv)
    gaussian.tangent_action(A_inv)
    assert np.allclose(man.log(gaussian.mu, mu), np.zeros((man.dim_tangent, 1)))
    assert np.allclose(gaussian.sigma, sigma)

    # test transformation from g to h
    h = man.id_elem + np.array([0.5, 0.2, 0.1], dtype=float)
    h *= 1 / np.linalg.norm(h)
    h = g

    # transform data from g -> h
    data_forward_h = man.exp(
        (A.reshape(1, 2, 2) @ man.log(data_forward, man.id_elem).reshape(-1, 2, 1)).reshape(-1, 2), h
    ).reshape(-1, 3)

    # forward-transform forward gaussian
    gaussian_forward.tangent_action(A)
    gaussian_forward.action(h)

    # create new gaussian
    gaussian_forward_h = Gaussian.mle_from_data(man, data_forward_h, reg=0, maxit=1000)
    assert np.allclose(gaussian_forward.mu, gaussian_forward_h.mu)
    assert np.allclose(gaussian_forward.sigma, gaussian_forward_h.sigma)
