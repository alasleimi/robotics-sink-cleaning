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

import pickle

import numpy as np

from pglfd.manif.product import ProductManifold
from pglfd.model.gaussian import Gaussian
from pglfd.model.lqt import LQT
from tests.model.test_data import have_cpp


def test_lqt() -> None:
    man = ProductManifold.from_string("R3*SO3")

    # Define via points
    cov = np.eye(6) * 1e-5
    t_vias = np.array([0.2, 0.6, 1.0])
    vias = [
        Gaussian(man, np.array([0.2, 0.0, 0.2, 0.0113761, 0.0864101, 0.1300295, 0.9876721]), cov),
        Gaussian(man, np.array([0.7, 0.0, 0.70, 0, 0, 0, 1]), cov),
        Gaussian(man, np.array([1.0, 0.0, 1.0, -0.0317164, 0.0919997, 0.1261366, 0.9872283]), cov),
    ]

    # Initialize LQT
    lqt = LQT(man, np.array([0, 0, 0, 0, 0, 0, 1]), t_vias, vias, dt=0.05, rfactor=1e-4)
    lqt.fit()

    if have_cpp():
        assert lqt.is_cpp

    # Save results
    # data = {}
    # data["means"] = [state.mu for state in lqt.states]
    # data["sigmas"] = [state.sigma for state in lqt.states]
    # with open("tests/test_lqt.pickle", "wb") as f:
    #    pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

    # Check results
    with open("tests/model/test_lqt.pickle", "rb") as f:
        data = pickle.load(f)
    assert len(lqt.states) == len(data["means"])
    for i in range(len(lqt.states)):
        assert np.allclose(data["means"][i], lqt.states[i].mu, rtol=1e-2, atol=1e-2)
        assert np.allclose(data["sigmas"][i], lqt.states[i].sigma, rtol=1e-2, atol=1e-2)
