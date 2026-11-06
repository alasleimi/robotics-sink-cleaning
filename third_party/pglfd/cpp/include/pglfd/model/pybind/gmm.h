/*
 * Copyright (c) 2026 Maximilian Mühlbauer
 *
 * This file is part of pglfd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
#include <pybind11/eigen.h>  // IWYU pragma: keep
#include <pybind11/stl.h>    // IWYU pragma: keep
#include <pybind11/stl.h>
#include "pglfd/model/gmm.h"
#include "pglfd/model/gmr.h"
#include "pglfd/util/pybind_helpers.h"

namespace pglfd {
namespace py = pybind11;

template<typename Manifold>
py::class_<GMM<Manifold>> declare_gmm(py::module &m, const std::string &name) {
	return py::class_<GMM<Manifold>>(m, name.c_str())
			.def(py::init<const std::vector<Gaussian<Manifold>> &, const std::vector<double> &>())
			.def("tangent_action", &GMM<Manifold>::tangentAction)
			.def("action", &GMM<Manifold>::action)
			.def(
					"expectation",
					[](const GMM<Manifold> &self, const Eigen::MatrixXd &samples) {
						std::vector<typename Manifold::Vector> samples_vec = matrixToVector<Manifold>(samples);
						auto probs = self.expectation(samples_vec);
						Eigen::MatrixXd res;
						res.resize(samples_vec.size(), probs[0].size());
						for(int i = 0; i < probs.size(); i++)
							for(int j = 0; j < probs[i].size(); j++)
								res(i, j) = probs[i][j];
						return res;
					})
			.def("approximate_single_gaussian", &GMM<Manifold>::approximateSingleGaussian)
			.def_static(
					"fit",
					[](int k,
						 const Eigen::MatrixXd &data,
						 const std::string &init_algorithm,
						 double eps,
						 int maxit) {
						std::vector<typename Manifold::Vector> data_vec = matrixToVector<Manifold>(data);
						typename GMM<Manifold>::InitAlgorithm init_algorithm_enum;
						if(init_algorithm.compare("kmeans") == 0) {
							init_algorithm_enum = GMM<Manifold>::InitAlgorithm::KMEANS;
						} else {
							throw std::runtime_error("Unknown init algorithm");
						}
						return GMM<Manifold>::fit(k, data_vec, init_algorithm_enum, eps, maxit);
					})
			.def("sample", &GMM<Manifold>::sample)
			.def_readwrite("gaussians", &GMM<Manifold>::gaussians_)
			.def_readwrite("priors", &GMM<Manifold>::priors_);
}
}  // namespace pglfd
