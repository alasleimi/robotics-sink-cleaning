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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // IWYU pragma: keep
#include <Eigen/Core>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "pglfd/model/conditioned_gaussian.h"
#include "pglfd/model/gaussian.h"
#include "pglfd/util/pybind_helpers.h"

namespace pglfd {
namespace py = pybind11;

template<typename Manifold>
py::class_<Gaussian<Manifold>> declare_gaussian(py::module &m, const std::string &name) {
	return py::class_<Gaussian<Manifold>, std::shared_ptr<Gaussian<Manifold>>>(m, name.c_str())
			.def(py::init<const typename Manifold::Vector &, const typename Manifold::TangentMatrix &>())
			.def_static(
					"mle_from_data",
					[](const Eigen::MatrixXd &data,
						 const Eigen::VectorXd &weights,
						 float reg,
						 float eps,
						 int maxit) {
						std::vector<typename Manifold::Vector> data_vec = matrixToVector<Manifold>(data);
						std::vector<typename Manifold::Scalar> weights_vec;
						for(int i = 0; i < weights.size(); i++) {
							weights_vec.push_back(weights[i]);
						}
						return Gaussian<Manifold>::mleFromData(data_vec, weights_vec, reg, eps, maxit);
					})
			.def(
					"__repr__",
					[](const Gaussian<Manifold> &self) {
						std::stringstream ss;
						ss << "<pglfd.model.gaussian.Gaussian at  '" << self.mu_.transpose() << "'>";
						return ss.str();
					})
			.def("tangent_action", &Gaussian<Manifold>::tangentAction)
			.def("action", &Gaussian<Manifold>::action)
			.def(
					"probability",
					[](std::shared_ptr<Gaussian<Manifold>> self, const Eigen::MatrixXd &samples) {
						std::vector<typename Manifold::Vector> samples_vec = matrixToVector<Manifold>(samples);
						Eigen::VectorXd res;
						res.resize(samples_vec.size());
						auto probs = self->probability(samples_vec);
						for(int i = 0; i < probs.size(); i++)
							res[i] = probs[i];
						return res;
					})
			.def("sample", &Gaussian<Manifold>::sample)
			.def_readwrite("mu", &Gaussian<Manifold>::mu_)
			.def_readwrite("sigma", &Gaussian<Manifold>::sigma_);
}
}  // namespace pglfd
