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
#include <string>
#include "pglfd/model/gaussian_regressor.h"
#include "pglfd/model/gmr.h"
#include "pglfd/util/pybind_helpers.h"

namespace pglfd {
namespace py = pybind11;

template<typename ManifoldIn, typename ManifoldOut>
void declare_gmr(py::module &m, const std::string &name) {
	py::class_<
			GMR<ManifoldIn, ManifoldOut>,
			std::shared_ptr<GMR<ManifoldIn, ManifoldOut>>,
			GaussianRegressor<ManifoldIn, ManifoldOut>,
			GaussianRegressorR3SO3<ManifoldIn, typename ManifoldOut::Scalar>>(m, name.c_str())
			.def(
					"__call__",
					[](const GMR<ManifoldIn, ManifoldOut> &gmr,
						 const Eigen::MatrixXd &x,
						 double eps_marg,
						 double eps_cond,
						 double eps_approx,
						 int maxit) {
						return gmr(
								matrixToVector<ManifoldIn>(x),
								typename GMR<ManifoldIn, ManifoldOut>::GmrParameters{
										eps_marg, eps_cond, eps_approx, maxit});
					});
}
}  // namespace pglfd
