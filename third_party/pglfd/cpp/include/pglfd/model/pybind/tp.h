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
#include <ranges>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include "pglfd/manif/product.h"
#include "pglfd/model/gaussian.h"
#include "pglfd/model/gaussian_regressor.h"
#include "pglfd/model/tp.h"
#include "pglfd/util/pybind_helpers.h"

namespace pglfd {
namespace py = pybind11;

template<typename ManifoldIn, typename ScalarOut>
void declare_tp(py::module &m, const std::string &name) {
	py::class_<
			TaskParameterizedModel<ManifoldIn, ScalarOut>,
			std::shared_ptr<TaskParameterizedModel<ManifoldIn, ScalarOut>>,
			GaussianRegressor<ManifoldIn, SE3Manifold<ScalarOut>>,
			GaussianRegressorR3SO3<ManifoldIn, ScalarOut>>(m, name.c_str())
			.def(py::init(
					[](const std::vector<std::string> &origin_names,
						 const std::vector<Eigen::MatrixXd> &py_origins,
						 const std::vector<std::shared_ptr<GaussianRegressorR3SO3<ManifoldIn, ScalarOut>>>
								 &models) {
						// Construct Origins
						std::vector<typename TaskParameterizedModel<ManifoldIn, ScalarOut>::Origin> origins;
						for(const auto &[origin_name, origin]: std::views::zip(origin_names, py_origins)) {
							origins.emplace_back(origin_name, origin);
						}
						return new TaskParameterizedModel<ManifoldIn, ScalarOut>(origins, models);
					}))
			.def(
					"update_origin",
					[](TaskParameterizedModel<ManifoldIn, ScalarOut> &self,
						 const std::string &origin_name,
						 const Eigen::MatrixXd &origin) { self.updateOrigin({origin_name, origin}); })
			.def(
					"get_origins",
					[](const TaskParameterizedModel<ManifoldIn, ScalarOut> &self)
							-> std::tuple<std::vector<std::string>, std::vector<Eigen::MatrixXd>> {
						std::vector<std::string> origin_names;
						std::vector<Eigen::MatrixXd> origins;
						for(const auto &origin: self.getOrigins()) {
							origin_names.push_back(origin.name);
							origins.push_back(origin.origin);
						}
						return {origin_names, origins};
					})
			.def(
					"__repr__",
					[](const TaskParameterizedModel<ManifoldIn, ScalarOut> &self) {
						std::stringstream ss;
						ss << "<pglfd.model.tp.TaskParameterizedModel>";
						return ss.str();
					})
			.def(
					"__call__",
					[](const TaskParameterizedModel<ManifoldIn, ScalarOut> &self, const Eigen::MatrixXd &x) {
						return self(matrixToVector<ManifoldIn>(x));
					});
}

template<typename ManifoldIn, typename ManifoldOut>
void declare_tp_manif(py::module &m, const std::string &name) {
	py::class_<
			TaskParameterizedModelManif<ManifoldIn, ManifoldOut>,
			std::shared_ptr<TaskParameterizedModelManif<ManifoldIn, ManifoldOut>>,
			GaussianRegressor<ManifoldIn, ManifoldOut>,
			GaussianRegressorR3SO3<ManifoldIn, typename ManifoldOut::Scalar>>(m, name.c_str())
			.def(py::init(
					[](const std::shared_ptr<GaussianRegressor<ManifoldIn, ManifoldOut>> &model_manif,
						 const std::vector<std::string> &origin_names,
						 const std::vector<Eigen::MatrixXd> &py_origins,
						 const std::vector<std::shared_ptr<
								 GaussianRegressorR3SO3<ManifoldIn, typename ManifoldOut::Scalar>>> &models) {
						// Construct Origins
						std::vector<
								typename TaskParameterizedModel<ManifoldIn, typename ManifoldOut::Scalar>::Origin>
								origins;
						for(const auto &[origin_name, origin]: std::views::zip(origin_names, py_origins)) {
							origins.emplace_back(origin_name, origin);
						}
						return new TaskParameterizedModelManif<ManifoldIn, ManifoldOut>(
								model_manif,
								std::make_shared<TaskParameterizedModel<ManifoldIn, typename ManifoldOut::Scalar>>(
										origins, models));
					}))
			.def(
					"update_origin",
					[](TaskParameterizedModelManif<ManifoldIn, ManifoldOut> &self,
						 const std::string &origin_name,
						 const Eigen::MatrixXd &origin) { self.updateOrigin({origin_name, origin}); })
			.def(
					"__repr__",
					[](const TaskParameterizedModelManif<ManifoldIn, ManifoldOut> &self) {
						std::stringstream ss;
						ss << "<pglfd.model.tp.TaskParameterizedModel>";
						return ss.str();
					})
			.def(
					"__call__",
					[](const TaskParameterizedModelManif<ManifoldIn, ManifoldOut> &self,
						 const Eigen::MatrixXd &x) { return self(matrixToVector<ManifoldIn>(x)); });
}
}  // namespace pglfd
