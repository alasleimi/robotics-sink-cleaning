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
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "pglfd/model/kmeans.h"
#include "pglfd/util/execution.h"
#include "pglfd/util/pybind_helpers.h"

namespace pglfd {
namespace py = pybind11;

template<typename Manifold>
py::class_<KMeans<Manifold>> declare_kmeans(py::module &m, const std::string &name) {
	return py::class_<KMeans<Manifold>, std::shared_ptr<KMeans<Manifold>>>(m, name.c_str())
			.def(py::init([](const std::vector<Eigen::MatrixXd> &cluster_centers) {
				std::vector<std::vector<typename Manifold::Vector>> cluster_centers_vec(
						cluster_centers.size());
				std::transform(
						execution::unseq,
						cluster_centers.begin(),
						cluster_centers.end(),
						cluster_centers_vec.begin(),
						[](const auto cluster_center) { return matrixToVector<Manifold>(cluster_center); });
				return KMeans<Manifold>(cluster_centers_vec);
			}))
			.def_static(
					"fit",
					[](int k, const Eigen::MatrixXd &data, int maxit) {
						std::vector<typename Manifold::Vector> data_vec = matrixToVector<Manifold>(data);
						return KMeans<Manifold>::fit(k, data_vec, maxit);
					})
			.def_property_readonly(
					"cluster_centers",
					[](const KMeans<Manifold> &self) {
						return vectorToMatrix<Manifold>(self.cluster_centers_);
					})
			.def_property_readonly(
					"clusters",
					[](const KMeans<Manifold> &self) {
						std::vector<Eigen::MatrixXd> res;
						for(int i = 0; i < self.clusters_.size(); i++) {
							res.push_back(vectorToMatrix<Manifold>(self.clusters_[i]));
						}
						return res;
					})
			.def_property_readonly("is_cpp", [](const KMeans<Manifold> &self) { return true; });
}
}  // namespace pglfd
