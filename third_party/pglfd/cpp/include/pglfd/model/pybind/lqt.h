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
#include <sstream>
#include <string>
#include <vector>
#include "pglfd/model/gaussian.h"
#include "pglfd/model/lqt.h"

namespace pglfd {
namespace py = pybind11;

template<typename Manifold>
void declare_lqt(py::module &m, const std::string &name) {
	py::class_<LQT<Manifold>>(m, name.c_str())
			.def(py::init([](const typename Manifold::Vector &initial_state,
											 const Eigen::MatrixXd &t_vias,
											 const std::vector<Gaussian<Manifold>> &vias,
											 double tend,
											 double dt,
											 const Manifold::TangentVector &rfactor,
											 const Manifold::TangentVector &initial_velocity) {
				std::vector<double> t_vias_list;
				t_vias_list.reserve(t_vias.rows());
				for(int i = 0; i < t_vias.rows(); i++) {
					t_vias_list.push_back(t_vias(i, 0));
				}
				return new LQT<Manifold>(
						initial_state, t_vias_list, vias, tend, dt, rfactor, initial_velocity);
			}))
			.def(
					"__repr__",
					[](LQT<Manifold> &self) {
						std::stringstream ss;
						ss << "<pglfd.model.lqt.LQT>";
						return ss.str();
					})
			.def("fit", &LQT<Manifold>::fit)
			.def("get_dt", &LQT<Manifold>::getDt)
			.def("get_states", &LQT<Manifold>::getStates)
			.def("get_poses", &LQT<Manifold>::getPoses)
			.def("get_velocities", &LQT<Manifold>::getVelocities)
			.def_readwrite("commands", &LQT<Manifold>::commands_);
}
}  // namespace pglfd
