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
#include <memory>
#include <string>
#include "pglfd/model/kernel.h"

namespace pglfd {
namespace py = pybind11;

template<typename ManifoldIn>
void declare_rbf_kernel(py::module &m, const std::string &name) {
	std::string name_base = name + "base";
	py::class_<Kernel<ManifoldIn>, std::shared_ptr<Kernel<ManifoldIn>>>(m, name_base.c_str());
	py::class_<RBFKernel<ManifoldIn>, std::shared_ptr<RBFKernel<ManifoldIn>>, Kernel<ManifoldIn>>(
			m, name.c_str())
			.def(py::init([](typename ManifoldIn::TangentVector length_scale,
											 typename ManifoldIn::Scalar height_scale) {
				return std::make_shared<RBFKernel<ManifoldIn>>(length_scale, height_scale);
			}));
}

template<typename ManifoldIn, int p>
void declare_matern_kernel(py::module &m, const std::string &name) {
	py::class_<
			MaternKernel<ManifoldIn, p>,
			std::shared_ptr<MaternKernel<ManifoldIn, p>>,
			Kernel<ManifoldIn>>(m, name.c_str())
			.def(py::init([](typename ManifoldIn::TangentVector length_scale,
											 typename ManifoldIn::Scalar height_scale) {
				return std::make_shared<MaternKernel<ManifoldIn, p>>(length_scale, height_scale);
			}));
}

template<typename ManifoldIn, typename ManifoldOut>
void declare_kernel_product(py::module &m, const std::string &name) {
	std::string name_base = name + "base";
	py::class_<
			MatrixKernel<ManifoldIn, ManifoldOut>,
			std::shared_ptr<MatrixKernel<ManifoldIn, ManifoldOut>>>(m, name_base.c_str());
	py::class_<
			KernelProductIdentity<ManifoldIn, ManifoldOut>,
			std::shared_ptr<KernelProductIdentity<ManifoldIn, ManifoldOut>>,
			MatrixKernel<ManifoldIn, ManifoldOut>>(m, name.c_str())
			.def(py::init([](std::shared_ptr<Kernel<ManifoldIn>> kernel) {
				return std::make_shared<KernelProductIdentity<ManifoldIn, ManifoldOut>>(kernel);
			}));
}

template<typename ManifoldIn, typename ManifoldOut>
void declare_rbf_kernel_diagonal(py::module &m, const std::string &name) {
	if constexpr(ManifoldIn::dim_tangent == ManifoldOut::dim_tangent) {
		py::class_<
				KernelProductDiagonalRBF<ManifoldIn, ManifoldOut>,
				std::shared_ptr<KernelProductDiagonalRBF<ManifoldIn, ManifoldOut>>,
				MatrixKernel<ManifoldIn, ManifoldOut>>(m, name.c_str())
				.def(py::init([](typename ManifoldIn::TangentVector length_scale_rbf,
												 typename ManifoldIn::TangentVector length_scale_diag,
												 typename ManifoldIn::Scalar height_scale) {
					return std::make_shared<KernelProductDiagonalRBF<ManifoldIn, ManifoldOut>>(
							std::make_shared<RBFKernel<ManifoldIn>>(length_scale_rbf, height_scale),
							length_scale_diag);
				}));
	}
}

template<typename ManifoldIn, typename ManifoldOut, int p>
void declare_matern_kernel_diagonal(py::module &m, const std::string &name) {
	if constexpr(ManifoldIn::dim_tangent == ManifoldOut::dim_tangent) {
		py::class_<
				KernelProductDiagonalMatern<ManifoldIn, ManifoldOut, p>,
				std::shared_ptr<KernelProductDiagonalMatern<ManifoldIn, ManifoldOut, p>>,
				MatrixKernel<ManifoldIn, ManifoldOut>>(m, name.c_str())
				.def(py::init([](typename ManifoldIn::TangentVector length_scale_matern,
												 typename ManifoldIn::TangentVector length_scale_diag,
												 typename ManifoldIn::Scalar height_scale) {
					return std::make_shared<KernelProductDiagonalMatern<ManifoldIn, ManifoldOut, p>>(
							std::make_shared<MaternKernel<ManifoldIn, p>>(length_scale_matern, height_scale),
							length_scale_diag);
				}));
	}
}
}  // namespace pglfd
