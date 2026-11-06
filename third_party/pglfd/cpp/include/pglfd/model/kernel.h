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
#include <memory>
#include <utility>
#include <vector>
#include "pglfd/manif/euclidean.h"

namespace pglfd {
template<typename Scalar, int val>
constexpr Scalar factorial() {
	if constexpr(val > 1)
		return val * factorial<Scalar, val - 1>();
	else
		return 1;
}

/**
 * Abstract base class for a Kernel.
 *
 * @param Manifold The manifold of the input data to this kernel.
 */
template<typename Manifold>
class Kernel {
public:
	/**
	 * Query the kernel for the distance between two vectors.
	 *
	 * @param x1 First input.
	 * @param x2 Second input.
	 * @return The scalar distance.
	 */
	virtual typename Manifold::Scalar operator()(
			const typename Manifold::Vector &x1, const typename Manifold::Vector &x2) const = 0;

	virtual ~Kernel() = default;
};

/**
 * Radial basis function as a Kernel using the Riemannian distance measure.
 *
 * @param Manifold The manifold of the input data to this kernel.
 */
template<typename Manifold>
class RBFKernel : public Kernel<Manifold> {
public:
	/**
	 * Constructor using a scalar length scale.
	 *
	 * @param length_scale The length scale parameter of the Kernel.
	 * @param height_scale The height scale parameter of the Kernel.
	 */
	explicit RBFKernel(
			typename Manifold::Scalar length_scale = 0.1, typename Manifold::Scalar height_scale = 1.0) :
			length_scaler_(Manifold::TangentVector::Ones() * 1. / (2. * length_scale * length_scale)),
			height_scale_(height_scale) {}

	/**
	 * Constructor using a vector length scale.
	 *
	 * @param length_scale The length scale parameter of the Kernel.
	 * @param height_scale The height scale parameter of the Kernel.
	 */
	explicit RBFKernel(
			typename Manifold::TangentVector length_scale = Manifold::TangentVector::Ones() * 0.1,
			typename Manifold::Scalar height_scale = 1.0) :
			length_scaler_((2. * length_scale.cwiseAbs2()).cwiseInverse()), height_scale_(height_scale) {}

	typename Manifold::Scalar operator()(
			const typename Manifold::Vector &x1, const typename Manifold::Vector &x2) const override {
		typename Manifold::TangentVector dist = manif_.log(x1, x2);
		return height_scale_
					 * std::exp(-(dist.transpose() * length_scaler_.asDiagonal() * dist).value());
	}

private:
	Manifold manif_;
	typename Manifold::TangentVector length_scaler_;
	typename Manifold::Scalar height_scale_;
};

/**
 * Matern Kernel using the Riemannian distance measure.
 *
 * @param Manifold The manifold of the input data to this kernel.
 * @param p P parameter.
 */
template<typename Manifold, int p = 2>
class MaternKernel : public Kernel<Manifold> {
public:
	static_assert(p >= 0, "p parameter must be non-negative.");

	/**
	 * Constructor using a scalar length scale.
	 *
	 * @param length_scale The length scale parameter of the Kernel.
	 * @param height_scale The height scale parameter of the Kernel.
	 */
	explicit MaternKernel(
			typename Manifold::Scalar length_scale = 0.1, typename Manifold::Scalar height_scale = 1.0) :
			length_scaler_(Manifold::TangentVector::Ones() * 1. / (length_scale * length_scale)),
			height_scale_(height_scale) {}

	/**
	 * Constructor using a vector length scale.
	 *
	 * @param length_scale The length scale parameter of the Kernel.
	 * @param height_scale The height scale parameter of the Kernel.
	 */
	explicit MaternKernel(
			typename Manifold::TangentVector length_scale = Manifold::TangentVector::Ones() * 0.1,
			typename Manifold::Scalar height_scale = 1.0) :
			length_scaler_(length_scale.cwiseAbs2().cwiseInverse()), height_scale_(height_scale) {}

	typename Manifold::Scalar operator()(
			const typename Manifold::Vector &x1, const typename Manifold::Vector &x2) const override {
		typename Manifold::TangentVector dist = manif_.log(x1, x2);
		typename Manifold::Scalar r =
				std::sqrt((dist.transpose() * length_scaler_.asDiagonal() * dist).value());
		return height_scale_ * std::exp(exp_prefix_ * r) * gamma_val_
					 * polynomial(std::make_integer_sequence<int, p + 1>(), r);
	}

private:
	Manifold manif_;
	typename Manifold::TangentVector length_scaler_;
	typename Manifold::Scalar height_scale_;
	constexpr static typename Manifold::Scalar nu_ = p + 0.5;
	constexpr static typename Manifold::Scalar gamma_val_ =
			factorial<typename Manifold::Scalar, p>() / factorial<typename Manifold::Scalar, 2 * p>();
	constexpr static typename Manifold::Scalar exp_prefix_ = -std::sqrt(2 * nu_);

	template<int i, int... is>
	typename Manifold::Scalar polynomial_impl(typename Manifold::Scalar r) const {
		constexpr typename Manifold::Scalar factorial_prefix =
				factorial<typename Manifold::Scalar, p + i>()
				/ (factorial<typename Manifold::Scalar, i>()
					 * factorial<typename Manifold::Scalar, p - i>());
		constexpr typename Manifold::Scalar exp_prefix = std::sqrt(8 * nu_);
		typename Manifold::Scalar val = factorial_prefix * std::pow(exp_prefix * r, p - i);
		if constexpr(sizeof...(is) > 0)
			return val + polynomial_impl<is...>(r);
		return val;
	}
	template<int... is>
	typename Manifold::Scalar polynomial(
			std::integer_sequence<int, is...>, typename Manifold::Scalar r) const {
		return polynomial_impl<is...>(r);
	}
};

/**
 * Abstract base class for a Matrix-valued Kernel.
 *
 * @param ManifoldIn The manifold of the input data to this kernel.
 * @param ManifoldOut The manifold of the output space of this kernel.
 */
template<typename ManifoldIn, typename ManifoldOut>
class MatrixKernel {
public:
	/**
	 * Query the kernel for the distance between two vectors.
	 *
	 * @param x1 First input.
	 * @param x2 Second input.
	 * @return The kernel matrix in output dimensions.
	 */
	virtual typename ManifoldOut::TangentMatrix operator()(
			const typename ManifoldIn::Vector &x1, const typename ManifoldIn::Vector &x2) const = 0;

	virtual ~MatrixKernel() = default;
};

/**
 * Product Kernel that multiplies a scalar-valued kernel with the identity matrix.
 *
 * @param ManifoldIn The manifold of the input data to this kernel.
 * @param ManifoldOut The manifold of the output space of this kernel.
 */
template<typename ManifoldIn, typename ManifoldOut>
class KernelProductIdentity : public MatrixKernel<ManifoldIn, ManifoldOut> {
public:
	/**
	 * Constructor.
	 *
	 * @param kernel The scalar kernel that will be multiplied with the identity matrix.
	 */
	explicit KernelProductIdentity(std::shared_ptr<Kernel<ManifoldIn>> kernel) : kernel_(kernel) {}

	typename ManifoldOut::TangentMatrix operator()(
			const typename ManifoldIn::Vector &x1, const typename ManifoldIn::Vector &x2) const override {
		return ManifoldOut::TangentMatrix::Identity() * (*kernel_)(x1, x2);
	}

private:
	std::shared_ptr<Kernel<ManifoldIn>> kernel_;
};

/**
 * Product Kernel that multiplies a scalar-valued kernel with diagonal rbf-valued entries.
 *
 * @param ManifoldIn The manifold of the input data to this kernel.
 * @param ManifoldOut The manifold of the output space of this kernel.
 */
template<typename Manifold, typename ManifoldOut>
class KernelProductDiagonalRBF : public MatrixKernel<Manifold, ManifoldOut> {
public:
	/**
	 * Constructor using a scalar length scale.
	 *
	 * @param kernel The scalar kernel that will be multiplied with the scaled identity matrix.
	 * @param length_scale The length scale parameter of the Kernel.
	 * @param height_scale The height scale parameter of the Kernel.
	 */
	explicit KernelProductDiagonalRBF(
			std::shared_ptr<Kernel<Manifold>> kernel,
			typename Manifold::Scalar length_scale = 0.1,
			typename Manifold::Scalar height_scale = 1.0) :
			kernel_(kernel),
			length_scaler_(Manifold::TangentVector::Ones() * 1. / (2. * length_scale * length_scale)) {}

	/**
	 * Constructor using a vector length scale.
	 *
	 * @param kernel The scalar kernel that will be multiplied with the scaled identity matrix.
	 * @param length_scale The length scale parameter of the Kernel.
	 */
	explicit KernelProductDiagonalRBF(
			std::shared_ptr<Kernel<Manifold>> kernel,
			typename Manifold::TangentVector length_scale = Manifold::TangentVector::Ones() * 0.1) :
			kernel_(kernel), length_scaler_((2. * length_scale.cwiseAbs2()).cwiseInverse()) {}

	typename Manifold::TangentMatrix operator()(
			const typename Manifold::Vector &x1, const typename Manifold::Vector &x2) const override {
		typename Manifold::TangentVector dist = manif_.log(x2, x1);
		return (*kernel_)(x1, x2)
					 * (-dist.cwiseAbs2().cwiseProduct(length_scaler_))
								 .array()
								 .exp()
								 .matrix()
								 .asDiagonal()
								 .toDenseMatrix();
	}

private:
	Manifold manif_;
	std::shared_ptr<Kernel<Manifold>> kernel_;
	typename Manifold::TangentVector length_scaler_;
};

/**
 * Product Kernel that multiplies a scalar-valued kernel with diagonal Matern-valued entries.
 *
 * @param Manifold The manifold of the input data to this kernel.
 * @param ManifoldOut The manifold of the output space of this kernel.
 * @param p P parameter.
 */
template<typename Manifold, typename ManifoldOut, int p = 2>
class KernelProductDiagonalMatern : public MatrixKernel<Manifold, ManifoldOut> {
public:
	/**
	 * Constructor using a scalar length scale.
	 *
	 * @param kernel The scalar kernel that will be multiplied with the scaled identity matrix.
	 * @param length_scale The length scale parameter of the Kernel.
	 * @param height_scale The height scale parameter of the Kernel.
	 */
	explicit KernelProductDiagonalMatern(
			std::shared_ptr<Kernel<Manifold>> kernel,
			typename Manifold::Scalar length_scale = 0.1,
			typename Manifold::Scalar height_scale = 1.0) :
			kernel_(kernel), length_scales_(Manifold::TangentVector::Ones() * length_scale) {
		initDiagonalKernels();
	}

	/**
	 * Constructor using a vector length scale.
	 *
	 * @param kernel The scalar kernel that will be multiplied with the scaled identity matrix.
	 * @param length_scale The length scale parameter of the Kernel.
	 */
	explicit KernelProductDiagonalMatern(
			std::shared_ptr<Kernel<Manifold>> kernel,
			typename Manifold::TangentVector length_scale = Manifold::TangentVector::Ones() * 0.1) :
			kernel_(kernel), length_scales_(length_scale) {
		initDiagonalKernels();
	}

	typename Manifold::TangentMatrix operator()(
			const typename Manifold::Vector &x1, const typename Manifold::Vector &x2) const override {
		typename Manifold::TangentVector dist = manif_.log(x2, x1);
		for(int i = 0; i < ManifoldOut::dim_tangent; i++) {
			dist[i] = diag_kernels_[i](
					EuclideanManifold<typename Manifold::Scalar, 1>::Vector::Zero(),
					(typename EuclideanManifold<typename Manifold::Scalar, 1>::Vector() << dist[i])
							.finished());
		}
		return (*kernel_)(x1, x2) * dist.asDiagonal().toDenseMatrix();
	}

private:
	Manifold manif_;
	std::shared_ptr<Kernel<Manifold>> kernel_;
	std::vector<MaternKernel<EuclideanManifold<typename Manifold::Scalar, 1>, p>> diag_kernels_;
	typename Manifold::TangentVector length_scales_;

	void initDiagonalKernels() {
		for(int i = 0; i < ManifoldOut::dim_tangent; i++) {
			diag_kernels_.emplace_back(length_scales_[i]);
		}
	}
};
}  // namespace pglfd
