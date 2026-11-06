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
#include <Eigen/Core>
#include <Eigen/Householder>  // IWYU pragma: keep
#include <algorithm>          // IWYU pragma: keep
#include <cassert>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <numbers>  // IWYU pragma: keep
#include <numeric>  // IWYU pragma: keep
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "pglfd/manif/product.h"
#include "pglfd/manif/util.h"
#include "pglfd/util/execution.h"

namespace pglfd {
template<typename Manifold>
class GaussianProduct;

template<typename ManifoldIn, typename ManifoldOut>
class ConditionedGaussian;

/**
 * Implementation of a Riemannian Gaussian distribution.
 */
template<typename Manifold>
class Gaussian {
public:
	template<typename ManifoldIn, typename ManifoldOut>
	friend class ConditionedGaussian;
	/**
	 * Constructor of the Gaussian.
	 *
	 * @param mu The mean of the gaussian.
	 * @param sigma The covariance matrix of the gaussian.
	 */
	Gaussian(  // cppcheck-suppress uninitMemberVar
			const typename Manifold::Vector &mu,
			const typename Manifold::TangentMatrix &sigma) :
			mu_(mu), sigma_(sigma) {}

	/**
	 * Constructor of the Gaussian.
	 *
	 * @param prod The gaussian product to evaluate.
	 */
	Gaussian(const GaussianProduct<Manifold> &prod) {  // cppcheck-suppress noExplicitConstructor
		Gaussian<Manifold> gauss = prod.eval();
		mu_ = gauss.mu_;
		sigma_ = gauss.sigma_;
	}

	/**
	 * Multiplication operator which evaluates to a GaussianProduct.
	 *
	 * @param rhs The other Gaussian to multiply with.
	 * @return A GaussianProduct.
	 */
	GaussianProduct<Manifold> operator*(const Gaussian<Manifold> &rhs) {
		return GaussianProduct<Manifold>(*this, rhs);
	}

	/**
	 * Multiplication operator which evaluates to a GaussianProduct.
	 *
	 * @param rhs The other GaussianProduct to multiply with.
	 * @return A GaussianProduct.
	 */
	GaussianProduct<Manifold> operator*(const GaussianProduct<Manifold> &rhs) {
		return GaussianProduct<Manifold>(*this, rhs);
	}

	// cppcheck-suppress functionConst
	/**
	 * Rotate our tangent space by A (pure rotation of the tangent space at the origin, no translation
	 * on the manifold). This implements the rotation by A of eq. 26 & eq. 27 in [1].
	 *
	 * @param A Rotation to perform in tangent space.
	 */
	void tangentAction(const typename Manifold::TangentMatrix &A) {
		mu_ = manif_.exp(A * manif_.log(mu_, manif_.id_elem()), manif_.id_elem());
		sigma_ = A * sigma_ * A.transpose();
	}

	/**
	 * Perform the action function on the Gaussian (pure translation, no rotation of the tangent
	 * space).
	 *
	 * @param trafo The transformation to apply to the gaussian.
	 */
	void action(const typename Manifold::Vector &trafo) {  // cppcheck-suppress functionConst
		std::string manif_name = manif_.name();
		if(manif_name.find("*") != std::string::npos
			 && (manif_name.find("S2") != std::string::npos
					 || manif_name.find("SO2") != std::string::npos)) {
			throw std::runtime_error(
					"Moving a Gaussian on {man_desc} is not implemented as it does not make sense.");
		}

		mu_ = manif_.action({mu_}, manif_.id_elem(), trafo)[0];
		if(manif_name.find("S2") != std::string::npos) {
			sigma_ = manif_.parallelTransportMatrix(sigma_, trafo, mu_);
		}
	}

	/**
	 * Calculate the weighted covariance of data.
	 *
	 * @param mu The data mean.
	 * @param data The data to compute the covariance of.
	 * @param weights The weights to use during covariance computation. Have to sum to 1.
	 * @param reg Diagonal regularization factor.
	 * @return The covariance value.
	 */
	static typename Manifold::TangentMatrix weightedCovarianceFromData(
			const typename Manifold::Vector &mu,
			const std::vector<typename Manifold::Vector> &data,
			const std::vector<typename Manifold::Scalar> &weights,
			typename Manifold::Scalar reg = 1e-8) {
		assert(data.size() == weights.size());
		Manifold manif;
		typename Manifold::TangentMatrix sigma;
		sigma.setZero();
		for(int i = 0; i < data.size(); i++) {
			typename Manifold::TangentVector logval = manif.log(data[i], mu);
			sigma += weights[i] * logval * logval.transpose();
		}
		return sigma + reg * Manifold::TangentMatrix::Identity();
	}

	/**
	 * Perform MLE on data.
	 *
	 * @param data The data to perform MLE of.
	 * @param weights Optional weights for the data points.
	 * @param reg Diagonal regularization factor for the covariance (to make it invertible).
	 * @param eps Stopping criterion.
	 * @param maxit Maximum number of iterations.
	 * @return The constructed gaussian.
	 */
	static Gaussian<Manifold> mleFromData(
			const std::vector<typename Manifold::Vector> &data,
			const std::vector<typename Manifold::Scalar> &weights = {},
			typename Manifold::Scalar reg = 1e-8,
			typename Manifold::Scalar eps = 1e-8,
			int maxit = 50) {
		std::vector<typename Manifold::Scalar> weights_copy = weights;
		if(weights_copy.size() == 0) {
			for(int i = 0; i < data.size(); i++) {
				weights_copy.push_back(1);
			}
		}
		assert(data.size() == weights_copy.size());

		// normalize weights
		typename Manifold::Scalar weight_sum = 0;
		weight_sum =
				std::reduce(execution::unseq, weights_copy.begin(), weights_copy.end(), weight_sum);
		std::for_each(
				execution::unseq,
				weights_copy.begin(),
				weights_copy.end(),
				[=](typename Manifold::Scalar &w) { w /= weight_sum; });

		typename Manifold::Vector mu = frechet_mean<Manifold>(data, weights_copy, eps, maxit);
		typename Manifold::TangentMatrix sigma =
				weightedCovarianceFromData(mu, data, weights_copy, reg);

		return Gaussian<Manifold>(mu, sigma);
	}

	/**
	 * Calculate the probability density function of individual samples.
	 *
	 * @param samples The samples to evaluate.
	 * @param recompute_decomposition Force to recompute the decomposition of sigma_.
	 * @return The probability density function evaluated at samples.
	 */
	std::vector<typename Manifold::Scalar> probability(
			const std::vector<typename Manifold::Vector> &samples,
			bool recompute_decomposition = true) const {
		if(recompute_decomposition || !sigma_qr_) {
			sigma_qr_ = std::make_shared<Eigen::HouseholderQR<typename Manifold::TangentMatrix>>(sigma_);
			constexpr typename Manifold::Scalar two_pi_k =
					std::pow(2 * std::numbers::pi, Manifold::dim_tangent);
			prob_divisor_ = std::sqrt(two_pi_k * sigma_.determinant()) + 1e-200;
		}

		std::vector<typename Manifold::Scalar> probabilities(samples.size());
		std::transform(
				execution::unseq,
				samples.begin(),
				samples.end(),
				probabilities.begin(),
				[&](const auto &sample) {
					typename Manifold::TangentVector log = manif_.log(sample, mu_);
					typename Manifold::Scalar dist = log.transpose() * sigma_qr_->solve(log);
					return std::exp(-0.5 * dist) / prob_divisor_;
				});
		return probabilities;
	}

	/**
	 * Sample from this Gaussian.
	 * https://peterroelants.github.io/posts/multivariate-normal-primer/#Sampling-from-a-multivariate-normal
	 *
	 * @param n The number of samples to draw
	 * @return Samples.
	 */
	std::vector<typename Manifold::Vector> sample(int n) {
		// Initialize the RNG
		static std::mt19937 gen{std::random_device{}()};
		static std::normal_distribution<> dist;

		// Transformation
		typename Manifold::TangentMatrix transformation = sigma_.llt().matrixL();

		// Sample
		std::vector<typename Manifold::Vector> res;
		for(int i = 0; i < n; i++) {
			res.emplace_back(manif_.exp(
					transformation *
							typename Manifold::TangentVector().unaryExpr([&](auto x) { return dist(gen); }),
					mu_));
		}
		return res;
	}

	/**
	 * Calculate the marginal distribution.
	 *
	 * @tparam idx_man Manifold dimensions.
	 * @return The marginal distribution.
	 */
	template<std::ptrdiff_t... idx_man>
	auto margin(const manifold_selector<idx_man...>) const {
		auto man = manif_.template submanifold<idx_man...>();
		return Gaussian<std::tuple_element_t<0, decltype(man)>>(
				mu_(std::get<1>(man)), sigma_(std::get<2>(man), std::get<2>(man)));
	}

	/**
	 * Gaussian conditioning. Returns a ConditionedGaussian which can be called with
	 * the concrete input values.
	 *
	 * @param idx_in_val Input manifolds.
	 * @param idx_out_val Output manifolds.
	 * @return The conditioned Gaussian.
	 */
	template<std::ptrdiff_t... idx_in, std::ptrdiff_t... idx_out>
	auto condition(
			const manifold_selector<idx_in...> &idx_in_val,
			const manifold_selector<idx_out...> &idx_out_val) const {
		auto man_tot = manif_.template submanifold<idx_in..., idx_out...>();
		auto man_in = manif_.template submanifold<idx_in...>();
		auto man_out = manif_.template submanifold<idx_out...>();

		// Get Gaussians
		auto gauss_tot = Gaussian<std::tuple_element_t<0, decltype(man_tot)>>(
				mu_(std::get<1>(man_tot)), sigma_(std::get<2>(man_tot), std::get<2>(man_tot)));
		auto gauss_in = Gaussian<std::tuple_element_t<0, decltype(man_in)>>(
				mu_(std::get<1>(man_in)), sigma_(std::get<2>(man_in), std::get<2>(man_in)));
		auto gauss_out = Gaussian<std::tuple_element_t<0, decltype(man_out)>>(
				mu_(std::get<1>(man_out)), sigma_(std::get<2>(man_out), std::get<2>(man_out)));

		// Get sigma_oi
		Eigen::Matrix<
				typename Manifold::Scalar,
				std::tuple_element_t<0, decltype(man_out)>::dim_tangent,
				std::tuple_element_t<0, decltype(man_in)>::dim_tangent>
				sigma_oi = sigma_(std::get<2>(man_out), std::get<2>(man_in));
		return ConditionedGaussian<
				std::tuple_element_t<0, decltype(man_in)>,
				std::tuple_element_t<0, decltype(man_out)>>(gauss_in, gauss_out, sigma_oi);
	}

	// Variables
	typename Manifold::Vector mu_;            //!< Mean of the gaussian.
	typename Manifold::TangentMatrix sigma_;  //!< Covariance of the gaussian.

private:
	Manifold manif_;
	mutable std::shared_ptr<Eigen::HouseholderQR<typename Manifold::TangentMatrix>>
			sigma_qr_;                                    // store qr decomposition for sampling
	mutable typename Manifold::Scalar prob_divisor_;  // store divisor for probability calculation
};

/**
 * Implementation of a Riemannian Gaussian product.
 */
template<typename Manifold>
class GaussianProduct {
public:
	std::vector<Gaussian<Manifold>> elems_;  //!< Gaussians to be multiplied by a call to ``.eval``.

	/**
	 * Constructor.
	 *
	 * @param elems Gaussian to multiply.
	 */
	explicit GaussianProduct(const std::vector<Gaussian<Manifold>> &elems) : elems_(elems) {}

	/**
	 * Constructor.
	 *
	 * @param left Gaussian to multiply from the left side.
	 * @param right Gaussian to multiply from the right side.
	 */
	GaussianProduct(const Gaussian<Manifold> &left, const Gaussian<Manifold> &right) {
		elems_.push_back(left);
		elems_.push_back(right);
	}

	/**
	 * Constructor.
	 *
	 * @param left Gaussian to multiply from the left side.
	 * @param right GaussianProduct to multiply from the right side.
	 */
	GaussianProduct(const Gaussian<Manifold> &left, const GaussianProduct<Manifold> &right) {
		elems_.push_back(left);
		std::copy(right.elems_.begin(), right.elems_.end(), std::back_inserter(elems_));
	}

	/**
	 * Constructor.
	 *
	 * @param left GaussianProduct to multiply from the left side.
	 * @param right Gaussian to multiply from the right side.
	 */
	GaussianProduct(const GaussianProduct<Manifold> &left, const Gaussian<Manifold> &right) {
		std::copy(left.elems_.begin(), left.elems_.end(), std::back_inserter(elems_));
		elems_.push_back(right);
	}

	/**
	 * Constructor.
	 *
	 * @param left GaussianProduct to multiply from the left side.
	 * @param right GaussianProduct to multiply from the right side.
	 */
	GaussianProduct(const GaussianProduct<Manifold> &left, const GaussianProduct<Manifold> &right) {
		std::copy(left.elems_.begin(), left.elems_.end(), std::back_inserter(elems_));
		std::copy(right.elems_.begin(), right.elems_.end(), std::back_inserter(elems_));
	}

	/**
	 * Multiply this GaussianProduct with another Gaussian.
	 *
	 * @param rhs The Gaussian to multiply from the right side.
	 * @return a new GaussianProduct.
	 */
	GaussianProduct<Manifold> operator*(const Gaussian<Manifold> &rhs) {
		return GaussianProduct<Manifold>(*this, rhs);
	}

	/**
	 * Multiply this GaussianProduct with another GaussianProduct.
	 *
	 * @param rhs The GaussianProduct to multiply from the right side.
	 * @return a new GaussianProduct.
	 */
	GaussianProduct<Manifold> operator*(const GaussianProduct<Manifold> &rhs) {
		return GaussianProduct<Manifold>(*this, rhs);
	}

	/**
	 * Evaluate the GaussianProduct.
	 *
	 * @param eps Epsilon value.
	 * @return The Gaussian resulting from the product operation.
	 */
	Gaussian<Manifold> eval(double eps = 1e-8) const {
		typename Manifold::Vector mu = elems_[0].mu_;
		typename Manifold::TangentMatrix sigma;
		sigma.setIdentity();

		// Precompute inverse covariances
		std::vector<typename Manifold::TangentMatrix> sigma_inv;
		for(int i = 0; i < elems_.size(); i++) {
			sigma_inv.push_back(elems_[i].sigma_.inverse());
		}

		int it = 0;
		typename Manifold::Scalar diff = 1;
		while(diff > eps) {
			// Compute new covariance
			std::vector<typename Manifold::TangentMatrix> sigma_transp;
			typename Manifold::TangentMatrix sigma_inv_sum;
			sigma_inv_sum.setZero();
			for(int i = 0; i < elems_.size(); i++) {
				sigma_transp.emplace_back(manif_.parallelTransportMatrix(sigma_inv[i], elems_[i].mu_, mu));
				sigma_inv_sum += sigma_transp[i];
			}
			sigma = sigma_inv_sum.inverse();

			// Update mu
			typename Manifold::TangentVector delta_inv_sum;
			delta_inv_sum.setZero();
			for(int i = 0; i < elems_.size(); i++) {
				delta_inv_sum += sigma_transp[i] * manif_.log(elems_[i].mu_, mu);
			}
			typename Manifold::TangentVector delta = sigma * delta_inv_sum;
			mu = manif_.exp(delta, mu);

			// Check convergence
			diff = delta.transpose() * delta;
			it++;
			if(it > 50) {
				throw std::runtime_error("Gaussian product did not converge");
			}
		}

		return Gaussian<Manifold>(mu, sigma);
	}

private:
	Manifold manif_;
};
}  // namespace pglfd
