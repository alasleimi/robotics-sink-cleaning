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
#include <any>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>
#include "pglfd/model/gaussian.h"
#include "pglfd/model/gaussian_regressor.h"
#include "pglfd/model/kernel.h"  // IWYU pragma: keep
#include "pglfd/util/eigen_extensions.h"
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * Riemannian Gaussian process implementation.
 *
 * @param ManifoldIn The input data manifold.
 * @param ManifoldOut The output data manifold.
 */
template<typename ManifoldIn, typename ManifoldOut>
class GaussianProcess : public GaussianRegressor<ManifoldIn, ManifoldOut> {
public:
	/**
	 * Constructor.
	 *
	 * @param x Input values.
	 * @param y Output values.
	 * @param kernel The kernel to compute distances between query values and data points.
	 * @param sigma Covariance parameter of the process.
	 * @param update_k Used by the KMP, as this has its own routine for updating k.
	 */
	GaussianProcess(
			const std::vector<typename ManifoldIn::Vector> &x,
			const std::vector<typename ManifoldOut::Vector> &y,
			const std::shared_ptr<MatrixKernel<ManifoldIn, ManifoldOut>> &kernel,
			typename ManifoldIn::Scalar sigma,
			bool update_k = true) :
			x_(x), y_(y), kernel_(kernel), sigma_(sigma), had_inverse_update_(false) {
		if(x.size() != y.size()) {
			throw std::runtime_error("sizes of input data output data must agree!");
		}

		if(update_k) {
			// Precompute what can be precomputed
			this->updateK();
		}
	}

	/**
	 * Query the Gaussian process for some output.
	 *
	 * @param x_query The query values.
	 * @param parameters The parameters to be supplied to the method, must be empty.
	 * @return The calculated outputs.
	 */
	std::vector<Gaussian<ManifoldOut>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const override {
		// Check input
		if(parameters.has_value()) {
			throw std::runtime_error(
					"operator() dos not take any additional parameters for Gaussian Process / KMP!");
		}

		// Preallocate result
		std::vector<Gaussian<ManifoldOut>> res;
		for(const auto &x_i: x_query) {
			res.emplace_back(manif_out_.id_elem(), ManifoldOut::TangentMatrix::Zero());
		}

		// Multi-threaded calculation
#pragma omp parallel for
		for(int i = 0; i < x_query.size(); i++) {
			typename ManifoldIn::MatrixX ks =
					applyKernel(std::vector<typename ManifoldIn::Vector>{x_query[i]}, x_);
			typename ManifoldIn::MatrixX kss = applyKernel(
					std::vector<typename ManifoldIn::Vector>{x_query[i]},
					std::vector<typename ManifoldIn::Vector>{x_query[i]});

			// Select sigma solve based on furthest discontinuity
			const typename ManifoldIn::MatrixX *sigma_solve =
					(discontinuities0_.size() == 0) ? &sigma_solve0_ : &sigma_solve1_;
			if(check_alt_log_) {
				// Calculate closest "normal" point
				std::vector<typename ManifoldIn::Scalar> distvals(x_.size());
				std::transform(
						execution::unseq, x_.begin(), x_.end(), distvals.begin(), [&](const auto &x) {
							return manif_in_.log(x_query[i], x).squaredNorm();
						});
				int closest_idx =
						std::distance(distvals.begin(), std::min_element(distvals.begin(), distvals.end()));
				if(vias_.count(closest_idx))
					closest_idx = vias_.find(closest_idx)->second;

				int closest0 = -1, closest1 = -1;
				auto it = std::min_element(
						execution::unseq,
						discontinuities0_.begin(),
						discontinuities0_.end(),
						[closest_idx](double a, double b) {
							return std::abs(a - closest_idx) < std::abs(b - closest_idx);
						});
				if(it != discontinuities0_.end()) {
					closest0 = std::distance(discontinuities0_.begin(), it);
				}
				it = std::min_element(
						execution::unseq,
						discontinuities1_.begin(),
						discontinuities1_.end(),
						[closest_idx](double a, double b) {
							return std::abs(a - closest_idx) < std::abs(b - closest_idx);
						});
				if(it != discontinuities1_.end()) {
					closest1 = std::distance(discontinuities1_.begin(), it);
				}
				bool disc_cond = std::fabs(closest_idx - closest0) > std::fabs(closest_idx - closest1);

				// Calculate closest via point
				int closest_via = -1;
				for(auto const &[key, val]: vias_) {
					if(std::fabs(closest_idx - val) < std::fabs(closest_idx - closest_via))
						closest_via = val;
				}
				bool via_disc_cond =
						std::fabs(closest_idx - closest1) > std::fabs(closest_idx - closest_via);

				// Calculate via distance
				bool via_cond = false;
				if(closest_via >= 0)
					via_cond = (log_x_in_orig_.template block<ManifoldOut::dim_tangent, 1>(0, closest_via)
											- log_x_in_1_.template block<ManifoldOut::dim_tangent, 1>(0, closest_via))
												 .squaredNorm()
										 < (log_x_in_orig_.template block<ManifoldOut::dim_tangent, 1>(0, closest_via)
												- log_x_in_0_.template block<ManifoldOut::dim_tangent, 1>(0, closest_via))
													 .squaredNorm();

				if((!disc_cond || !via_disc_cond) && !via_cond)
					sigma_solve = &sigma_solve0_;
				else
					sigma_solve = &sigma_solve1_;
			}

			typename ManifoldOut::Vector mu = manif_out_.id_elem();
			typename ManifoldOut::TangentVector delta =
					-ks * *sigma_solve + calculateNsAction(x_query[i], ks);
			mu = manif_out_.exp(delta, mu);

			// Calculate sigma
			typename ManifoldOut::TangentMatrix sigma = calculateSigma(kss, ks);
			res[i] = Gaussian<ManifoldOut>(mu, sigma);
		}
		return res;
	}

	/**
	 * Get a reference to the input data of the GaussianProcess.
	 *
	 * @return A reference to the input data.
	 */
	const std::vector<typename ManifoldIn::Vector> &getX() const {
		return x_;
	}

	/**
	 * Get a reference to the output data of the GaussianProcess.
	 *
	 * @return A reference to the output data.
	 */
	const std::vector<typename ManifoldOut::Vector> &getY() const {
		return y_;
	}

	/**
	 * Get the K matrix.
	 *
	 * @return The K matrix.
	 */
	typename ManifoldIn::MatrixX getK() const {
		if(!had_inverse_update_)
			return k_;
		else
			return this->applyKernel(this->x_, this->x_);
	}

	/**
	 * Get sigma ii inv.
	 *
	 * @return Sigma ii inv.
	 */
	typename ManifoldIn::MatrixX getSigmaIiInv() const {
		return sigma_llt_.solve(
				ManifoldIn::MatrixX::Identity(sigma_llt_.matrixL().rows(), sigma_llt_.matrixL().cols()));
	}

protected:
	// Manifold stuff
	ManifoldIn manif_in_;    //!< The input data manifold.
	ManifoldOut manif_out_;  //!< The output data manifold.

	// Parameters & Data
	std::vector<typename ManifoldIn::Vector> x_;                     //!< The input data.
	std::vector<typename ManifoldOut::Vector> y_;                    //!< The output data.
	std::shared_ptr<MatrixKernel<ManifoldIn, ManifoldOut>> kernel_;  //!< The kernel correlating query
																																	 //!< values with known inputs.
	typename ManifoldIn::Scalar sigma_;                              //!< Covariance of the process.

	// Precomputed values
	bool had_inverse_update_;
	typename ManifoldIn::MatrixX k_;                            //!< K matrix
	typename ManifoldIn::MatrixX sigma_block_;                  //!< covariance matrix
	typename ManifoldIn::MatrixX sigma_ii_inv_;                 //!< Inverse of Kss.
	Eigen::UpdateLLT<typename ManifoldIn::MatrixX> sigma_llt_;  //!< Cholesky decomposition of sigma.
	typename ManifoldIn::MatrixX sigma_solve0_;   //!< Pre-solved sigma for first log choice.
	typename ManifoldIn::MatrixX sigma_solve1_;   //!< Pre-solved sigma for second log choice.
	std::vector<int> discontinuities0_;           //!< Logarithm discontinuities for first choice.
	std::vector<int> discontinuities1_;           //!< Logarithm discontinuities for second choice.
	bool check_alt_log_;                          //!< Also calculate with second log choice.
	typename ManifoldIn::MatrixX log_x_in_orig_;  //!< Original input log.
	typename ManifoldIn::MatrixX log_x_in_0_;     //!< Logs for first choice.
	typename ManifoldIn::MatrixX log_x_in_1_;     //!< Logs for second choice.
	std::map<int, int> vias_;                     //!< Store location of via points.

	/**
	 * Apply the kernel to data.
	 *
	 * @param x1 First elements.
	 * @param x2 Second elements.
	 * @return The kernel matrix.
	 */
	typename ManifoldIn::MatrixX applyKernel(
			const std::span<const typename ManifoldIn::Vector> &x1,
			const std::span<const typename ManifoldIn::Vector> &x2) const {
		typename ManifoldIn::MatrixX res;
		res.resize(x1.size() * ManifoldOut::dim_tangent, x2.size() * ManifoldOut::dim_tangent);
#pragma omp parallel for collapse(2)
		for(int i = 0; i < x1.size(); i++) {
			for(int j = 0; j < x2.size(); j++) {
				res.template block<ManifoldOut::dim_tangent, ManifoldOut::dim_tangent>(
						i * ManifoldOut::dim_tangent, j * ManifoldOut::dim_tangent) = (*kernel_)(x1[i], x2[j]);
			}
		}
		return res;
	}

	/**
	 * Pre-solve sigma for faster GP evaluations.
	 *
	 * @param append_after Elements were inserted after this idx.
	 */
	void preSolveSigma(int append_after = -1) {
		typename ManifoldOut::Vector mu = manif_out_.id_elem();
		log_x_in_orig_.conservativeResize(ManifoldOut::dim_tangent, y_.size());

		// Calculate the update
		int start = 0;
		if(append_after > 0)
			start = append_after;
#pragma omp parallel for
		for(int j = start; j < y_.size(); j++) {
			log_x_in_orig_.template block<ManifoldOut::dim_tangent, 1>(0, j) = manif_out_.log(mu, y_[j]);
		}
		check_alt_log_ = false;
		if constexpr(ManifoldIn::dim_tangent == 1 && ManifoldOut::discont_log_alt) {
			std::tie(log_x_in_0_, discontinuities0_) =
					manif_out_.continuousLog(log_x_in_orig_, vias_, false);
			check_alt_log_ = discontinuities0_.size() > 0 || vias_.size() > 0;
			if(check_alt_log_)
				std::tie(log_x_in_1_, discontinuities1_) =
						manif_out_.continuousLog(log_x_in_orig_, vias_, true);
		} else {
			log_x_in_0_ = log_x_in_orig_;
		}
		sigma_solve0_ = sigma_llt_.solve(log_x_in_0_.reshaped(ManifoldOut::dim_tangent * y_.size(), 1));
		if(check_alt_log_)
			sigma_solve1_ =
					sigma_llt_.solve(log_x_in_1_.reshaped(ManifoldOut::dim_tangent * y_.size(), 1));
	}

	/**
	 * Function to update Kss.
	 *
	 * @param append_after Elements were inserted after this idx.
	 * @param remove_idxs Elements to remove.
	 */
	virtual void updateK(int append_after = -1, const std::vector<int> &remove_idxs = {}) {
		// K matrix
		k_ = applyKernel(x_, x_);

		// sigma * I
		sigma_block_ = ManifoldIn::MatrixX::Identity(k_.rows(), k_.cols()) * sigma_ * sigma_;

		// TODO check parallel transport, also with the python part!!!
		sigma_llt_ = (k_ + sigma_block_).llt();
		if(sigma_llt_.info() != Eigen::ComputationInfo::Success) {
			std::cerr << "[WARNING] Gaussian Process: Cholesky decomposition unsuccessful, maybe the "
									 "kernel is not PD? Error code: "
								<< sigma_llt_.info() << ", rcond: " << sigma_llt_.rcond() << std::endl;
		}
#ifndef NDEBUG
		std::cout << "[INFO] Gaussian Process: Cholesky decomposition rcond: " << sigma_llt_.rcond()
							<< std::endl;
#endif

		// Pre-solve sigma
		preSolveSigma(-1);
	}

	/**
	 * Calculate the nullspace action.
	 *
	 * @param x_query The query value.
	 * @param ks The input-data kernel evaluation.
	 * @return The nullspace action.
	 */
	virtual typename ManifoldOut::TangentVector calculateNsAction(
			const typename ManifoldIn::Vector &x_query, const typename ManifoldIn::MatrixX &ks) const {
		return ManifoldOut::TangentVector::Zero();
	}

	/**
	 * Function to calculate sigma after the mean has been determined.
	 *
	 * @param kss The input-input kernel evaluation.
	 * @param ks The input-data kernel evaluation.
	 * @return The covariance.
	 */
	virtual typename ManifoldOut::TangentMatrix calculateSigma(
			const typename ManifoldIn::MatrixX &kss, const typename ManifoldIn::MatrixX &ks) const {
		typename ManifoldIn::MatrixX v = sigma_llt_.matrixL().solve(ks.transpose());
		return 1 * (kss - v.transpose() * v);
	}
};
}  // namespace pglfd
