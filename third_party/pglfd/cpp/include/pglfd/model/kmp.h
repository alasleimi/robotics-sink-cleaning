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
#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>
#include "pglfd/model/gp.h"
#include "pglfd/model/kernel.h"  // IWYU pragma: keep
#include "pglfd/util/eigen_extensions.h"
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * Riemannian KMP implementation. The KMP can be queried just like a GaussianProcess.
 */
template<typename ManifoldIn, typename ManifoldOut>
class KMP : public GaussianProcess<ManifoldIn, ManifoldOut> {
public:
	/**
	 * Constructor.
	 *
	 * @param x Input values.
	 * @param y Output values.
	 * @param kernel The kernel to compute distances between query values and data points.
	 * @param covariances Covariance matrices associated with the output values.
	 * @param lambda1 Lambda 1 parameter of the KMP.
	 * @param lambda2 Lambda 2 parameter of the KMP.
	 * @param alpha Alpha parameter of the KMP.
	 */
	KMP(const std::vector<typename ManifoldIn::Vector> &x,
			const std::vector<typename ManifoldOut::Vector> &y,
			const std::shared_ptr<MatrixKernel<ManifoldIn, ManifoldOut>> &kernel,
			const std::vector<typename ManifoldOut::TangentMatrix> &covariances,
			typename ManifoldIn::Scalar lambda1 = 0.1,
			typename ManifoldIn::Scalar lambda2 = 1,
			typename ManifoldIn::Scalar alpha = 1) :
			lambda1_(lambda1),
			lambda2_(lambda2),
			alpha_(alpha),
			covariances_(covariances),
			ns_use_k_underbar_(true),
			ns_decorrelate_(false),
			GaussianProcess<ManifoldIn, ManifoldOut>(x, y, kernel, 0, false) {
		if(covariances.size() != x.size()) {
			throw std::runtime_error("sizes of input data and covariance must agree!");
		}

		// Precompute what can be precomputed
		this->updateK();
	}
	virtual ~KMP() {}

	/**
	 * Append measurements to the KMP.
	 *
	 * @param x x values.
	 * @param y y values.
	 * @param covariances Covariances corresponding to y values.
	 */
	void appendMeasurements(
			const std::vector<typename ManifoldIn::Vector> &x,
			const std::vector<typename ManifoldOut::Vector> &y,
			const std::vector<typename ManifoldOut::TangentMatrix> &covariances) {
		// Update via points
		for(int i = 0; i < x.size(); i++) {
			std::vector<typename ManifoldIn::Scalar> distvals(this->x_.size());
			std::transform(
					execution::unseq,
					this->x_.begin(),
					this->x_.end(),
					distvals.begin(),
					[&](const auto &xs) { return this->manif_in_.log(x[i], xs).squaredNorm(); });
			this->vias_[i + this->x_.size()] =
					std::distance(distvals.begin(), std::min_element(distvals.begin(), distvals.end()));
		}

		// Append data
		this->x_.insert(this->x_.end(), x.begin(), x.end());
		this->y_.insert(this->y_.end(), y.begin(), y.end());
		covariances_.insert(covariances_.end(), covariances.begin(), covariances.end());

		// Update K matrix
		this->updateK(this->x_.size() - x.size(), {});
	}

	/**
	 * Remove measurements from a KMP.
	 *
	 * @param measurements_idx Measurements to remove.
	 */
	void removeMeasurements(std::vector<int> measurements_idx) {
		// Check if there is something to do
		if(measurements_idx.size() > 0) {
			// First sort measurements idx
			std::sort(measurements_idx.begin(), measurements_idx.end());
			measurements_idx.erase(
					std::unique(measurements_idx.begin(), measurements_idx.end()), measurements_idx.end());

			// Check for in range
			if(measurements_idx.front() < 0 || measurements_idx.front() >= this->x_.size()
				 || measurements_idx.back() >= this->x_.size()) {
				throw std::runtime_error("Trying to remove measurements that do not exist.");
			}

			// Update via points
			for(int i = 0; i < measurements_idx.size(); i++) {
				if(this->vias_.count(measurements_idx[i]))
					this->vias_.erase(measurements_idx[i]);
				else {
					std::vector<int> keys;
					keys.reserve(this->vias_.size());
					for(const auto &[key, value]: this->vias_) {
						keys.push_back(key);
					}
					for(const auto key: keys) {
						if(this->vias_[key] > measurements_idx[i] - i)
							this->vias_[key]--;
						if(key > measurements_idx[i] - i) {
							this->vias_[key - 1] = this->vias_[key];
							this->vias_.erase(key);
						}
					}
				}
			}

			// Remove data
			for(int i = 0; i < measurements_idx.size(); i++) {
				this->x_.erase(this->x_.begin() + measurements_idx[i] - i);
				this->y_.erase(this->y_.begin() + measurements_idx[i] - i);
				this->covariances_.erase(this->covariances_.begin() + measurements_idx[i] - i);
			}

			// Update K matrix
			this->updateK(-1, measurements_idx);
		}
	}

	/**
	 * remove the last measurement from a KMP.
	 */
	void removeLastMeasurement() {
		removeMeasurements({static_cast<int>(this->x_.size()) - 1});
	}

	/**
	 * Set ns parameters.
	 * @param ns_use_k_underbar Correlate between ns actions.
	 * @param ns_decorrelate: Decorrelate DoFs.
	 */
	void setNsParams(bool ns_use_k_underbar, bool ns_decorrelate) {
		ns_use_k_underbar_ = ns_use_k_underbar;
		ns_decorrelate_ = ns_decorrelate;

		// Update nullspace action
		this->updateNsAction();
	}

	/**
	 * Add nullspace actions to the KMP.
	 *
	 * @param x x values.
	 * @param y y values.
	 */
	void appendNsAction(
			const std::vector<typename ManifoldIn::Vector> &x,
			const std::vector<typename ManifoldOut::TangentVector> &y) {
		// Append data
		this->x_ns_.insert(this->x_ns_.end(), x.begin(), x.end());
		this->y_ns_.insert(this->y_ns_.end(), y.begin(), y.end());

		// Update nullspace action
		this->updateNsAction();
	}

	/**
	 * Remove nullspace actions from a KMP.
	 *
	 * @param remove_idx Measurements to remove.
	 */
	void removeNsAction(std::vector<int> remove_idx) {
		// Return if there is nothing to do
		if(remove_idx.size() == 0) {
			return;
		}

		// First sort measurements idx
		std::sort(remove_idx.begin(), remove_idx.end());
		remove_idx.erase(std::unique(remove_idx.begin(), remove_idx.end()), remove_idx.end());

		// Check for in range
		if(remove_idx.front() < 0 || remove_idx.front() >= this->x_ns_.size()
			 || remove_idx.back() >= this->x_ns_.size()) {
			throw std::runtime_error("Trying to remove nullspace actions that do not exist.");
		}

		// Remove data
		for(int i = 0; i < remove_idx.size(); i++) {
			this->x_ns_.erase(this->x_ns_.begin() + remove_idx[i] - i);
			this->y_ns_.erase(this->y_ns_.begin() + remove_idx[i] - i);
		}

		// Update nullspace action
		this->updateNsAction();
	}

	/**
	 * Get sigma ii inv2.
	 *
	 * @return Sigma ii inv2.
	 */
	typename ManifoldIn::MatrixX getSigmaIiInv2() const {
		return sigma2_llt_.solve(
				ManifoldIn::MatrixX::Identity(sigma2_llt_.matrixL().rows(), sigma2_llt_.matrixL().cols()));
	}

protected:
	// precomputed values
	Eigen::UpdateLLT<typename ManifoldIn::MatrixX> sigma2_llt_;  //!< Cholesky decomposition of
																															 //!< sigma2.

	// overridden functions
	void updateK(int append_after = -1, const std::vector<int> &remove_idxs = {}) override {
		// mark matrices for recomputation
		this->had_inverse_update_ = true;

		// Calculate dims
		int k_dim = this->x_.size() * this->manif_out_.dim_tangent;

		if(append_after > 0) {
			// Calculate dims
			int append_idx = append_after * this->manif_out_.dim_tangent;
			auto new_sel = Eigen::seq(append_idx, Eigen::last);

			// Calculate what should be appended to K
			typename ManifoldIn::MatrixX append_k = this->applyKernel(
					this->x_,
					std::span<typename ManifoldIn::Vector>(this->x_.begin() + append_after, this->x_.end()));

			// Calculate what should be appended to sigma
			typename ManifoldIn::MatrixX append_sigma =
					ManifoldIn::MatrixX::Zero(append_k.cols(), append_k.cols());
			for(int i = append_after; i < covariances_.size(); i++) {
				append_sigma.template block<ManifoldOut::dim_tangent, ManifoldOut::dim_tangent>(
						ManifoldOut::dim_tangent * (i - append_after),
						ManifoldOut::dim_tangent * (i - append_after)) = covariances_[i];
			}

			typename ManifoldIn::MatrixX update_llt = append_k;
			update_llt.block(append_idx, 0, append_sigma.cols(), append_sigma.cols()) +=
					lambda1_ * append_sigma;
			this->sigma_llt_.extendMatrix(update_llt);
			update_llt = append_k;
			update_llt.block(append_idx, 0, append_sigma.cols(), append_sigma.cols()) +=
					lambda2_ * append_sigma;
			sigma2_llt_.extendMatrix(update_llt);
		} else if(remove_idxs.size() > 0) {
			// Calculate all idxs to remove
			std::vector<int> all_remove_idxs;
			all_remove_idxs.reserve(remove_idxs.size() * ManifoldOut::dim_tangent);
			for(int remove_idx: remove_idxs) {
				for(int j = 0; j < ManifoldOut::dim_tangent; j++) {
					all_remove_idxs.push_back(remove_idx * ManifoldOut::dim_tangent + j);
				}
			}

			// Check if anything needs to be reordered
			bool need_reorder = all_remove_idxs[0] < k_dim;

			// Calculate reorderings
			std::vector<int> reorderings;
			if(need_reorder) {
				int old_k_dim = k_dim + remove_idxs.size() * ManifoldOut::dim_tangent;
				reorderings.reserve(old_k_dim);
				int cur_remove = 0;
				for(int i = 0; i < old_k_dim; i++) {
					if(cur_remove >= all_remove_idxs.size() || i < all_remove_idxs[cur_remove]) {
						reorderings.push_back(i);
					} else {
						cur_remove++;
					}
				}
				for(int i = 0; i < all_remove_idxs.size(); i++) {
					reorderings.push_back(all_remove_idxs[i]);
				}
			}

			this->sigma_llt_.reorderDelete(reorderings, k_dim);
			sigma2_llt_.reorderDelete(reorderings, k_dim);
		} else {
			// K matrix
			this->k_ = this->applyKernel(this->x_, this->x_);
			this->had_inverse_update_ = false;

			// blockdiagonal matrix of covariances
			this->sigma_block_ = ManifoldIn::MatrixX::Identity(this->k_.rows(), this->k_.cols());
			for(int i = 0; i < covariances_.size(); i++) {
				this->sigma_block_.template block<ManifoldOut::dim_tangent, ManifoldOut::dim_tangent>(
						ManifoldOut::dim_tangent * i, ManifoldOut::dim_tangent * i) = covariances_[i];
			}

			// Cholesky update
			this->sigma_llt_ = (this->k_ + lambda1_ * this->sigma_block_).llt();
			sigma2_llt_ = (this->k_ + lambda2_ * this->sigma_block_).llt();
			if(this->sigma_llt_.info() != Eigen::ComputationInfo::Success
				 || sigma2_llt_.info() != Eigen::ComputationInfo::Success) {
				std::cerr << "[WARNING] KMP: Cholesky decomposition unsuccessful, maybe the kernel is "
										 "not PD? Error codes: "
									<< this->sigma_llt_.info() << ", " << sigma2_llt_.info()
									<< ", rconds: " << this->sigma_llt_.rcond() << ", " << sigma2_llt_.rcond()
									<< std::endl;
#ifndef NDEBUG
				std::cout << "[INFO] KMP: Cholesky decomposition rconds: " << this->sigma_llt_.rcond()
									<< ", " << sigma2_llt_.rcond() << std::endl;
#endif
			}
		}

		// Pre-solve sigma
		this->preSolveSigma(append_after);

		// Calculate ns action
		updateNsAction();
	}

	/**
	 * Update precalculations of the nullspace action.
	 */
	void updateNsAction() {
		if(x_ns_.size() > 0) {
			ns_solve_ = this->sigma_llt_.solve(this->applyKernel(this->x_, x_ns_));

			k_underbar_solve_.resize(ManifoldOut::dim_tangent);
			for(int i = 0; i < ManifoldOut::dim_tangent; i++) {
				if(i > 0 && !ns_decorrelate_)
					break;

				Eigen::Matrix<typename ManifoldIn::Scalar, Eigen::Dynamic, 1> y_ns_vec(
						y_ns_.size() * ManifoldOut::dim_tangent, 1);

				// Calculate the update
				for(int j = 0; j < y_ns_.size(); j++) {
					if(ns_decorrelate_) {
						y_ns_vec.template block<ManifoldOut::dim_tangent, 1>(j * ManifoldOut::dim_tangent, 0)
								.setZero();
						y_ns_vec(j * ManifoldOut::dim_tangent + i, 0) = y_ns_[j][i];
					} else {
						y_ns_vec.template block<ManifoldOut::dim_tangent, 1>(j * ManifoldOut::dim_tangent, 0) =
								y_ns_[j];
					}
				}
				if(ns_use_k_underbar_)
					k_underbar_solve_[i] = this->applyKernel(x_ns_, x_ns_).inverse() * y_ns_vec;
				else
					k_underbar_solve_[i] = y_ns_vec;
			}
		}
	}

	typename ManifoldOut::TangentVector calculateNsAction(
			const typename ManifoldIn::Vector &x_query,
			const typename ManifoldIn::MatrixX &ks) const override {
		if(x_ns_.size() > 0) {
			typename ManifoldIn::MatrixX k_hat =
					this->applyKernel(std::vector<typename ManifoldIn::Vector>{x_query}, x_ns_);
			if(ns_decorrelate_) {
				typename ManifoldOut::TangentVector res;
				for(int i = 0; i < ManifoldOut::dim_tangent; i++) {
					res[i] = ((k_hat - ks * ns_solve_) * k_underbar_solve_[i])(i, 0);
				}
				return res;
			} else
				return (k_hat - ks * ns_solve_) * k_underbar_solve_[0];
		}
		return ManifoldOut::TangentVector::Zero();
	}

	typename ManifoldOut::TangentMatrix calculateSigma(
			const typename ManifoldIn::MatrixX &kss,
			const typename ManifoldIn::MatrixX &ks) const override {
		typename ManifoldIn::MatrixX v = sigma2_llt_.matrixL().solve(ks.transpose());
		return alpha_ * (kss - v.transpose() * v);
	}

private:
	typename ManifoldIn::Scalar lambda1_, lambda2_, alpha_;
	std::vector<typename ManifoldOut::TangentMatrix> covariances_;

	// Nullspace actions
	std::vector<typename ManifoldIn::Vector> x_ns_;
	std::vector<typename ManifoldOut::TangentVector> y_ns_;
	typename ManifoldIn::MatrixX ns_solve_;
	std::vector<typename ManifoldIn::MatrixX> k_underbar_solve_;
	bool ns_use_k_underbar_, ns_decorrelate_;
};
}  // namespace pglfd
