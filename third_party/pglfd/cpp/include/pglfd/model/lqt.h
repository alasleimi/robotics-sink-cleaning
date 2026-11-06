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
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <vector>
#include "pglfd/model/gaussian.h"

namespace pglfd {
/**
 * Riemannian LQT implementation.
 *
 * @param Manifold The output data manifold.
 */
template<typename Manifold>
class LQT {
public:
	/**
	 * Constructor.
	 *
	 * @param initial_state Initial state.
	 * @param t_vias Timestamps of via points.
	 * @param vias Via points.
	 * @param tend End timepoint.
	 * @param dt Time step.
	 * @param rfactor R factor.
	 * @param initial_velocity The initial velocity.
	 */
	LQT(const typename Manifold::Vector &initial_state,
			const std::vector<typename Manifold::Scalar> &t_vias,
			const std::vector<Gaussian<Manifold>> &vias,
			typename Manifold::Scalar tend,
			typename Manifold::Scalar dt,
			typename Manifold::Scalar rfactor,
			const typename Manifold::TangentVector &initial_velocity = Manifold::TangentVector::Zero()) :
			LQT(initial_state,
					t_vias,
					vias,
					tend,
					dt,
					Manifold::TangentVector::Ones() * rfactor,
					initial_velocity) {}

	/**
	 * Constructor.
	 *
	 * @param initial_state Initial state.
	 * @param t_vias Timestamps of via points.
	 * @param vias Via points.
	 * @param tend End timepoint.
	 * @param dt Time step.
	 * @param rfactor R factor.
	 * @param initial_velocity The initial velocity.
	 */
	LQT(const typename Manifold::Vector &initial_state,
			const std::vector<typename Manifold::Scalar> &t_vias,
			const std::vector<Gaussian<Manifold>> &vias,
			typename Manifold::Scalar tend,
			typename Manifold::Scalar dt,
			typename Manifold::TangentVector rfactor,
			const typename Manifold::TangentVector &initial_velocity = Manifold::TangentVector::Zero()) :
			initial_state_(initial_state),
			initial_velocity_(initial_velocity),
			t_vias_(t_vias),
			vias_(vias),
			rfactor_(rfactor),
			dt_(dt),
			last_cost_(0),
			last_len_(0) {
		if(t_vias.size() != vias.size()) {
			throw std::runtime_error("sizes of vias and their timestamps must agree!");
		}

		// preallocate timestamps
		for(int i = 0; i < tend / dt + 1; i++) {
			ts_.push_back(i * dt);
		}

		// calculate via idxs
		for(int i = 0; i < t_vias_.size(); i++) {
			idx_vias_.push_back(std::round(t_vias_[i] / dt_));
		}

		// reserve space for comands and states
		commands_ = Manifold::MatrixX::Zero(Manifold::dim_tangent, ts_.size() - 1);
		velocities_ = typename Manifold::MatrixX(Manifold::dim_tangent, ts_.size());
		poses_ = typename Manifold::MatrixX(Manifold::dim_manifold, ts_.size());

		// allocate su, precompute q
		su_ = Manifold::MatrixX::Zero(
				ts_.size() * Manifold::dim_tangent, (ts_.size() - 1) * Manifold::dim_tangent);
		su_reduced_ = Manifold::MatrixX::Zero(
				t_vias.size() * Manifold::dim_tangent, (ts_.size() - 1) * Manifold::dim_tangent);
		q_ = Manifold::MatrixX::Zero(
				t_vias.size() * Manifold::dim_tangent, t_vias.size() * Manifold::dim_tangent);
		for(int i = 0; i < t_vias_.size(); i++) {
			int idx_start = i * Manifold::dim_tangent;
			auto sel = Eigen::seq(idx_start, idx_start + Manifold::dim_tangent - 1);
			via_precisions_.push_back(vias_[i].sigma_.inverse());
			q_(sel, sel) = via_precisions_.back();
		}

		// precompute r
		r_ = Manifold::MatrixX::Zero(
				(ts_.size() - 1) * Manifold::dim_tangent, (ts_.size() - 1) * Manifold::dim_tangent);
		for(int i = 0; i < ts_.size() - 1; i++) {
			r_.template block<Manifold::dim_tangent, Manifold::dim_tangent>(
					i * Manifold::dim_tangent, i * Manifold::dim_tangent) = rfactor_.asDiagonal();
		}
	}

	virtual ~LQT() {}

	/**
	 * Fit the LQT.
	 *
	 * @param eps Epsilon value.
	 * @param maxit Maximum number of iterations.
	 * @return The number of performed iterations.
	 */
	int fit(typename Manifold::Scalar eps = 1e-8, int maxit = 50) {
		int it = 0;
		std::tie(last_cost_, last_len_) = calcCost(commands_);
		do {
			rollout(poses_, velocities_, commands_);
			it += 1;
			if(it >= maxit) {
				throw std::runtime_error("LQT did not converge in " + std::to_string(maxit) + " steps");
			}
		} while(calcCommands() > eps);

		calcCovariance();

		return it - 1;  // align iterations with Python
	}

	/**
	 * Get the time steps.
	 *
	 * @return dt.
	 */
	const typename Manifold::Scalar getDt() const {
		return dt_;
	}

	/**
	 * Get the states.
	 *
	 * @return The stored states.
	 */
	const std::vector<Gaussian<Manifold>> getStates() const {
		return states_;
	}

	/**
	 * Get the poses.
	 *
	 * @return The stored poses.
	 */
	const typename Manifold::MatrixX getPoses() const {
		return poses_.transpose();
	}

	/**
	 * Get the velocities.
	 *
	 * @return The stored velocities.
	 */
	const typename Manifold::MatrixX getVelocities() const {
		return velocities_.transpose();
	}

	typename Manifold::MatrixX commands_;  //< The commands of this LQT

protected:
	// cppcheck-suppress constParameterReference
	void calcSu(typename Manifold::MatrixX &su, auto &&ivals, auto &&idx_states, bool to_k = false) {
#pragma omp parallel for
		for(const auto &[i, idx_state]: std::views::zip(ivals, idx_states)) {
			int idx_start = i * Manifold::dim_tangent;
			auto sel_i = Eigen::seq(idx_start, idx_start + Manifold::dim_tangent - 1);
#pragma omp parallel for
			for(int j = 0; j < idx_state - 1; j++) {
				int jdx_start = j * Manifold::dim_tangent;
				auto sel_j = Eigen::seq(jdx_start, jdx_start + Manifold::dim_tangent - 1);
				typename Manifold::TangentMatrix trafo = Manifold::TangentMatrix::Identity();
				for(int k = j + 1; k < idx_state + 1; k++) {
					trafo = manif_.parallelTransportTrafo(poses_(Eigen::all, k - 1), poses_(Eigen::all, k))
									* trafo;
				}

				// final parallel transport to the via point
				if(to_k) {
					trafo =
							manif_.parallelTransportTrafo(poses_(Eigen::all, idx_state), vias_[i].mu_) * trafo;
				}
				su(sel_i, sel_j) = trafo * (ts_[idx_state] - ts_[j] - dt_) * dt_;
			}
		}
	}

	void calcCovariance() {
		// full su
		calcSu(su_, std::views::iota(1, (int)ts_.size()), std::views::iota(1, (int)ts_.size()));

		// covariance
		typename Manifold::MatrixX v = llt_cov_.matrixL().solve(su_.transpose());
		typename Manifold::MatrixX cov = v.transpose() * v;
		states_.clear();
		states_.reserve(ts_.size() - 1);
		for(int i = 1; i < ts_.size(); i++) {
			int idx_start = i * Manifold::dim_tangent;
			auto sel = Eigen::seq(idx_start, idx_start + Manifold::dim_tangent - 1);
			states_.emplace_back(poses_(Eigen::all, i), cov(sel, sel));
		}
	}

	void rollout(
			typename Manifold::MatrixX &poses,       // cppcheck-suppress constParameterReference
			typename Manifold::MatrixX &velocities,  // cppcheck-suppress constParameterReference
			const typename Manifold::MatrixX &commands) {
		poses(Eigen::all, 0) = initial_state_;
		velocities(Eigen::all, 0) = initial_velocity_;
		for(int i = 1; i < ts_.size(); i++) {
			// next pose
			poses(Eigen::all, i) =
					manif_.exp(velocities(Eigen::all, i - 1) * dt_, poses(Eigen::all, i - 1));
			// next velocity
			velocities(Eigen::all, i) = manif_.parallelTransportVector(
					velocities(Eigen::all, i - 1) + commands(Eigen::all, i - 1) * dt_,
					poses(Eigen::all, i - 1),
					poses(Eigen::all, i));
		}
	}

	std::tuple<typename Manifold::Scalar, typename Manifold::Scalar> calcCost(
			const typename Manifold::MatrixX &commands) {
		// initial rollout
		typename Manifold::MatrixX poses(poses_.rows(), poses_.cols());
		typename Manifold::MatrixX velocities(velocities_.rows(), velocities_.cols());
		rollout(poses, velocities, commands);

		// calculate via point cost
		typename Manifold::VectorX cost(vias_.size());
		for(const auto &[i, idx_state]:
				std::views::zip(std::views::iota(0, (int)t_vias_.size()), idx_vias_)) {
			typename Manifold::TangentVector logval =
					manif_.log(poses(Eigen::all, idx_state), vias_[i].mu_);
			cost[i] = (logval.transpose() * via_precisions_[i] * logval).value();
		}

		// calculate total cost
		typename Manifold::Scalar total_cost =
				cost.sum()
				+ (commands.reshaped((ts_.size() - 1) * Manifold::dim_tangent, 1).transpose() * r_
					 * commands.reshaped((ts_.size() - 1) * Manifold::dim_tangent, 1))
							.value();

		// calculate total len
		typename Manifold::Scalar total_len = 0;
		for(int i = 0; i < poses.cols() - 1; i++) {
			total_len += (velocities(Eigen::all, i) * dt_).squaredNorm();
		}

		return {total_cost, total_len};
	}

	typename Manifold::Scalar calcCommands() {
		// reduced su
		calcSu(su_reduced_, std::views::iota(0, (int)t_vias_.size()), idx_vias_, true);

		// preallocate matrix
		typename Manifold::MatrixX logs;
		logs.resize(Manifold::dim_tangent, t_vias_.size());

		// fill logs
		for(int i = 0; i < t_vias_.size(); i++) {
			int idx_state = std::round(t_vias_[i] / dt_);
			logs.template block<Manifold::dim_tangent, 1>(0, i) =
					manif_.log(poses_(Eigen::all, idx_state), vias_[i].mu_);
		}

		// Calculate the update
		llt_cov_ = (su_reduced_.transpose() * q_ * su_reduced_ + r_).llt();
		typename Manifold::MatrixX eps = -llt_cov_.solve(
				su_reduced_.transpose() * q_ * logs.reshaped(Manifold::dim_tangent * t_vias_.size(), 1)
				+ r_ * commands_.reshaped((ts_.size() - 1) * Manifold::dim_tangent, 1));

		// Line search
		auto [cur_cost, cur_len] =
				calcCost(commands_ + eps.reshaped(Manifold::dim_tangent, ts_.size() - 1));
		typename Manifold::Scalar scaler = 1.;
		while(cur_cost > last_cost_ && cur_len > last_len_ * 2) {
			scaler /= 2.;
			std::tie(cur_cost, cur_len) =
					calcCost(commands_ + scaler * eps.reshaped(Manifold::dim_tangent, ts_.size() - 1));
		}
		last_cost_ = cur_cost;
		last_len_ = cur_len;

		// update and return
		commands_ += scaler * eps.reshaped(Manifold::dim_tangent, ts_.size() - 1);
		return eps.norm() / (ts_.size() - 1);
	}

	// Manifold stuff
	Manifold manif_;  //!< The data manifold.

	// Parameters & Data
	typename Manifold::Vector initial_state_;
	typename Manifold::TangentVector initial_velocity_;
	std::vector<typename Manifold::Scalar> t_vias_;
	std::vector<int> idx_vias_;
	std::vector<Gaussian<Manifold>> vias_;
	typename Manifold::TangentVector rfactor_;
	typename Manifold::Scalar dt_;
	std::vector<typename Manifold::Scalar> ts_;

	// States
	typename Manifold::MatrixX velocities_;
	typename Manifold::MatrixX poses_;
	std::vector<Gaussian<Manifold>> states_;

	// su, q, r
	typename Manifold::MatrixX su_, su_reduced_;
	typename Manifold::MatrixX q_;
	typename Manifold::MatrixX r_;

	// Rollout cost
	typename Manifold::Scalar last_cost_, last_len_;
	std::vector<typename Manifold::TangentMatrix> via_precisions_;

	// Cholesky decomposition of cov
	Eigen::LLT<typename Manifold::MatrixX> llt_cov_;
};
}  // namespace pglfd
