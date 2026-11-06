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
#include <algorithm>
#include <any>
#include <iostream>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>
#include "pglfd/geom/se3.h"
#include "pglfd/geom/so3.h"
#include "pglfd/manif/conversions.h"
#include "pglfd/manif/jacobian.h"
#include "pglfd/manif/product.h"
#include "pglfd/model/gaussian.h"  // IWYU pragma: keep
#include "pglfd/model/gaussian_regressor.h"
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * Riemannian task parameterization implementation :cite:t:`muehlbauer2026nonparametric`.
 */
template<typename ManifoldIn, typename ScalarOut>
class TaskParameterizedModel : public GaussianRegressor<ManifoldIn, SE3Manifold<ScalarOut>> {
public:
	/**
	 * Model origin.
	 */
	struct Origin {
		std::string name;                                        //!< Name of the origin.
		pglfd::SE3<typename ManifoldIn::Scalar>::Vector origin;  //!< Pose of the origin.
	};

	/**
	 * Constructor.
	 *
	 * @param origins SE(3) / position / orientation coordinate origins of the respective models in
	 * the "global origin".
	 * @param models The corresponding models.
	 */
	TaskParameterizedModel(
			const std::vector<Origin> &origins,
			const std::vector<std::shared_ptr<GaussianRegressorR3SO3<ManifoldIn, ScalarOut>>> &models) :
			origins_(origins), models_(models) {
		if(origins.size() != models.size()) {
			throw std::runtime_error("sizes of origins and models must agree!");
		}
	}

	/**
	 * Update an origin of the task-parameterized model.
	 *
	 * @param new_origin The new origin.
	 */
	void updateOrigin(const Origin &new_origin) {
		const auto it =
				std::find_if(execution::unseq, origins_.begin(), origins_.end(), [&](const auto &origin) {
					return origin.name.compare(new_origin.name) == 0;
				});
		if(it != origins_.end()) {
			it->origin = new_origin.origin;
			return;
		}
		throw std::runtime_error("Did not find model origin " + new_origin.name);
	}

	/**
	 * Get the number of models.
	 *
	 * @return The number of models.
	 */
	int getNumModels() const {
		return origins_.size();
	}

	/**
	 * Query the task-parameterized model for some output.
	 *
	 * @param x_query The query values.
	 * @param parameters A std::vector with parameters to be supplied to each method or empty.
	 * @return The calculated outputs.
	 */
	std::vector<Gaussian<SE3Manifold<ScalarOut>>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const override {
		// Prepare parameters
		std::vector<std::any> params(origins_.size());
		if(parameters.has_value()) {
			params = std::any_cast<const std::vector<std::any> &>(params);
		}

		// Prepare results
		std::vector<std::vector<Gaussian<SE3Manifold<ScalarOut>>>> model_gaussians(origins_.size());
		// Query all our models
		for(int i = 0; i < origins_.size(); i++) {
			// Query the model
			model_gaussians[i] = models_[i]->query_r3_so3(x_query, params[i]);

			// Get the tangent space rotation
			typename SE3Manifold<ScalarOut>::TangentMatrix A;
			A.setIdentity();
			A.template block<3, 3>(0, 0) =
					SO3<ScalarOut>(origins_[i].origin.tail(4)).toQuat().toRotationMatrix();

			std::for_each(
					execution::par_unseq,
					model_gaussians[i].begin(),
					model_gaussians[i].end(),
					[&](auto &gaussian) {
						gaussian.tangentAction(A);
						gaussian.action(origins_[i].origin);
					});
		}
		// Calculate the Gaussian product
		std::vector<Gaussian<SE3Manifold<ScalarOut>>> result_gaussians;
		result_gaussians.reserve(x_query.size());
		for(int i = 0; i < x_query.size(); i++) {
			std::vector<Gaussian<SE3Manifold<ScalarOut>>> gaussians;
			gaussians.reserve(origins_.size());
			for(int j = 0; j < origins_.size(); j++) {
				gaussians.push_back(model_gaussians[j][i]);
			}
			result_gaussians.emplace_back(GaussianProduct<SE3Manifold<ScalarOut>>(gaussians));
		}
		return result_gaussians;
	}

	/**
	 * Get the current origins.
	 *
	 * @return The origins.
	 */
	const std::vector<Origin> &getOrigins() const {
		return origins_;
	}

private:
	std::vector<Origin> origins_;
	std::vector<std::shared_ptr<GaussianRegressorR3SO3<ManifoldIn, ScalarOut>>> models_;
};

/**
 * Riemannian task parameterization implementation :cite:t:`muehlbauer2026nonparametric`.
 */
template<typename ManifoldIn, typename ManifoldOut>
class TaskParameterizedModelManif : public GaussianRegressor<ManifoldIn, ManifoldOut> {
public:
	/**
	 * Constructor.
	 *
	 * @param model_manif Model on our manifold of interest.
	 * @param tp_model Task-parameterized model containing the other models.
	 */
	TaskParameterizedModelManif(
			const std::shared_ptr<GaussianRegressor<ManifoldIn, ManifoldOut>> &model_manif,
			const std::shared_ptr<TaskParameterizedModel<ManifoldIn, typename ManifoldOut::Scalar>>
					&tp_model) :
			model_manif_(model_manif), tp_model_(tp_model) {}

	/**
	 * Update an origin of the task-parameterized model.
	 *
	 * @param new_origin The new origin.
	 */
	void updateOrigin(
			const typename TaskParameterizedModel<ManifoldIn, typename ManifoldOut::Scalar>::Origin
					&new_origin) {
		tp_model_->updateOrigin(new_origin);
	}

	/**
	 * Query the task-parameterized model for some output.
	 *
	 * @param x_query The query values.
	 * @param parameters A std::vector with parameters to be supplied to each method or empty.
	 * @return The calculated outputs.
	 */
	std::vector<Gaussian<ManifoldOut>> operator()(
			const std::vector<typename ManifoldIn::Vector> &x_query,
			const std::any &parameters = {}) const override {
		// Prepare parameters
		std::vector<std::any> params(tp_model_->getNumModels() + 1);
		if(parameters.has_value()) {
			params = std::any_cast<const std::vector<std::any> &>(params);
		}

		// Get the manifold results
		std::vector<std::vector<Gaussian<ManifoldOut>>> model_gaussians;
		model_gaussians.push_back(model_manif_->operator()(x_query, params[0]));

		// Get the other model results + transform
		std::vector<Gaussian<SE3Manifold<typename ManifoldOut::Scalar>>> tp_res =
				tp_model_->operator()(x_query, std::span(params).subspan(0, params.size()));
		model_gaussians.push_back({});
		for(const auto &gaussian: tp_res) {
			typename ManifoldOut::Vector mu_man = manifold_from_r3_so3<ManifoldOut>(gaussian.mu_);
			typename ManifoldOut::TangentMatrix jacobian = manifold_jacobian<ManifoldOut>(mu_man);
			typename ManifoldOut::TangentMatrix sigma_man =
					jacobian * gaussian.sigma_ * jacobian.transpose();
			model_gaussians.back().push_back(Gaussian<ManifoldOut>(mu_man, sigma_man));
		}

		// Calculate the Gaussian product
		std::vector<Gaussian<ManifoldOut>> result_gaussians;
		result_gaussians.reserve(x_query.size());
		for(int i = 0; i < x_query.size(); i++) {
			std::vector<Gaussian<ManifoldOut>> gaussians;
			gaussians.reserve(2);
			for(int j = 0; j < 2; j++) {
				gaussians.push_back(model_gaussians[j][i]);
			}
			result_gaussians.emplace_back(GaussianProduct<ManifoldOut>(gaussians));
		}
		return result_gaussians;
	}

	/**
	 * Get the current origins.
	 *
	 * @return The origins.
	 */
	std::vector<typename TaskParameterizedModel<ManifoldIn, typename ManifoldOut::Scalar>::Origin> &
			getOrigins() const {
		return tp_model_.getOrigins();
	}

private:
	std::shared_ptr<GaussianRegressor<ManifoldIn, ManifoldOut>> model_manif_;
	std::shared_ptr<TaskParameterizedModel<ManifoldIn, typename ManifoldOut::Scalar>> tp_model_;
};
}  // namespace pglfd
