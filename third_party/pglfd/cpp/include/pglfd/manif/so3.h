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
#include <algorithm>  // IWYU pragma: keep
#include <cmath>      // IWYU pragma: keep
#include <map>
#include <numbers>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "pglfd/geom/so3.h"
#include "pglfd/manif/manifold.h"
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * SO(3) manifold implementation.
 *
 * @param Scalar_ The data type to use for this manifold.
 */
template<typename Scalar_>
class SO3Manifold : public Manifold<Scalar_, 4, 3, true> {
public:
	using Scalar = Scalar_;                                  //!< Scalar type used on the manifold.
	using Vector = typename Manifold<Scalar, 4, 3>::Vector;  //!< On-manifold vector type.
	using TangentVector = typename Manifold<Scalar, 4, 3>::TangentVector;   //!< Tangent vector type.
	using TangentMatrix = typename Manifold<Scalar, 4, 3>::TangentMatrix;   //!< Tangent matrix type.
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  //!< Dynamic matrix type.

	std::string name() const override {
		return "SO3";
	}

	Vector id_elem() const override {
		Vector id;
		id << 0, 0, 0, 1;
		return id;
	}

	Vector normalize(const Vector &vector) const override {
		return SO3<Scalar>(vector).normalize().toVec();
	}

	Vector exp(const TangentVector &tangent, const Vector &base) const override {
		return (SO3<Scalar>(base) * SO3<Scalar>::exp(tangent)).toVec();
	}

	TangentVector log(const Vector &point, const Vector &base) const override {
		return (SO3<Scalar>(base).inverse() * SO3<Scalar>(point)).log();
	}

	std::tuple<MatrixX, std::vector<int>> continuousLog(
			const MatrixX &tangents,
			const std::map<int, int> &vias = {},
			bool start_low = true) const override {
		// handle vias
		int first_via = tangents.cols();
		for(const auto &[key, value]: vias) {
			if(key < first_via)
				first_via = key;
		}

		// check vias
		for(int i = first_via; i < tangents.cols(); i++) {
			if(vias.count(i) == 0)
				throw std::runtime_error("Got normal point after a via point!");
		}

		MatrixX res = tangents;
		std::vector<int> discontinuities;
		for(int i = 0; i < 3; i++) {
			bool discont = false;
			for(int j = 1; j < first_via; j++) {
				auto testval = res(i, j) + 2 * std::numbers::pi;
				if(res(i, j) > 0)
					testval = res(i, j) - 2 * std::numbers::pi;
				if(std::abs(testval - res(i, j - 1)) < std::abs(res(i, j) - res(i, j - 1)))
					discont = true;
			}

			if(discont) {
				if(start_low && res(i, 0) > 0)
					res(i, 0) -= 2 * std::numbers::pi;
				else if(!start_low && res(i, 0) < 0)
					res(i, 0) += 2 * std::numbers::pi;

				for(int j = 1; j < first_via; j++) {
					auto oldval = res(i, j);
					auto testval = oldval + 2 * std::numbers::pi;
					if(oldval > 0)
						testval = oldval - 2 * std::numbers::pi;
					if(std::abs(testval - res(i, j - 1)) < std::abs(oldval - res(i, j - 1)))
						res(i, j) = testval;

					auto testval2 = res(i, j - 1) + 2 * std::numbers::pi;
					if(res(i, j - 1) > 0)
						testval2 = res(i, j - 1) - 2 * std::numbers::pi;
					if(std::abs(res(i, j) - testval2) < std::abs(res(i, j) - res(i, j - 1))) {
						discontinuities.push_back(j);
						if(std::abs(oldval - res(i, j - 1)) < std::abs(testval - res(i, j - 1)))
							res(i, j) = testval;
						else
							res(i, j) = oldval;
					}
				}
			}

			for(const auto &[j, value]: vias) {
				auto oldval = res(i, j);
				auto testval = oldval + 2 * std::numbers::pi;
				if(oldval > 0)
					testval = oldval - 2 * std::numbers::pi;
				if(std::abs(testval - res(i, value)) < std::abs(oldval - res(i, value)))
					res(i, j) = testval;
			}
		}
		return {res, discontinuities};
	}

	std::vector<Vector> action(
			const std::vector<Vector> &elems, const Vector &g, const Vector &h) const override {
		std::vector<Vector> res(elems.size());
		std::transform(
				execution::unseq, elems.begin(), elems.end(), res.begin(), [&](const auto &elem) {
					return (SO3<Scalar>(h) * SO3<Scalar>(g).inverse() * SO3<Scalar>(elem)).toVec();
				});
		return res;
	}

	TangentMatrix parallelTransportTrafo(const Vector &g, const Vector &h) const override {
		SO3<Scalar> trafo = SO3<Scalar>::exp(0.5 * (SO3<Scalar>(g).inverse() * SO3<Scalar>(h)).log());
		return trafo.toQuat().toRotationMatrix().transpose();
	}
};

using SO3Manifoldd = SO3Manifold<double>;
}  // namespace pglfd
