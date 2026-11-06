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
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include "pglfd/manif/manifold.h"
#include "pglfd/util/execution.h"

namespace pglfd {
/**
 * Euclidean manifold implementation.
 *
 * @param Scalar_ The data type to use for this manifold.
 * @param dim The dimension of the manifold.
 */
template<typename Scalar_, int dim>
class EuclideanManifold : public Manifold<Scalar_, dim, dim> {
public:
	using Scalar = Scalar_;  //!< Scalar type used on the manifold.
	using Vector = typename Manifold<Scalar, dim, dim>::Vector;  //!< On-manifold vector type.
	using TangentVector =
			typename Manifold<Scalar, dim, dim>::TangentVector;  //!< Tangent vector type.
	using TangentMatrix =
			typename Manifold<Scalar, dim, dim>::TangentMatrix;                 //!< Tangent matrix type.
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  //!< Dynamic matrix type.

	std::string name() const override {
		return "R" + std::to_string(dim);
	}

	Vector id_elem() const override {
		return Vector::Zero();
	}

	Vector normalize(const Vector &vector) const override {
		return vector;
	}

	Vector exp(const TangentVector &tangent, const Vector &base) const override {
		return tangent + base;
	}

	TangentVector log(const Vector &point, const Vector &base) const override {
		return point - base;
	}

	std::tuple<MatrixX, std::vector<int>> continuousLog(
			const MatrixX &tangents,
			const std::map<int, int> &vias = {},
			bool start_low = true) const override {
		return {tangents, {}};
	}

	std::vector<Vector> action(
			const std::vector<Vector> &elems, const Vector &g, const Vector &h) const override {
		std::vector<Vector> res(elems.size());
		std::transform(
				execution::unseq, elems.begin(), elems.end(), res.begin(), [&](const auto &elem) {
					return elem - g + h;
				});
		return res;
	}

	TangentMatrix parallelTransportTrafo(const Vector &g, const Vector &h) const override {
		return TangentMatrix::Identity();
	}

	TangentVector parallelTransportVector(
			const TangentVector &vec, const Vector &g, const Vector &h) const override {
		return vec;
	}
	TangentMatrix parallelTransportMatrix(
			const TangentMatrix &mat, const Vector &g, const Vector &h) const override {
		return mat;
	}
};

using EuclideanManifold1d = EuclideanManifold<double, 1>;
using EuclideanManifold2d = EuclideanManifold<double, 2>;
using EuclideanManifold3d = EuclideanManifold<double, 3>;
using EuclideanManifold6d = EuclideanManifold<double, 6>;
}  // namespace pglfd
