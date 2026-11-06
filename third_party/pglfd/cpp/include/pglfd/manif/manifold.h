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
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace pglfd {
/**
 * Abstract base class for a Riemannian Manifold with placeholders for standard manifold operations.
 *
 * @param Scalar_ The data type to use for this manifold.
 * @param dim_manifold_ The dimension of on-manifold vectors.
 * @param dim_tangent_ The dimension of vectors in the tangent space.
 */
template<typename Scalar_, int dim_manifold_, int dim_tangent_, bool discont_log_alt_ = false>
class Manifold {
public:
	using Scalar = Scalar_;                                  //!< Scalar type used on the manifold.
	using Vector = Eigen::Matrix<Scalar, dim_manifold_, 1>;  //!< On-manifold vector type.
	using TangentVector = Eigen::Matrix<Scalar, dim_tangent_, 1>;  //!< Tangent vector type.
	using TangentMatrix =
			Eigen::Matrix<Scalar, dim_tangent_, dim_tangent_>;                  //!< Tangent matrix type.
	using VectorX = Eigen::Matrix<double, Eigen::Dynamic, 1>;               //!< Dynamic vector type.
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  //!< Dynamic matrix type.

	// Sizes
	static constexpr int dim_manifold = dim_manifold_;  //!< Dimension of a on-manifold element.
	static constexpr int dim_tangent = dim_tangent_;    //!< Dimension of a tangent element.

	// Infos
	static constexpr bool discont_log_alt =
			discont_log_alt_;  //!< Alternatives for discontinuous log.

	// Methods
	/**
	 * Get the manifold name.
	 *
	 * @return The manifold name.
	 */
	virtual std::string name() const = 0;

	/**
	 * Get the id element.
	 *
	 * @return The id element.
	 */
	virtual Vector id_elem() const = 0;

	/**
	 * Normalize a vector.
	 *
	 * @param vector The vector to normalize.
	 * @return The normalized vector.
	 */
	virtual Vector normalize(const Vector &vector) const = 0;

	/**
	 * Exponentiate a tangent vector at base.
	 *
	 * @param tangent The tangent vector to exponentiate.
	 * @param base The point at which to perform the exponential.
	 * @return The exponentiated element (point on manifold).
	 */
	virtual Vector exp(const TangentVector &tangent, const Vector &base) const = 0;

	/**
	 * Take to logarithm of a on-manifold point at the base.
	 *
	 * @param point The point to take the manifold of.
	 * @param base The point at which to perform the logarithm.
	 * @return The tangent vector.
	 */
	virtual TangentVector log(const Vector &point, const Vector &base) const = 0;

	/**
	 * Try to move discontinuities away on compact manifolds.
	 *
	 * @param tangents The initial logarithm values.
	 * @param vias Via points, just order them closest to the other points.
	 * @param start_low Start with low (vs high) values for the log.
	 * @return The wrapped logarithm value
	 */
	virtual std::tuple<MatrixX, std::vector<int>> continuousLog(
			const MatrixX &tangents,
			const std::map<int, int> &vias = {},
			bool start_low = true) const = 0;

	/**
	 * Perform an action on manifold elements: move elems relative to g to elems relative to h.
	 *
	 * @param elems The manifold elements to transform, dim_tan = manifold dimension.
	 * @param g The point in the manifold from which to transform, dim_man = manifold dimension.
	 * @param h The point in the manifold to which to transform, dim_man = manifold dimension.
	 * @return Transformed manifold elements, dim_man = manifold dimension.
	 */
	virtual std::vector<Vector> action(
			const std::vector<Vector> &elems, const Vector &g, const Vector &h) const = 0;

	/**
	 * Create the parallel transport (rotation matrix) for a point in g to h.
	 *
	 * @param g The point from which the transport starts.
	 * @param h The point where the transport should end.
	 * @return The parallel transport matrix.
	 */
	virtual TangentMatrix parallelTransportTrafo(const Vector &g, const Vector &h) const = 0;

	/**
	 * Perform the parallel transport for a vector in g to h.
	 *
	 * @param vec The vector to transport.
	 * @param g The point from which the transport starts.
	 * @param h The point where the transport should end.
	 * @return The transported vector.
	 */
	virtual TangentVector parallelTransportVector(
			const TangentVector &vec, const Vector &g, const Vector &h) const {
		return parallelTransportTrafo(g, h) * vec;
	}

	/**
	 * Perform the parallel transport for a matrix in g to h.
	 *
	 * @param mat The matrix to transport.
	 * @param g The point from which the transport starts.
	 * @param h The point where the transport should end.
	 * @return The transported matrix.
	 */
	virtual TangentMatrix parallelTransportMatrix(
			const TangentMatrix &mat, const Vector &g, const Vector &h) const {
		TangentMatrix trafo = parallelTransportTrafo(g, h);
		return trafo * mat * trafo.transpose();
	}

	/**
	 * The destructor. Needs to be virtual because we have virtual methods.
	 */
	virtual ~Manifold(){};

protected:
	Manifold(){};
};
}  // namespace pglfd
