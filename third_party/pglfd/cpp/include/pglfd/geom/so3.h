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
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>

namespace pglfd {
template<typename Scalar>
/**
 * SO(3) Manifold implementation.
 *
 * @param Scalar The data type to use.
 */
class SO3 {
public:
	using Vector = Eigen::Matrix<Scalar, 4, 1>;         //!< On-manifold vector type.
	using Tangent = Eigen::Matrix<Scalar, 3, 1>;        //!< Tangent vector type.
	using TangentMatrix = Eigen::Matrix<Scalar, 3, 3>;  //!< Tangent matrix type.

	/**
	 * SO(3) element constructor.
	 *
	 * @param vec The xyzw quaternion.
	 */
	explicit SO3(const Vector &vec) : vec_(vec) {
		assert(std::abs(vec_.norm() - 1) < 0.1);
	}

	/**
	 * Normalize the quaternion.
	 *
	 * @return Reference to self.
	 */
	SO3<Scalar> &normalize() {
		assert(std::abs(vec_.norm() - 1) < 0.1);
		vec_ /= vec_.norm();
		return *this;
	}

	/**
	 * Get a xyzw quaternion vector.
	 *
	 * @return A xyzw vector.
	 */
	constexpr const Vector &toVec() const {
		return vec_;
	}

	/**
	 * Get a map to an Eigen Quaternion (zero copy).
	 *
	 * @return The Eigen Quaternion.
	 */
	constexpr const Eigen::Map<const Eigen::Quaternion<Scalar>> toQuat() const {
		return Eigen::Map<const Eigen::Quaternion<Scalar>>(vec_.data());
	}

	/**
	 * Get the inverse element.
	 *
	 * @return The inverse SO(3) element.
	 */
	constexpr SO3<Scalar> inverse() const {
		Vector new_vec = vec_;
		new_vec.head(3) = -vec_.head(3);
		return SO3<Scalar>(new_vec);
	}

	/**
	 * Group operation.
	 *
	 * @param rhs The other element to multiply with.
	 * @return The group operation of this and rhs.
	 */
	constexpr SO3<Scalar> operator*(const SO3<Scalar> &rhs) const {
		// Use quaternion multiplication from Eigen
		return SO3<Scalar>((toQuat() * rhs.toQuat()).coeffs());
	}

	/**
	 * Exponential operation using code from Sophus: https://github.com/strasdat/sophus.
	 *
	 * @param tangent The tangent vector to exponentiate.
	 * @param eps "Small value" threshold to perform linear operation.
	 * @return The manifold element.
	 */
	static constexpr SO3<Scalar> exp(const Tangent &tangent, Scalar eps = 1e-10) {
		Scalar theta_sq = tangent.squaredNorm();

		Scalar imag_factor;
		Scalar real_factor;
		if(theta_sq < eps * eps) {
			Scalar theta_po4 = theta_sq * theta_sq;
			imag_factor = Scalar(0.5) - Scalar(1.0 / 48.0) * theta_sq + Scalar(1.0 / 3840.0) * theta_po4;
			real_factor = Scalar(1) - Scalar(1.0 / 8.0) * theta_sq + Scalar(1.0 / 384.0) * theta_po4;
		} else {
			Scalar theta = std::sqrt(theta_sq);
			Scalar half_theta = Scalar(0.5) * theta;
			Scalar sin_half_theta = std::sin(half_theta);
			imag_factor = sin_half_theta / theta;
			real_factor = std::cos(half_theta);
		}

		Vector res;
		res << imag_factor * tangent(0), imag_factor * tangent(1), imag_factor * tangent(2),
				real_factor;
		return SO3<Scalar>(res);
	}

	/**
	 * Logarithm operation using code from Sophus: https://github.com/strasdat/sophus.
	 *
	 * @param eps "Small value" threshold to perform linear operation.
	 * @return The tangent vector at the origin.
	 */
	constexpr Tangent log(Scalar eps = 1e-10) const {
		Scalar squared_n = vec_.head(3).squaredNorm();
		Scalar w = vec_(3);

		Scalar two_atan_nbyw_by_n;

		/// Atan-based log thanks to
		///
		/// C. Hertzberg et al.:
		/// "Integrating Generic Sensor Fusion Algorithms with Sound State
		/// Representation through Encapsulation of Manifolds"
		/// Information Fusion, 2011

		if(squared_n < eps * eps) {
			// If quaternion is normalized and n=0, then w should be 1;
			// w=0 should never happen here!
			assert(std::abs(w) >= eps);
			Scalar squared_w = w * w;
			two_atan_nbyw_by_n = Scalar(2) / w - Scalar(2.0 / 3.0) * (squared_n) / (w * squared_w);
		} else {
			Scalar n = std::sqrt(squared_n);

			// w < 0 ==> cos(theta/2) < 0 ==> theta > pi
			//
			// By convention, the condition |theta| < pi is imposed by wrapping theta
			// to pi; The wrap operation can be folded inside evaluation of atan2
			//
			// theta - pi = atan(sin(theta - pi), cos(theta - pi))
			//            = atan(-sin(theta), -cos(theta))
			//
			Scalar atan_nbyw = (w < Scalar(0)) ? Scalar(std::atan2(-n, -w)) : Scalar(std::atan2(n, w));
			two_atan_nbyw_by_n = Scalar(2) * atan_nbyw / n;
		}

		return two_atan_nbyw_by_n * vec_.head(3);
	}

private:
	Vector vec_;
};
using SO3d = SO3<double>;
}  // namespace pglfd
