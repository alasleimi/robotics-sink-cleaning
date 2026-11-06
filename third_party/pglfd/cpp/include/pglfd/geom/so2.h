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
#include <cassert>
#include <cmath>

namespace pglfd {
/**
 * SO(2) Manifold implementation.
 *
 * @param Scalar The data type to use.
 */
template<typename Scalar>
class SO2 {
public:
	using Vector = Eigen::Matrix<Scalar, 2, 1>;         //!< On-manifold vector type.
	using Tangent = Eigen::Matrix<Scalar, 1, 1>;        //!< Tangent vector type.
	using TangentMatrix = Eigen::Matrix<Scalar, 1, 1>;  //!< Tangent matrix type.

	/**
	 * SO(2) element constructor.
	 *
	 * @param vec The SO(2) element.
	 */
	explicit SO2(const Vector& vec) : vec_(vec) {
		assert(std::abs(vec_.norm() - 1) < 0.1);
	}

	/**
	 * Normalize the SO(2) element.
	 */
	SO2<Scalar>& normalize() {
		assert(std::abs(vec_.norm() - 1) < 0.1);
		vec_ /= vec_.norm();
		return *this;
	}

	/**
	 * Get the SO(2) vector.
	 *
	 * @return The SO(2) vector.
	 */
	constexpr const Vector& toVec() const {
		return vec_;
	}

	/**
	 * Get the inverse element.
	 *
	 * @return The inverse SO(2) element.
	 */
	constexpr SO2<Scalar> inverse() const {
		Vector new_vec = vec_;
		new_vec[1] *= -1;
		return SO2<Scalar>(new_vec);
	}

	/**
	 * Group operation.
	 *
	 * @param rhs The other element to multiply with.
	 * @return The group operation of this and rhs.
	 */
	constexpr SO2<Scalar> operator*(const SO2<Scalar>& rhs) const {
		Vector new_vec;
		new_vec << 0, 0;
		new_vec[0] = vec_[0] * rhs.toVec()[0] - vec_[1] * rhs.toVec()[1];
		new_vec[1] = vec_[0] * rhs.toVec()[1] + vec_[1] * rhs.toVec()[0];
		return SO2<Scalar>(new_vec);
	}

	/**
	 * Exponential operation.
	 *
	 * @param tangent The tangent vector to exponentiate.
	 * @return The manifold element.
	 */
	static constexpr SO2<Scalar> exp(const Tangent& tangent) {
		Vector new_vec;
		new_vec << std::cos(tangent[0]), std::sin(tangent[0]);
		return SO2<Scalar>(new_vec);
	}

	/**
	 * Logarithm operation.
	 *
	 * @return The tangent vector at the origin.
	 */
	constexpr Tangent log() const {
		Tangent tangent;
		tangent << std::atan2(vec_[1], vec_[0]);
		return tangent;
	}

private:
	Vector vec_;
};
using SO2d = SO2<double>;
}  // namespace pglfd
