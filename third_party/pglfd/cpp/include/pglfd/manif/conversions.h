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
#include <cmath>
#include <string>
#include <vector>
#include "pglfd/geom/s2.h"
#include "pglfd/geom/so3.h"
#include "pglfd/manif/euclidean.h"
#include "pglfd/manif/product.h"
#include "pglfd/manif/s2.h"
#include "pglfd/manif/so2.h"
#include "pglfd/manif/so3.h"

namespace pglfd {
/**
 * Transform R3*SO3 data to manifold data.
 *
 * @param Manifold The manifold to transform into.
 * @param vec The vector to transform.
 * @param eps Epsilon value for small radii.
 * @return The transformed data.
 */
template<typename Manifold>
inline typename Manifold::Vector manifold_from_r3_so3(
		const SE3Manifold<double>::Vector &vec, double eps = 1e-10);

// R3*SO(3) -> R3 (just keep translational part)
template<>
inline EuclideanManifold<double, 3>::Vector manifold_from_r3_so3<EuclideanManifold<double, 3>>(
		const SE3Manifold<double>::Vector &vec, double eps) {
	return vec.head(3);
}

// R3*SO(3) -> R3*SO(3) (identity)
template<>
inline SE3Manifold<double>::Vector manifold_from_r3_so3<SE3Manifold<double>>(
		const SE3Manifold<double>::Vector &vec, double eps) {
	return vec;
}

// R3*SO(3) -> SO(2)*R2 (just keep translational part)
template<>
inline ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>::Vector
		manifold_from_r3_so3<
				ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>>(
				const SE3Manifold<double>::Vector &vec, double eps) {
	ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>::Vector vec_out;
	// Compute the radius
	double r = vec.head(2).norm();

	// Calculate the SO(2) data, using the id element for r < eps
	if(r > eps) {
		vec_out.head(2) = vec.head(2) / r;
		vec_out(2) = r;
	} else {
		vec_out.head(2) = SO2Manifold<double>().id_elem();
		vec_out(2) = 0;
	}

	// Add z entry
	vec_out(3) = vec(2);
	return vec_out;
}

// R3*SO(3) -> SO(2)*R2*SO(3)
template<>
inline CylindricalManifold<double>::Vector manifold_from_r3_so3<CylindricalManifold<double>>(
		const SE3Manifold<double>::Vector &vec, double eps) {
	CylindricalManifold<double>::Vector vec_out;

	// First add translational part
	vec_out.head(4) = manifold_from_r3_so3<
			ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>>(vec);

	// Rotate the SO(3) part
	SO3Manifold<double>::TangentVector so3_addtangent = SO3Manifold<double>::TangentVector::Zero();
	so3_addtangent.tail(1) =
			-SO2Manifold<double>().log(vec_out.head(2), SO2Manifold<double>().id_elem());
	vec_out.tail(4) = (SO3d::exp(so3_addtangent) * SO3d(vec.tail(4))).toVec();

	return vec_out;
}

// R3*SO(3) -> S2*R1 (just keep translational part)
template<>
inline ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>::Vector
		manifold_from_r3_so3<ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>>(
				const SE3Manifold<double>::Vector &vec, double eps) {
	ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>::Vector vec_out;
	// Compute the radius
	double r = vec.head(3).norm();

	// Calculate the S2 data, using the id element for r < eps
	if(r > eps) {
		vec_out.head(3) = vec.head(3) / r;
		vec_out(3) = r;
	} else {
		vec_out.head(3) = S2Manifold<double>().id_elem();
		vec_out(3) = 0;
	}
	return vec_out;
}

// R3*SO(3) -> S2*R1*SO(3)
template<>
inline SphericalManifold<double>::Vector manifold_from_r3_so3<SphericalManifold<double>>(
		const SE3Manifold<double>::Vector &vec, double eps) {
	SphericalManifold<double>::Vector vec_out;

	// First add translational part
	vec_out.head(4) = manifold_from_r3_so3<
			ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>>(vec);

	// Rotate the SO(3) part
	Eigen::Quaterniond orientation =
			Eigen::Quaterniond((S2<double>(vec_out.head<3>())).axis_angle()).inverse();
	Eigen::Vector4d orientation_vec;
	orientation_vec << orientation.x(), orientation.y(), orientation.z(), orientation.w();
	vec_out.tail<4>() = (SO3<double>(orientation_vec) * SO3<double>(vec.tail<4>())).toVec();

	return vec_out;
}

/**
 * Transform manifold data to R3*SO3 data.
 *
 * @param Manifold The manifold to transform from.
 * @param vec The vector to transform.
 * @return The transformed vector.
 */
template<typename Manifold>
inline SE3Manifold<double>::Vector manifold_to_r3_so3(const typename Manifold::Vector &vec);

// R3 -> R3*SO(3) (just keep translational part)
template<>
inline SE3Manifold<double>::Vector manifold_to_r3_so3<EuclideanManifold<double, 3>>(
		const EuclideanManifold<double, 3>::Vector &vec) {
	SE3Manifold<double>::Vector vec_out;
	vec_out.setZero();
	vec_out.head(3) = vec;
	vec_out[6] = 1;
	return vec_out;
}

// R3*SO(3) -> R3*SO(3)
template<>
inline SE3Manifold<double>::Vector manifold_to_r3_so3<SE3Manifold<double>>(
		const SE3Manifold<double>::Vector &vec) {
	return vec;
}

// SO(2)*R2  -> R3*SO(3) (just keep translational part)
template<>
inline SE3Manifold<double>::Vector manifold_to_r3_so3<
		ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>>(
		const ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>::Vector &vec) {
	SE3Manifold<double>::Vector vec_out;
	vec_out.setZero();
	vec_out[0] = vec[0] * vec[2];
	vec_out[1] = vec[1] * vec[2];
	vec_out[2] = vec[3];
	vec_out[6] = 1;
	return vec_out;
}

// SO(2)*R2*SO83)  -> R3*SO(3)
template<>
inline SE3Manifold<double>::Vector manifold_to_r3_so3<CylindricalManifold<double>>(
		const CylindricalManifold<double>::Vector &vec) {
	SE3Manifold<double>::Vector vec_out = manifold_to_r3_so3<
			ProductManifold<double, SO2Manifold<double>, EuclideanManifold<double, 2>>>(vec.head(4));

	// Rotate the orientation part
	SO3Manifold<double> manif_so3;
	SO3Manifold<double>::TangentVector so3_addtangent;
	so3_addtangent << 0, 0, SO2Manifold<double>().log(vec.head<2>(), SO2Manifold<double>().id_elem());
	vec_out.tail<4>() =
			(SO3<double>(manif_so3.exp(so3_addtangent, manif_so3.id_elem())) * SO3<double>(vec.tail<4>()))
					.toVec();
	return vec_out;
}

// S2*R1  -> R3*SO(3) (just keep translational part)
template<>
inline SE3Manifold<double>::Vector manifold_to_r3_so3<
		ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>>(
		const ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>::Vector &vec) {
	SE3Manifold<double>::Vector vec_out;
	vec_out.setZero();
	vec_out[0] = vec[0] * vec[3];
	vec_out[1] = vec[1] * vec[3];
	vec_out[2] = vec[2] * vec[3];
	vec_out[6] = 1;
	return vec_out;
}

// S2*R1*SO(3)  -> R3*SO(3)
template<>
inline SE3Manifold<double>::Vector manifold_to_r3_so3<SphericalManifold<double>>(
		const SphericalManifold<double>::Vector &vec) {
	SE3Manifold<double>::Vector vec_out =
			manifold_to_r3_so3<ProductManifold<double, S2Manifold<double>, EuclideanManifold<double, 1>>>(
					vec.head(4));

	// Rotate the orientation part
	Eigen::Quaterniond orientation((S2<double>(vec.head<3>())).axis_angle());
	Eigen::Vector4d orientation_vec;
	orientation_vec << orientation.x(), orientation.y(), orientation.z(), orientation.w();
	vec_out.tail<4>() = (SO3<double>(orientation_vec) * SO3<double>(vec.tail<4>())).toVec();
	return vec_out;
}
}  // namespace pglfd