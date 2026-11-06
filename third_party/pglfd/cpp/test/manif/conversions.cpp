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

#ifdef NDEBUG
#undef NDEBUG
#endif
#include "pglfd/manif/conversions.h"
#include <cassert>
#include <utility>
#include "pglfd/manif/euclidean.h"
#include "pglfd/manif/product.h"
#include "pglfd/manif/s2.h"
#include "pglfd/manif/so2.h"
#include "pglfd/manif/so3.h"

using namespace pglfd;
int main() {
	SE3Manifoldd manif;
	SE3Manifoldd::Vector vec;
	vec << 0, 0, 0, 0, 0, 0, 1;

	// R3*SO(3) -> R3*SO(3)
	assert((manifold_to_r3_so3<SE3Manifoldd>(vec) - vec).isMuchSmallerThan(1e-5));
	assert((manifold_from_r3_so3<SE3Manifoldd>(vec) - vec).isMuchSmallerThan(1e-5));

	// R3*SO(3) -> R3
	assert((manifold_from_r3_so3<EuclideanManifold3d>(vec) - vec.head(3)).isMuchSmallerThan(1e-5));
	assert((manifold_to_r3_so3<EuclideanManifold3d>(vec.head(3)) - vec).isMuchSmallerThan(1e-5));

	CylindricalManifoldd::Vector vec2;
	vec2 << 0, 1, 0, 0, 0, 0, 0, 1;

	// SO(2)*R2*SO(3) -> R3*SO(3)
	assert((manifold_to_r3_so3<CylindricalManifoldd>(vec2) - vec).isMuchSmallerThan(1e-5));
	assert((manifold_from_r3_so3<CylindricalManifoldd>(vec) - vec2).isMuchSmallerThan(1e-5));

	// SO(2)*R2 -> R3*SO(3)
	using CylindPosPartd =
			std::tuple_element_t<0, decltype(CylindricalManifoldd().submanifold<0, 1>())>;
	assert((manifold_to_r3_so3<CylindPosPartd>(vec2.head(4)) - vec).isMuchSmallerThan(1e-5));
	assert((manifold_from_r3_so3<CylindPosPartd>(vec) - vec2.head(4)).isMuchSmallerThan(1e-5));

	SphericalManifoldd::Vector vec3;
	vec3 << 0, 0, 1, 0, 0, 0, 0, 1;

	// S2*R1*SO(3) -> R3*SO(3)
	assert((manifold_to_r3_so3<SphericalManifoldd>(vec3) - vec).isMuchSmallerThan(1e-5));
	assert((manifold_from_r3_so3<SphericalManifoldd>(vec) - vec3).isMuchSmallerThan(1e-5));

	// S2*R1 -> R3*SO(3)
	using SpherPosPartd = std::tuple_element_t<0, decltype(SphericalManifoldd().submanifold<0, 1>())>;
	assert((manifold_to_r3_so3<SpherPosPartd>(vec3.head(4)) - vec).isMuchSmallerThan(1e-5));
	assert((manifold_from_r3_so3<SpherPosPartd>(vec) - vec3.head(4)).isMuchSmallerThan(1e-5));

	return 0;
}
