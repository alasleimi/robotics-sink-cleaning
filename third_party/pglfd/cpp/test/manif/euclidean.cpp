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
#include "pglfd/manif/euclidean.h"
#include <cassert>

using namespace pglfd;
int main() {
	EuclideanManifold1d manif_1d;
	EuclideanManifold1d::Vector vec_1d;
	EuclideanManifold1d::TangentMatrix mat_1d;
	assert(EuclideanManifold1d::dim_manifold == 1);
	assert(EuclideanManifold1d::dim_tangent == 1);
	vec_1d << 0;
	assert((manif_1d.id_elem() - vec_1d).isMuchSmallerThan(1e-5));
	assert((manif_1d.action({vec_1d}, vec_1d, vec_1d)[0] - vec_1d).isMuchSmallerThan(1e-5));
	mat_1d << 1;
	assert((manif_1d.parallelTransportTrafo(vec_1d, vec_1d) - mat_1d).isMuchSmallerThan(1e-5));
	assert(
			(manif_1d.parallelTransportVector(vec_1d, vec_1d, vec_1d) - vec_1d).isMuchSmallerThan(1e-5));
	assert(
			(manif_1d.parallelTransportMatrix(mat_1d, vec_1d, vec_1d) - mat_1d).isMuchSmallerThan(1e-5));

	EuclideanManifold3d manif_3d;
	EuclideanManifold3d::Vector vec_3d;
	EuclideanManifold3d::TangentMatrix mat_3d;
	assert(EuclideanManifold3d::dim_manifold == 3);
	assert(EuclideanManifold3d::dim_tangent == 3);
	vec_3d << 0, 0, 0;
	assert((manif_3d.id_elem() - vec_3d).isMuchSmallerThan(1e-5));
	assert((manif_3d.action({vec_3d}, vec_3d, vec_3d)[0] - vec_3d).isMuchSmallerThan(1e-5));
	mat_3d.setIdentity();
	assert((manif_3d.parallelTransportTrafo(vec_3d, vec_3d) - mat_3d).isMuchSmallerThan(1e-5));
	assert(
			(manif_3d.parallelTransportVector(vec_3d, vec_3d, vec_3d) - vec_3d).isMuchSmallerThan(1e-5));
	assert(
			(manif_3d.parallelTransportMatrix(mat_3d, vec_3d, vec_3d) - mat_3d).isMuchSmallerThan(1e-5));

	return 0;
}