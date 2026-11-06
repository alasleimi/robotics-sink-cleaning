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
#include "pglfd/model/lqt.h"
#include <cassert>
#include "pglfd/model/gaussian.h"

using namespace pglfd;
int main() {
	EuclideanManifold3d::Vector identity;
	identity << 0, 0, 0;

	std::vector<double> t_vias;
	std::vector<Gaussian<EuclideanManifold3d>> vias;
	for(int i = 0; i < 5; i++) {
		t_vias.push_back(i / 5.);
		vias.emplace_back(
				(EuclideanManifold3d::Vector() << i, i, i).finished(),
				EuclideanManifold3d::TangentMatrix::Identity() * 1e-3);
	}

	LQT<EuclideanManifold3d> lqt = LQT<EuclideanManifold3d>(identity, t_vias, vias, 1.0, 0.1, 0.001);
	lqt.fit();

	assert((lqt.getStates()[8].mu_
					- (EuclideanManifold3d::Vector() << 4.49806, 4.49806, 4.49806).finished())
						 .isMuchSmallerThan(1., 1e-5));

	return 0;
}