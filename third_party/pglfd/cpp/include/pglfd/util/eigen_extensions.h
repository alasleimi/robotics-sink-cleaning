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
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <numeric>
#include <vector>

namespace Eigen {
/**
 * Permute rows in a matrix in place.
 *
 * @param MatrixType_ The matrix type to be used.
 * @param mat The matrix with rows to be permuted in place.
 * @param reorderings Row indices for reorderings.
 */
template<typename MatrixType_>
void permuteRowsInPlace(MatrixType_ &mat, const std::vector<int> &reorderings) {
	// Need a temporary for explicit evaluation, otherwise Eigen overrides elements
	MatrixType_ tmp = mat(reorderings, Eigen::all);
	mat = tmp;
}

/**
 * Perform a matrix permutation in place without multiplication.
 *
 * @param mat The matrix to permute in place.
 * @param reorderings New list of indices.
 */
template<typename MatrixType_>
void permuteMatrixInPlace(MatrixType_ &mat, const std::vector<int> &reorderings) {
	// Need a temporary for explicit evaluation, otherwise Eigen overrides elements
	MatrixType_ tmp = mat(reorderings, Eigen::all);
	mat = tmp(Eigen::all, reorderings);
}

/**
 * Cholesky-decomposition allowing for updates (adding / deleting rows).
 *
 * @param MatrixType_ The matrix type used by the Cholesky decomposition.
 */
template<typename MatrixType_>
class UpdateLLT : public LLT<MatrixType_, Lower> {
public:
	typedef typename MatrixType_::Scalar Scalar;  //!< Scalar type used by the Cholesky decmposition.

	/**
	 * Empty constructor.
	 */
	UpdateLLT() : LLT<MatrixType_, Lower>() {}

	/**
	 * Copy constructor.
	 *
	 * @param llt The Cholesky decomposition to copy.
	 */
	UpdateLLT(const LLT<MatrixType_, Lower> &llt) :  // cppcheck-suppress noExplicitConstructor
			LLT<MatrixType_, Lower>(llt) {}

	/**
	 * Extend the decomposed matrix. This assumes that rows and columns are being added at the end.
	 *
	 * @param input The matrix with size increase.
	 */
	void extendMatrix(const MatrixType_ &input) {
		// Selectors
		auto old_sel = Eigen::seq(0, this->m_matrix.rows() - 1);
		auto new_sel = Eigen::seq(this->m_matrix.rows(), Eigen::last);

		// Calculate lower left & appended diagonal factors
		MatrixType_ new_lower_left_t = this->matrixL().solve(input(old_sel, Eigen::all));
		MatrixType_ new_lower_left = new_lower_left_t.transpose();  // need that one twice
		MatrixType_ new_lower_diag = (input(new_sel, Eigen::all) - new_lower_left * new_lower_left_t)
																		 .llt()
																		 .matrixL()
																		 .toDenseMatrix();

		// Update our internal matrix
		this->m_matrix.conservativeResize(input.rows(), input.rows());
		this->m_matrix(new_sel, old_sel) = new_lower_left;
		this->m_matrix(new_sel, new_sel) = new_lower_diag;
	}

	/**
	 * Delete rows and columns from this decomposition.
	 *
	 * @param reorderings The reordering to perform.
	 * @param new_size New size of the problem.
	 */
	void reorderDelete(const std::vector<int> &reorderings, int new_size) {
		// If nothing needs to be reordered, just resize!
		if(reorderings.size() > 0) {
			// First set the upper diagonal entries to zero - this is required as the original
			// decompositions leaves them uninitialized
			this->m_matrix.block(0, 1, this->m_matrix.rows() - 1, this->m_matrix.cols() - 1)
					.template triangularView<Upper>()
					.setZero();

			// Calculate stick out, but only for the elements we want to keep
			std::vector<int> stick_out;
			stick_out.resize(new_size);
			int maxval = 0;
			int stick_out_counter = 0;
			for(int i = 0; i < new_size; i++) {
				// Calculate how far it sticks out
				maxval = std::max(maxval, reorderings[i]);
				stick_out[i] = maxval - i;
				if(stick_out[i] > 0)
					stick_out_counter++;
			}

			// Reorder the matrix rows
			permuteRowsInPlace(this->m_matrix, reorderings);

			// Reserve indices
			std::vector<int> indices;
			indices.resize(new_size);

			// Fix elements while something sticks out
			while(stick_out_counter > 0) {
				stick_out_counter = 0;

				// Sort stick out values
				std::iota(indices.begin(), indices.end(), 0);
				std::stable_sort(indices.begin(), indices.end(), [&stick_out](size_t i1, size_t i2) {
					return stick_out[i1] > stick_out[i2];
				});

				// Fix elements that "stick out" using Givens rotations
				int max_stick_out = 0;
				for(int i: indices) {
					// Check if something sticks out
					if(stick_out[i] >= max_stick_out) {
						max_stick_out = stick_out[i];

						// Calculate indices
						int first = i + stick_out[i] - 1;
						int second = i + stick_out[i];

						// Calculate Givens rotation
						Scalar g_c, g_s;
						Scalar first_elem = this->m_matrix(i, first);
						Scalar second_elem = this->m_matrix(i, second);
						if(first_elem > second_elem) {
							Scalar rel = second_elem / first_elem;
							Scalar g_c_inv = std::sqrt(rel * rel + 1);
							if(first_elem < 0)
								g_c_inv = -g_c_inv;
							g_c = Scalar(1) / g_c_inv;
							g_s = (g_c == Scalar(1)) ? Scalar(0) : rel * g_c;
						} else {
							Scalar rel = first_elem / second_elem;
							Scalar g_s_inv = std::sqrt(rel * rel + 1);
							if(second_elem < 0)
								g_s_inv = -g_s_inv;
							g_s = Scalar(1) / g_s_inv;
							g_c = (g_s == Scalar(1)) ? Scalar(0) : rel * g_s;
						}

						// Perform Givens rotation on this and all following rows
						for(int ii = i; ii < new_size; ii++) {
							// cppcheck-suppress shadowVariable
							Scalar first_elem = this->m_matrix(ii, first);
							// cppcheck-suppress shadowVariable
							Scalar second_elem = this->m_matrix(ii, second);
							this->m_matrix(ii, first) = first_elem * g_c + second_elem * g_s;
							this->m_matrix(ii, second) = -first_elem * g_s + second_elem * g_c;
						}

						// Decrease stick out counter
						stick_out[i]--;
						if(stick_out[i] > 0) {
							stick_out_counter++;
						}
					} else {
						// We can jump out of the indices for loop early as there's nothing left to do now
						break;
					}
				}
			}
		}

		// Finally resize
		this->m_matrix.conservativeResize(new_size, new_size);
	}
};
};  // namespace Eigen
