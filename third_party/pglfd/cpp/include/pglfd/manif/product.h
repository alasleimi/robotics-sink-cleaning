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
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "pglfd/manif/euclidean.h"
#include "pglfd/manif/manifold.h"
#include "pglfd/manif/s2.h"
#include "pglfd/manif/so2.h"
#include "pglfd/manif/so3.h"
#include "pglfd/util/execution.h"

namespace pglfd {
// Indexing of submanifolds
template<std::ptrdiff_t... Ints>
using manifold_selector = std::integer_sequence<std::ptrdiff_t, Ints...>;

template<typename Manifold, typename... Manifolds>
constexpr int dim_manifold() {
	if constexpr(sizeof...(Manifolds) == 0)
		return Manifold::dim_manifold;
	else
		return Manifold::dim_manifold + dim_manifold<Manifolds...>();
}

template<typename Manifold, typename... Manifolds>
constexpr int dim_tangent() {
	if constexpr(sizeof...(Manifolds) == 0)
		return Manifold::dim_tangent;
	else
		return Manifold::dim_tangent + dim_tangent<Manifolds...>();
}

template<typename Manifold, typename... Manifolds>
constexpr bool discont_log_alt() {
	if constexpr(sizeof...(Manifolds) == 0)
		return Manifold::discont_log_alt;
	else
		return Manifold::discont_log_alt || discont_log_alt<Manifolds...>();
}

template<typename T, T t, T... remainder>
constexpr void array_append(std::vector<T> &array) {
	array.emplace_back(t);
	if constexpr(sizeof...(remainder) > 0)
		array_append<T, remainder...>(array);
}

template<typename T, T... idx_man_sel>
constexpr std::vector<T> to_array() {
	std::vector<T> array;
	array_append<T, idx_man_sel...>(array);
	return array;
}

/**
 * Product manifold implementation.
 *
 * @param Scalar_ The data type to use for the contained manifolds.
 * @param Manifolds The manifolds that should be stacked.
 */
template<typename Scalar_, typename... Manifolds>
class ProductManifold :
		public Manifold<
				Scalar_,
				dim_manifold<Manifolds...>(),
				dim_tangent<Manifolds...>(),
				discont_log_alt<Manifolds...>()> {
public:
	using Scalar = Scalar_;  //!< Scalar type used on the manifold.
	using Vector =
			typename Manifold<Scalar, dim_manifold<Manifolds...>(), dim_tangent<Manifolds...>()>::
					Vector;  //!< On-manifold vector type.
	using TangentVector =
			typename Manifold<Scalar, dim_manifold<Manifolds...>(), dim_tangent<Manifolds...>()>::
					TangentVector;  //!< Tangent vector type.
	using TangentMatrix =
			typename Manifold<Scalar, dim_manifold<Manifolds...>(), dim_tangent<Manifolds...>()>::
					TangentMatrix;                                                  //!< Tangent matrix type.
	using MatrixX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  //!< Dynamic matrix type.

	std::string name() const override {
		return append_names<Manifolds...>();
	}

	Vector id_elem() const override {
		Vector id;
		int offset = 0;
		append_id_elems<Manifolds...>(id, offset);
		return id;
	}

	/**
	 * @copydoc Manifold::normalize()
	 */
	Vector normalize(const Vector &vector) const override {
		Vector norm_vec;
		int offset_manifold = 0;
		append_normalize<Manifolds...>(norm_vec, offset_manifold, vector);
		return norm_vec;
	}

	/**
	 * @copydoc Manifold::exp()
	 */
	Vector exp(const TangentVector &tangent, const Vector &base) const override {
		Vector exp_vec;
		int offset_manifold = 0;
		int offset_tangent = 0;
		append_exp<Manifolds...>(exp_vec, offset_manifold, offset_tangent, tangent, base);
		return exp_vec;
	}

	/**
	 * @copydoc Manifold::log()
	 */
	TangentVector log(const Vector &point, const Vector &base) const override {
		TangentVector log_vec;
		int offset_manifold = 0;
		int offset_tangent = 0;
		append_log<Manifolds...>(log_vec, offset_manifold, offset_tangent, point, base);
		return log_vec;
	}

	/**
	 * @copydoc Manifold::continuousLog()
	 */
	std::tuple<MatrixX, std::vector<int>> continuousLog(
			const MatrixX &tangents,
			const std::map<int, int> &vias = {},
			bool start_low = true) const override {
		MatrixX tangents_continuous(tangents.rows(), tangents.cols());
		std::vector<int> discontinuities;
		int offset_tangent = 0;
		append_continuousLog<Manifolds...>(
				tangents_continuous, discontinuities, offset_tangent, tangents, vias, start_low);
		return {tangents_continuous, discontinuities};
	}

	/**
	 * @copydoc Manifold::action()
	 */
	std::vector<Vector> action(
			const std::vector<Vector> &elems, const Vector &g, const Vector &h) const override {
		std::vector<Vector> res;
		for(const auto &elem: elems) {
			Vector action_vec;
			int offset = 0;
			append_action<Manifolds...>(action_vec, offset, elem, g, h);
			res.push_back(action_vec);
		}
		return res;
	}

	/**
	 * @copydoc Manifold::parallelTransportTrafo()
	 */
	TangentMatrix parallelTransportTrafo(const Vector &g, const Vector &h) const override {
		TangentMatrix trafo;
		trafo.setZero();
		int offset_manifold = 0;
		int offset_tangent = 0;
		append_trafo<Manifolds...>(trafo, offset_manifold, offset_tangent, g, h);
		return trafo;
	}

	/**
	 * Get the submanifold of idxs.
	 *
	 * @tparam idx_man_sel Submanifold elements.
	 * @return The submanifold, on-manifold idx and tangent idx.
	 */
	template<std::ptrdiff_t... idx_man_sel>
	std::tuple<
			ProductManifold<Scalar_, std::tuple_element_t<idx_man_sel, std::tuple<Manifolds...>>...>,
			std::vector<int>,
			std::vector<int>>
			submanifold() const {
		// First get the new manifold
		ProductManifold<Scalar_, std::tuple_element_t<idx_man_sel, std::tuple<Manifolds...>>...> man;

		// Then extract corresponding indices for use with Eigen
		std::vector idx_sel_vec = to_array<std::ptrdiff_t, idx_man_sel...>();
		std::vector<int> idx_man;
		std::vector<int> last_offsets;
		std::vector<int> last_dims;
		get_idx_man<Manifolds...>(idx_sel_vec, idx_man, last_offsets, last_dims, 0);
		std::vector<int> idx_tan;
		last_offsets.clear();
		last_dims.clear();
		get_idx_tan<Manifolds...>(idx_sel_vec, idx_tan, last_offsets, last_dims, 0);
		return {man, idx_man, idx_tan};
	}

private:
	template<typename M, typename... Ms>
	void get_idx_man(
			std::vector<std::ptrdiff_t> idx_sel,
			std::vector<int> &indices,
			std::vector<int> last_offsets,
			std::vector<int> last_dims,
			int idx_offset) const {
		last_offsets.push_back(idx_offset);
		last_dims.push_back(M::dim_manifold);
		if(idx_sel.size() > 0 && idx_sel[0] == 0) {
			while(idx_sel.size() > 0 && idx_sel[0] <= 0) {
				for(int i = 0; i < last_dims[last_dims.size() - 1 + idx_sel[0]]; i++) {
					indices.push_back(last_offsets[last_offsets.size() - 1 + idx_sel[0]]++);
				}
				idx_sel.erase(idx_sel.begin());
			}
			idx_offset = last_offsets[last_offsets.size() - 1];
		} else {
			for(int i = 0; i < M::dim_manifold; i++) {
				idx_offset++;
			}
		}
		std::for_each(execution::unseq, idx_sel.begin(), idx_sel.end(), [](auto &val) { val -= 1; });
		if constexpr(sizeof...(Ms) > 0) {
			get_idx_man<Ms...>(idx_sel, indices, last_offsets, last_dims, idx_offset);
		}
	}

	template<typename M, typename... Ms>
	void get_idx_tan(
			std::vector<std::ptrdiff_t> &idx_sel,
			std::vector<int> &indices,
			std::vector<int> last_offsets,
			std::vector<int> last_dims,
			int idx_offset) const {
		last_offsets.push_back(idx_offset);
		last_dims.push_back(M::dim_tangent);
		if(idx_sel.size() > 0 && idx_sel[0] == 0) {
			while(idx_sel.size() > 0 && idx_sel[0] <= 0) {
				for(int i = 0; i < last_dims[last_offsets.size() - 1 + idx_sel[0]]; i++) {
					indices.push_back(last_offsets[last_offsets.size() - 1 + idx_sel[0]]++);
				}
				idx_sel.erase(idx_sel.begin());
			}
			idx_offset = last_offsets[last_offsets.size() - 1];
		} else {
			for(int i = 0; i < M::dim_manifold; i++) {
				idx_offset++;
			}
		}
		std::for_each(execution::unseq, idx_sel.begin(), idx_sel.end(), [](auto &val) { val -= 1; });
		if constexpr(sizeof...(Ms) > 0) {
			get_idx_tan<Ms...>(idx_sel, indices, last_offsets, last_dims, idx_offset);
		}
	}

	template<typename M, typename... Ms>
	std::string append_names() const {
		std::string full_name;
		M manif;
		full_name = manif.name();
		if constexpr(sizeof...(Ms) > 0)
			full_name += "*" + append_names<Ms...>();
		return full_name;
	}

	template<typename M, typename... Ms>
	void append_id_elems(Vector &id, int offset) const {
		M manif;
		id.template block<M::dim_manifold, 1>(offset, 0) = manif.id_elem();
		offset += M::dim_manifold;
		if constexpr(sizeof...(Ms) > 0)
			append_id_elems<Ms...>(id, offset);
	}

	template<typename M, typename... Ms>
	void append_normalize(Vector &norm_vec, int offset_manifold, const Vector &vector) const {
		M manif;
		norm_vec.template block<M::dim_manifold, 1>(offset_manifold, 0) =
				manif.normalize(vector.template block<M::dim_manifold, 1>(offset_manifold, 0));
		offset_manifold += M::dim_manifold;
		if constexpr(sizeof...(Ms) > 0)
			append_normalize<Ms...>(norm_vec, offset_manifold, vector);
	}

	template<typename M, typename... Ms>
	void append_exp(
			Vector &exp_vec,
			int offset_manifold,
			int offset_tangent,
			const TangentVector &tangent,
			const Vector &base) const {
		M manif;
		exp_vec.template block<M::dim_manifold, 1>(offset_manifold, 0) = manif.exp(
				tangent.template block<M::dim_tangent, 1>(offset_tangent, 0),
				base.template block<M::dim_manifold, 1>(offset_manifold, 0));
		offset_manifold += M::dim_manifold;
		offset_tangent += M::dim_tangent;
		if constexpr(sizeof...(Ms) > 0)
			append_exp<Ms...>(exp_vec, offset_manifold, offset_tangent, tangent, base);
	}

	template<typename M, typename... Ms>
	void append_log(
			TangentVector &log_vec,
			int offset_manifold,
			int offset_tangent,
			const Vector &point,
			const Vector &base) const {
		M manif;
		log_vec.template block<M::dim_tangent, 1>(offset_tangent, 0) = manif.log(
				point.template block<M::dim_manifold, 1>(offset_manifold, 0),
				base.template block<M::dim_manifold, 1>(offset_manifold, 0));
		offset_manifold += M::dim_manifold;
		offset_tangent += M::dim_tangent;
		if constexpr(sizeof...(Ms) > 0)
			append_log<Ms...>(log_vec, offset_manifold, offset_tangent, point, base);
	}

	template<typename M, typename... Ms>
	void append_continuousLog(
			MatrixX &tangents_continuous,
			std::vector<int> &discontinuities,
			int offset_tangent,
			const MatrixX &tangents,
			const std::map<int, int> &vias,
			bool start_low) const {
		M manif;
		auto [tangents_man, discontinuities_man] = manif.continuousLog(
				tangents.block(offset_tangent, 0, M::dim_tangent, tangents.cols()), vias, start_low);
		tangents_continuous.block(offset_tangent, 0, M::dim_tangent, tangents.cols()) = tangents_man;
		discontinuities.insert(
				discontinuities.end(), discontinuities_man.begin(), discontinuities_man.end());
		offset_tangent += M::dim_tangent;
		if constexpr(sizeof...(Ms) > 0)
			append_continuousLog<Ms...>(
					tangents_continuous, discontinuities, offset_tangent, tangents, vias, start_low);
	}

	template<typename M, typename... Ms>
	void append_action(
			Vector &action_vec, int offset, const Vector &elem, const Vector &g, const Vector &h) const {
		M manif;
		action_vec.template block<M::dim_manifold, 1>(offset, 0) = manif.action(
				{elem.template block<M::dim_manifold, 1>(offset, 0)},
				g.template block<M::dim_manifold, 1>(offset, 0),
				h.template block<M::dim_manifold, 1>(offset, 0))[0];
		offset += M::dim_manifold;
		if constexpr(sizeof...(Ms) > 0)
			append_action<Ms...>(action_vec, offset, elem, g, h);
	}

	template<typename M, typename... Ms>
	void append_trafo(
			TangentMatrix &trafo,
			int offset_manifold,
			int offset_tangent,
			const Vector &g,
			const Vector &h) const {
		M manif;
		trafo.template block<M::dim_tangent, M::dim_tangent>(offset_tangent, offset_tangent) =
				manif.parallelTransportTrafo(
						g.template block<M::dim_manifold, 1>(offset_manifold, 0),
						h.template block<M::dim_manifold, 1>(offset_manifold, 0));
		offset_manifold += M::dim_manifold;
		offset_tangent += M::dim_tangent;
		if constexpr(sizeof...(Ms) > 0)
			append_trafo<Ms...>(trafo, offset_manifold, offset_tangent, g, h);
	}
};

template<typename Scalar>
using SE3Manifold = ProductManifold<Scalar, EuclideanManifold<Scalar, 3>, SO3Manifold<Scalar>>;
using SE3Manifoldd = SE3Manifold<double>;

template<typename Scalar>
using CylindricalManifold =
		ProductManifold<Scalar, SO2Manifold<Scalar>, EuclideanManifold<Scalar, 2>, SO3Manifold<Scalar>>;
using CylindricalManifoldd = CylindricalManifold<double>;

template<typename Scalar>
using SphericalManifold =
		ProductManifold<Scalar, S2Manifold<Scalar>, EuclideanManifold<Scalar, 1>, SO3Manifold<Scalar>>;
using SphericalManifoldd = SphericalManifold<double>;
}  // namespace pglfd
