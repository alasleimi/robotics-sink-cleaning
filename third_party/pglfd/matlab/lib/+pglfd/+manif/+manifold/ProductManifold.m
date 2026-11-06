% Copyright (c) 2026 Maximilian Mühlbauer
%
% This file is part of pglfd.
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

classdef ProductManifold < pglfd.manif.manifold.Manifold
    %PRODUCTMANIFOLD Product of multiple manifolds.
    %   This manifold combines multiple sub-manifolds using a Riemannian
    %   product.

    properties
        name % Generated name of the manifold.
        id_element % Identity element.
        manifolds % Submanifolds of this manifold.
    end
    properties (Nontunable)
        dim_manifold % Dimension of a manifold vector.
        dim_tangent % Dimension of a tangent space vector.
    end

    methods
        function obj = ProductManifold(dim_manifold, dim_tangent, manifolds)
            % Accepts multiple manifolds as input and constructs the according product.
            % See also :mat:func:`pglfd.manif.get_productmanifold` which should be used to construct
            % a product manifold.
            %
            % :param dim_manifold: Dimension of the product manifold.
            % :param dim_tangent: Dimension of the tangent space.
            % :param manifolds: The manifolds this product is composed of.
            obj.dim_manifold = dim_manifold;
            obj.dim_tangent = dim_tangent;
            obj.manifolds = manifolds;

            % construct the id element
            obj.id_element = zeros(obj.dim_manifold, 1);
            start = 1;
            for i = 1:size(obj.manifolds, 2)
                obj.id_element(start:start+obj.manifolds{i}.dim_manifold-1) = obj.manifolds{i}.id_element;
                start = start + obj.manifolds{i}.dim_manifold;
            end

            % construct manifold name
            name_len = 0;
            for i = 1:size(obj.manifolds, 2)
                name_len = name_len + size(obj.manifolds{i}.name, 2) + 1;
            end
            name_len = name_len - 1;
            obj.name = char(zeros(1, name_len));
            name_offset = 1;
            for i = 1:size(obj.manifolds, 2)
                name_size = size(obj.manifolds{i}.name, 2);
                obj.name(:, name_offset:name_offset+name_size-1) = obj.manifolds{i}.name;
                name_offset = name_offset + name_size;
                if i < size(obj.manifolds, 2)
                    obj.name(:, name_offset:name_offset+1) = '*';
                    name_offset = name_offset + 1;
                end
            end
        end

        function expval = exp(obj, tangent, base)
            %EXP Riemannian exponential of input.
            %
            %   :param tangent: The tangent vector to exponentiate.
            %   :param base: The point in the manifold at which to exponentiate.
            %   :return: The exponentiated value.

            % check input dimensions
            obj.check_tangent_shape(tangent);
            obj.check_manifold_shape(base);

            % compute using submanifolds
            expval = zeros(obj.dim_manifold, 1);
            idx_man_start = 1;
            idx_tan_start = 1;
            for i = 1:size(obj.manifolds, 2)
                idx_man_end = idx_man_start + obj.manifolds{1, i}.dim_manifold - 1;
                idx_tan_end = idx_tan_start + obj.manifolds{1, i}.dim_tangent - 1;
                expval(idx_man_start:idx_man_end) = obj.manifolds{1, i}.exp(tangent(idx_tan_start:idx_tan_end), base(idx_man_start:idx_man_end));
                idx_man_start = idx_man_end + 1;
                idx_tan_start = idx_tan_end + 1;
            end
        end

        function logval = log(obj, point, base)
            %LOG Riemannian logarithm of input.
            %
            %   :param point: The point on the manifold to take the logarithm of.
            %   :param base: The point on the manifold at which to take the logarithm.
            %   :return: The tangent-space vector.

            % check input dimensions
            obj.check_manifold_shape(point);
            obj.check_manifold_shape(base);

            % compute using submanifolds
            logval = zeros(obj.dim_tangent, 1);
            idx_man_start = 1;
            idx_tan_start = 1;
            for i = 1:size(obj.manifolds, 2)
                idx_man_end = idx_man_start + obj.manifolds{1, i}.dim_manifold - 1;
                idx_tan_end = idx_tan_start + obj.manifolds{1, i}.dim_tangent - 1;
                logval(idx_tan_start:idx_tan_end) = obj.manifolds{1, i}.log(point(idx_man_start:idx_man_end), base(idx_man_start:idx_man_end));
                idx_man_start = idx_man_end + 1;
                idx_tan_start = idx_tan_end + 1;
            end
        end

        function actionval = action(obj, point, g, h)
            %ACTION Action function, move point relative to g to h.
            %
            %   :param point: The manifold element to transform.
            %   :param g: Point in the manifold from which to transform.
            %   :param h: Point in the manifold to which to transform.
            %   :return: Transformed manifold elements.

            % check input dimensions
            obj.check_manifold_shape(point);
            obj.check_manifold_shape(g);
            obj.check_manifold_shape(h);

            % compute action
            actionval = zeros(obj.dim_manifold, 1);
            idx_start = 1;
            for i = 1:size(obj.manifolds, 2)
                idx_end = idx_start + obj.manifolds{1, i}.dim_manifold - 1;
                actionval(idx_start:idx_end) = obj.manifolds{1, i}.action(point(idx_start:idx_end), g(idx_start:idx_end), h(idx_start:idx_end));
                idx_start = idx_end + 1;
            end
        end

        function parallel_transport_trafo_val = parallel_transport_trafo(obj, g, h)
            %PARALLEL_TRANSPORT_TRAFO The parallel transport matrix.
            %   Parallel transport transformation matrix for tangent elements from manifold point g to h.
            %
            %   :param g: The point in the manifold from which to transform.
            %   :param h: The point in the manifold to which to transform.
            %   :return: Transformation matrix.

            % check input dimensions
            obj.check_manifold_shape(g);
            obj.check_manifold_shape(h);

            % compute parallel transport
            parallel_transport_trafo_val = zeros(obj.dim_tangent, obj.dim_tangent);
            idx_man_start = 1;
            idx_tan_start = 1;
            for i = 1:size(obj.manifolds, 2)
                idx_man_end = idx_man_start + obj.manifolds{1, i}.dim_manifold - 1;
                idx_tan_end = idx_tan_start + obj.manifolds{1, i}.dim_tangent - 1;
                parallel_transport_trafo_val(idx_tan_start:idx_tan_end, idx_tan_start:idx_tan_end) = ...
                    obj.manifolds{1, i}.parallel_transport_trafo(g(idx_man_start:idx_man_end), h(idx_man_start:idx_man_end));
                idx_man_start = idx_man_end + 1;
                idx_tan_start = idx_tan_end + 1;
            end
        end
    end
end
