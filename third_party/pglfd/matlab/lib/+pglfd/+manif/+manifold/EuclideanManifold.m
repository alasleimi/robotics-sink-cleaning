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

classdef EuclideanManifold < pglfd.manif.manifold.Manifold
    %EUCLIDEANMANIFOLD Euclidean Manifold with selectable dimension.
    %   This manifold implements the euclidean manifold with variable dimension.
    properties
        name % Name of the manifold.
        id_element % Identity element.
    end
    properties (Nontunable)
        dim_manifold % Dimension of a manifold vector.
        dim_tangent % Dimension of a tangent space vector.
    end
    methods
        function obj = EuclideanManifold(dimension)
            %   :param dimension: Dimension of the manifold.

            % construct name -> num2str won't compile
            obj.name = ['R' pglfd.util.num2str(dimension)];

            % save dimensions
            obj.dim_manifold = dimension;
            obj.dim_tangent = dimension;

            % set the id eleemnt
            obj.id_element = zeros(dimension, 1);
        end
    end
    methods (Static)
        function expval = exp(tangent, base)
            %EXP Riemannian exponential of input.
            %
            %   :param tangent: The tangent vector to exponentiate.
            %   :param base: The point in the manifold at which to exponentiate.
            %   :return: The exponentiated value.
            expval = tangent + base;
        end

        function logval = log(point, base)
            %LOG Riemannian logarithm of input.
            %
            %   :param point: The point on the manifold to take the logarithm of.
            %   :param base: The point on the manifold at which to take the logarithm.
            %   :return: The tangent-space vector.
            logval = point - base;
        end

        function actionval = action(point, g, h)
            %ACTION Action function, move point relative to g to h.
            %
            %   :param point: The manifold element to transform.
            %   :param g: Point in the manifold from which to transform.
            %   :param h: Point in the manifold to which to transform.
            %   :return: Transformed manifold elements.
            actionval = point - g + h;
        end

        function parallel_transport_trafo_val = parallel_transport_trafo(g, ~)
            %PARALLEL_TRANSPORT_TRAFO The parallel transport matrix.
            %   Parallel transport transformation matrix for tangent elements from manifold point g to h.
            %
            %   :param g: The point in the manifold from which to transform.
            %   :return: Transformation matrix.
            parallel_transport_trafo_val = eye(size(g, 1));
        end

        function parallel_transport_val = parallel_transport(input, ~, ~)
            %PARALLEL_TRANSPORT Calculate the parallel transport.
            %
            %   :param input: The vector or matrix to transport.
            %   :return: The parallel-transported value.
            parallel_transport_val = input;
        end
    end
end
