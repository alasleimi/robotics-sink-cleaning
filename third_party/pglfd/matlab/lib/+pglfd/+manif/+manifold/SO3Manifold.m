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

classdef SO3Manifold < pglfd.manif.manifold.Manifold
    %SO3Manifold SO(3) Manifold.
    %   This manifold implements the S3 / SO(3) manifold.

    properties (Constant)
        id_element_static = [0 0 0 1]';
    end
    properties
        name = 'SO3'; % Name of the manifold.
        id_element = [0 0 0 1]'; % Identity element.
    end
    properties (Nontunable)
        dim_manifold = 4; % Dimension of a manifold vector.
        dim_tangent = 3; % Dimension of a tangent space vector.
    end
    methods (Static)

        function expval = exp(tangent, base)
            %EXP Riemannian exponential of input.
            %
            %   :param tangent: The tangent vector to exponentiate.
            %   :param base: The point in the manifold at which to exponentiate.
            %   :return: The exponentiated value.

            % check input dimensions
            pglfd.manif.manifold.Manifold.check_tangent_shape_static(pglfd.manif.manifold.SO3Manifold, tangent);
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, base);

            % move to base
            expval = pglfd.geom.so3.quaternion_multiply(base, pglfd.geom.so3.exp(tangent));
        end

        function logval = log(point, base)
            %LOG Riemannian logarithm of input.
            %
            %   :param point: The point on the manifold to take the logarithm of.
            %   :param base: The point on the manifold at which to take the logarithm.
            %   :return: The tangent-space vector.

            % check input dimensions
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, point);
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, base);

            % move to base
            logval = pglfd.geom.so3.log(pglfd.geom.so3.quaternion_multiply(pglfd.geom.so3.quaternion_inverse(base), point));
        end

        function actionval = action(point, g, h)
            %ACTION Action function, move point relative to g to h.
            %
            %   :param point: The manifold element to transform.
            %   :param g: Point in the manifold from which to transform.
            %   :param h: Point in the manifold to which to transform.
            %   :return: Transformed manifold elements.

            % check input dimensions
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, point);
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, g);
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, h);

            actionval = pglfd.geom.so3.quaternion_multiply( ...
                pglfd.geom.so3.quaternion_multiply(h, pglfd.geom.so3.quaternion_inverse(g)), point);
        end

        function parallel_transport_trafo_val = parallel_transport_trafo(g, h)
            %PARALLEL_TRANSPORT_TRAFO The parallel transport matrix.
            %   Parallel transport transformation matrix for tangent elements from manifold point g to h.
            %
            %   :param g: The point in the manifold from which to transform.
            %   :param h: The point in the manifold to which to transform.
            %   :return: Transformation matrix.

            % check input dimensions
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, g);
            pglfd.manif.manifold.Manifold.check_manifold_shape_static(pglfd.manif.manifold.SO3Manifold, h);

            parallel_transport_trafo_val = pglfd.geom.so3.parallel_transport(g, h);
        end
    end
end
