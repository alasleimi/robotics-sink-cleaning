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

classdef (Abstract) Manifold < matlab.System
    %MANIFOLD Manifold base class.
    %   This class defines basic properties and methods of a manifold.

    properties (Abstract)
        name % Name of the manifold.
        id_element % Identity element.
        dim_manifold % Dimension of a manifold vector.
        dim_tangent % Dimension of a tangent space vector.
    end

    methods
        function check_tangent_shape(obj, point)
            if size(point, 1) ~= obj.dim_tangent || ...
                    (size(point, 2) ~= 1 && size(point, 2) ~= obj.dim_tangent)
                error(['Tangent shape mismatch, expected ' ...
                    pglfd.util.num2str(obj.dim_tangent) ' but got ' ...
                    pglfd.util.num2str(size(point, 1))]);
            end
        end

        function check_manifold_shape(obj, point)
            if size(point, 1) ~= obj.dim_manifold
                error(['Manifold shape mismatch, expected ' ...
                    pglfd.util.num2str(obj.dim_manifold) ' but got ' ...
                    pglfd.util.num2str(size(point, 1))]);
            end
        end

        function parallel_transport_val = parallel_transport(obj, input, g, h)
            %PARALLEL_TRANSPORT Calculate the parallel transport from g to h.
            %
            %   :param input: The vector or matrix to transport.
            %   :param g: Tangent space of the tangent vector / matrix.
            %   :param h: Target tangent space.
            %   :return: The parallel-transported value.

            % check input dimensions
            obj.check_tangent_shape(input);
            obj.check_manifold_shape(g);
            obj.check_manifold_shape(h);

            % calculate parallel transport
            pt_trafo = obj.parallel_transport_trafo(g, h);
            if size(input, 2) == 1
                parallel_transport_val = pt_trafo * input;
            elseif size(input, 2) == obj.dim_tangent
                parallel_transport_val = pt_trafo * input * pt_trafo';
            else
                error(['Wrong dimension for parallel transport ' size(input)]);
            end
        end
    end
    methods (Static)
        function check_tangent_shape_static(cls, point)
            if size(point, 1) ~= cls.dim_tangent || ...
                    (size(point, 2) ~= 1 && size(point, 2) ~= Manifold.dim_tangent)
                error(['Tangent shape mismatch, expected ' ...
                    pglfd.util.num2str(cls.dim_tangent) ' but got ' ...
                    pglfd.util.num2str(size(point, 1))]);
            end
        end

        function check_manifold_shape_static(cls, point)
            if size(point, 1) ~= cls.dim_manifold
                error(['Manifold shape mismatch, expected ' ...
                    pglfd.util.num2str(cls.dim_manifold) ' but got ' ...
                    pglfd.util.num2str(size(point, 1))]);
            end
        end
    end
    methods (Abstract)
        expval = exp(obj, tangent, base)
        logval = log(obj, point, base)
        actionval = action(obj, point, g, h)
        parallel_transport_trafo_val = parallel_transport_trafo(obj, g, h)
    end
end
