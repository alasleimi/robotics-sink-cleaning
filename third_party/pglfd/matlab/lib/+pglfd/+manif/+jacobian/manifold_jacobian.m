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

function [jacobian, singular] = manifold_jacobian(manifold, point, eps)
%MANIFOLD_JACOBIAN Jacobian from R3*SO3 velocity to manifold at point.
%   Calculate the Jacobian to transform a velocity in the tangent space
%   of R3*SO3 to a velocity in tangent space of manifold at point.
%
%   :param manifold: Target manifold for the Jacobian.
%   :param point: The point at which to calculate the Jacobian.
%   :param eps: Epsilon value for small radii.
%   :return: The jacobian and whether it is singular.

% check input shape
manifold.check_manifold_shape(point);

if strcmp(manifold.name, 'R3*SO3')
    manifold_id = 0;
elseif strcmp(manifold.name, 'SO2*R2*SO3')
    manifold_id = 1;
elseif strcmp(manifold.name, 'S2*R1*SO3')
    manifold_id = 2;
else
    error(['Unknown Manifold ' manifold.name]);
end
[jacobian, singular] = pglfd.manif.jacobian.manifold_jacobian_by_id(manifold_id, manifold, point, eps);
end
