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

function [r3_so3] = manifold_to_r3_so3(manifold, point)
%MANIFOLD_TO_R3_SO3 Convert a manifold element to ``R3*SO3``.
%   Transform an element from the manifold described by manifold to its
%   equivalent element on ``R3*SO3`` at the same origin.
%
%   :param manifold: The manifold the vector resides in.
%   :param point: The vector on the manifold.
%   :return: The vector in ``R3*SO3``.

% check inputs
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
r3_so3 = pglfd.manif.conversions.manifold_to_r3_so3_by_id(manifold_id, manifold, point);
end
