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

function [axis_angle] = vector_to_axis_angle(vector)
%VECTOR_TO_AXIS_ANGLE Get the axis-angle representation of vector.
%
%   :param vector: The vector to get the axis-angle representation of.
%   :return: The axis-angle representation.

normval = norm(vector(1:2));
if normval < 1e-6
    vec = pglfd.manif.manifold.S2Manifold.id_element_static;
    if vector(3) < 0
        vec = [1 0 0]';
        angle = pi;
    else
        angle = 0;
    end
else
    vec = [-vector(2); vector(1); 0] / normval;
    angle = acos(vector(3));
end
axis_angle = [vec; angle];
end
