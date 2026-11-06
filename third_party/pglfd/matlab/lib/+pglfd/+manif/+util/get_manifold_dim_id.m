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

function [dim_manifold, dim_tangent, man_id] = get_manifold_dim_id(description)
%GET_MANIFOLD_DIM_ID Get manifold dimensions and id.
%   Constant manifold dimensions and id for use within simulink.
%
%   :param description: The manifold description to extract from.
%   :return: Dimension of manifold, tangent space, and manifold id.

manifolds = pglfd.manif.util.get_submanifolds(description);
dim_manifold = 0;
dim_tangent = 0;
man_id = -1;
for man = manifolds
    dim_manifold = dim_manifold + man{1}.dim_manifold;
    dim_tangent = dim_tangent + man{1}.dim_tangent;
end

if strcmp(description, 'R3*SO3')
    man_id = 0;
elseif strcmp(description, 'SO2*R2*SO3')
    man_id = 1;
elseif strcmp(description, 'S2*R1*SO3')
    man_id = 2;
end
end
