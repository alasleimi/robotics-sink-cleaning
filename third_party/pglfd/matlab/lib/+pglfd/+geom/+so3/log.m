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

function [logval] = log(logpoint)
%LOG calculate the SO(3) logarithm at theidentity.
%
%   :param logpoint: The SO(3) element to take the log of.
%   :return: The tangent vector w.r.t. the identity.

n = norm(logpoint(1:3));
if n < 1e-10
    % taylor expansion for norm < eps
    two_atan_nbyw_by_n = 2 / logpoint(4) - 2 / 3 * n^2 / (logpoint(4)^3);
else
    if logpoint(4) < 0
        atan_nbyw = atan2(-n, -logpoint(4));
    else
        atan_nbyw = atan2(n, logpoint(4));
    end
    two_atan_nbyw_by_n = 2 * atan_nbyw / n;
end
logval = two_atan_nbyw_by_n * logpoint(1:3);
end
