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

function [expval] = exp(tangent)
%EXP Calculate the SO(3) exponential at the identity.
%
%   :param tangent: The tangent vector to exponentiate.
%   :return: The SO(3) element.

theta = norm(tangent);
theta_sq = theta^2;
if theta_sq < 1e-10^2
    % taylor expansion for theta^2 < eps^2
    theta_po4 = theta_sq^2;
    imag_factor = 0.5 - 1 / 48 * theta_sq + 1 / 3840 * theta_po4;
    real_factor = 1 - 1 / 8 * theta_sq + 1 / 384 * theta_po4;
else
    half_theta = 0.5 * theta;
    imag_factor = sin(half_theta) / theta;
    real_factor = cos(half_theta);
end
expval = [tangent * imag_factor; real_factor];
end
