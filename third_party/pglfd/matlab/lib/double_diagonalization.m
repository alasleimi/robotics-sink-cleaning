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

function [Q, D] = double_diagonalization(A, B)
%DOUBLE_DIAGONALIZATION Calculate the double diagonalization.

% A. Albu-Schaffer, C. Ott, U. Frese and G. Hirzinger,
% "Cartesian impedance control of redundant robots: recent results with the DLR-light-weight-arms,"
% 2003 IEEE International Conference on Robotics and Automation,
% Taipei, Taiwan, 2003, pp. 3704-3709 vol.3, doi: 10.1109/ROBOT.2003.1242165.

L = chol(A, 'lower');
Li = inv(L);
C = Li * B * Li';
[U, D] = eig(C);
Q = L * real(U);
D = real(D);

end
