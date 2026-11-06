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

function blkStruct = slblocks
blkStruct.Name = 'pglfd';
blkStruct.OpenFcn = {'pgtrafo', 'pgmanif', 'pgctrl'};

% pgtrafo
Browser(1).Library = 'pgtrafo';
Browser(1).Name = 'pgtrafo';
Browser(1).IsFlat = 1;

% pgmanif
Browser(2).Library = 'pgmanif';
Browser(2).Name = 'pgmanif';
Browser(2).IsFlat = 1;

% pgctrl
Browser(3).Library = 'pgctrl';
Browser(3).Name = 'pgctrl';
Browser(3).IsFlat = 1;

blkStruct.Browser = Browser;
end
