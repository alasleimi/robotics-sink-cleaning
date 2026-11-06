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

function export_mask_image(blk, outname)

load_system(bdroot(blk));

blkName  = get_param(blk, 'Name');

%% 2) Mask dialog ----------------------------------------------------------
open_system(blk, 'mask');
drawnow; pause(1);                      % let the dialog finish rendering

dlg = findMaskDialog(blkName);
if isempty(dlg)
    warning('exportMaskImages:noDialog', ...
        'Could not find an open mask dialog for "%s".', blkName);
    return
end

% Clear MATLAB's LD_LIBRARY_PATH so ImageMagick loads the system
% libraries instead of the ones bundled with MATLAB.
im = 'env -u LD_LIBRARY_PATH import';
conv = 'env -u LD_LIBRARY_PATH convert';
 
% Capture the dialog window by its title (import -window accepts a name).
title = strrep(char(dlg.getTitle), '"', '\"');
outname_first = [outname '_orig.png'];
system(sprintf('%s -window "%s" "%s"', ...
    im, title, outname_first));
system([conv ' ' outname_first ' -alpha set -fuzz 30% -fill none ' ...
  '-draw "color 0,0 floodfill" -draw "color 469,0 floodfill" ' ...
  '-draw "color 0,375 floodfill" -draw "color 469,375 floodfill" ' ...
  '-trim +repage ' outname '.png']);
system(['rm ' outname_first]);

delete(dlg);                            % close the dialog

end

%% ------------------------------------------------------------------------
function dlg = findMaskDialog(blkName)
% Return the open DDG dialog whose title contains the block name.
dlg  = [];
dlgs = DAStudio.ToolRoot.getOpenDialogs;
for k = 1:numel(dlgs)
    if contains(dlgs(k).getTitle, blkName)
        dlg = dlgs(k);
        return
    end
end
end