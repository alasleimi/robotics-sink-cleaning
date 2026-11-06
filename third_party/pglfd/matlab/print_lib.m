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

addpath('lib');

for sys = {'pgctrl', 'pgmanif', 'pgtrafo'}
    sysname = sys{1};
    disp(sysname);
    load_system(sysname);
    for block = find_system(sysname, 'SearchDepth', 1)'
        blockname = block{1};
        disp(blockname);
        svgfile = '';
        if ~strcmp(blockname, sysname)
            newsys = matlab.lang.makeValidName(strrep(blockname, '/', '_'));
            newblock = [newsys '/' strrep(blockname, [sysname '/'], '')];
            h = new_system(newsys);
            add_block(blockname, newblock);
            set_param(newblock, 'HideAutomaticName', 'off');
            set_param(newblock, 'ShowName', 'on');
            svgfile = [newsys '.svg'];
            print('-dsvg', '-r300', '-bestfit', ['-s' newsys], svgfile);
            if ~isempty(Simulink.Mask.get(newblock))
                export_mask_image(newblock, [newsys '_mask']);
            end
            close_system(h, 0);
        else
            load_system(sysname);
            svgfile = [sysname '.svg'];
            print('-dsvg', '-r300', '-bestfit', ['-s' sysname], svgfile);
            close_system(sysname, 0);
        end
        % make the svg transparent
        txt = fileread(svgfile);

        % replace opacity
        txt = regexprep(txt, 'fill-opacity="1"', 'fill-opacity="0"', 'once');

        % write back the transparent svg
        fid = fopen(svgfile, 'w');
        fwrite(fid, txt, 'char');
        fclose(fid);
    end
end
