#!/bin/bash

for file in *.svg; do
    filename="${file%.svg}"
    echo ${filename}
    inkscape "$file" -d 1200 --export-filename="${filename}.pdf"
    inkscape "$file" -d 1200 --export-filename="${filename}.png"
done

rm *.svg
mv *.png ../docs/modules/ctrl/images/
mv *.pdf ../docs/modules/ctrl/images/