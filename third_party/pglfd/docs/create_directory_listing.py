#!/usr/bin/env python3
import os

with open("index.html", "w") as f:
    f.write(
        """<!DOCTYPE html>
<html lang="en-US">
<head>
	<title>Branches of pglfd docs</title>
	<meta charset="utf-8" />
	<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate" />
	<meta http-equiv="Pragma" content="no-cache" />
	<meta http-equiv="Expires" content="0" />
	<link rel="stylesheet" type="text/css" href="main/format.css" />
</head>
<body>
	<h2>Branches</h2>
	<div>If you don't know what to choose, please choose '<a href="main/index.html">main</a>'.</div>
	<ul style="font-size: large;">"""
    )
    for element in sorted(os.listdir(os.getcwd())):
        if os.path.isdir(element) and not element.startswith("."):
            emph_element = element
            if element == "main":
                emph_element = "<strong>" + emph_element + "</strong>"
            f.write("\n		<li><a href=" + element + "/index.html" + ">" + emph_element + "</a></li>")

    f.write(
        """
	</ul>
</body>
</html>
"""
    )
