# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys

sys.path.insert(0, os.path.abspath("_ext"))

# -- Path setup --------------------------------------------------------------

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))


# -- Project information -----------------------------------------------------

project = "pglfd"
copyright = "2026, Maximilian Mühlbauer"
author = "Maximilian Mühlbauer"

# The full version, including alpha/beta/rc tags
release = os.getenv("BRANCH_NAME", "main")


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx.ext.autodoc",
    "sphinxcontrib.lightbox2",
    "breathe",
    "sphinxcontrib.matlab",
    "matlab_sourcelinks",  # Claude-generated magic to add Matlab source
    "matlab_unique_ids",  # Claude-generated magic to fix Matlab id clashes with Python
    "sphinxcontrib.bibtex",
    "sphinxcontrib.jquery",
]

todo_include_todos = True

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

source_suffix = ".rst"
master_doc = "index"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

autoclass_content = "both"

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ["_static"]
autodoc_member_order = "bysource"
autodoc_mock_imports = ["matplotlib", "mpl_toolkits"]

html_static_path = ["_static"]


def setup(app):
    app.add_css_file("custom.css")


# configure breathe
cpp_index_common_prefix = [
    "pglfd::",
]
breathe_projects = {"pglfd_cpp": os.path.join(os.path.dirname(__file__), "docs_cpp", "xml")}
breathe_default_project = "pglfd_cpp"

# configure matlab
matlab_src_dir = os.path.join(os.path.dirname(__file__), "..", "matlab", "lib")
matlab_short_links = False
matlab_auto_link = "all"
matlab_class_signature = True
matlab_keep_package_prefix = False


#### hack by Claude - give a toc to matlab entries ####
# Give MATLAB domain objects table-of-contents entries
from sphinxcontrib import matlab as _mat


def _object_hierarchy_parts(self, sig_node):
    if "fullname" not in sig_node:
        return ()
    mod = sig_node.get("module")
    full = sig_node["fullname"]
    return (mod, *full.split(".")) if mod else tuple(full.split("."))


def _toc_entry_name(self, sig_node):
    if not sig_node.get("_toc_parts"):
        return ""
    cfg = self.config
    objtype = sig_node.parent.get("objtype")
    parens = "()" if (cfg.add_function_parentheses and objtype in {"function", "method", "staticmethod"}) else ""
    *parents, name = sig_node["_toc_parts"]
    mode = cfg.toc_object_entries_show_parents
    if mode == "domain":
        return sig_node.get("fullname", name) + parens
    if mode == "hide":
        return name + parens
    if mode == "all":
        return ".".join([*parents, name + parens])
    return ""


_mat.MatObject._object_hierarchy_parts = _object_hierarchy_parts
_mat.MatObject._toc_entry_name = _toc_entry_name

# configure bibtex
bibtex_bibfiles = ["refs_algorithms.bib", "refs_projects.bib"]
bibtex_default_style = "plain"

# configure latex
latex_elements = {
    "preamble": r"""
    \addto\captionsenglish{\renewcommand{\contentsname}{Table of contents}}
    \setcounter{tocdepth}{5}

    % for wrapfill
    \usepackage{etoolbox}
    \makeatletter
    % End any active wrapfigure: skip down past it, then reset wrapfig
    \newcommand\wrapfill{\par
    \ifx\parshape\WF@fudgeparshape
        \nobreak
        \vskip-\baselineskip
        \vskip\c@WF@wrappedlines\baselineskip
        \allowbreak
        \WFclear
    \fi}
    \makeatother
    % Equivalent of "clear: both" before every heading
    \pretocmd{\section}{\wrapfill}{}{}
    \pretocmd{\subsection}{\wrapfill}{}{}
    \pretocmd{\subsubsection}{\wrapfill}{}{}
    \pretocmd{\paragraph}{\wrapfill}{}{}
    """,
    "atendofbody": r"""
    \part{Indices and Tables}
    \clearpage
    \phantomsection
    \addcontentsline{toc}{chapter}{\listfigurename}
    \listoffigures
    \clearpage
    \phantomsection
    \addcontentsline{toc}{chapter}{\listtablename}
    \listoftables
    """,
}
