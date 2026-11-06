SHELL=/bin/bash
LINT_PATHS=python/pglfd/ python/examples/ python/tests/
PY_CFG=python/pyproject.toml
CPP_CFG=cpp/pyproject.toml
CPP_LINT_PATHS=cpp/

local_install:
	mkdir -p build/pglfd && cd build/pglfd && cmake ../../cpp && make -j6 && cmake --install . --prefix ./install

pytest:
	cd python && python3 -m pytest --cov-config .coveragerc --cov-report html --cov-report term --cov=. -v --color=yes --durations=10 $(args)

gdb-pytest:
	cd python && gdb -ex run -ex bt -ex quit --args python3 -m pytest --cov-config .coveragerc --cov-report html --cov-report term --cov=. -v --color=yes --durations=10 $(args)

mypy:
	mypy ${LINT_PATHS} --config-file ${PY_CFG}

missing-annotations:
	mypy --disallow-untyped-calls --disallow-untyped-defs --ignore-missing-imports python/pglfd python/examples --config-file ${PY_CFG}

type: mypy

lint:
	# stop the build if there are Python syntax errors or undefined names
	# see https://www.flake8rules.com/
	ruff check ${LINT_PATHS} --select=E9,F63,F7,F82 --output-format=full
	# exit-zero treats all errors as warnings.
	ruff check ${LINT_PATHS} --exit-zero --output-format=pylint --output-file build/issues-ruff.txt

cpp-lint:
	ruff check ${CPP_LINT_PATHS} --exit-zero --output-format=pylint --output-file build/issues-ruff.txt
	mypy ${CPP_LINT_PATHS} --config-file ${CPP_CFG}
	mypy --disallow-untyped-calls --disallow-untyped-defs --ignore-missing-imports cpp/ --config-file ${CPP_CFG} || true

format:
	# Sort imports
	ruff check --select I ${LINT_PATHS} --fix
	# Reformat + check
	ruff format ${LINT_PATHS}
	ruff check --select I ${CPP_LINT_PATHS} --fix
	ruff format ${CPP_LINT_PATHS}
	# Format the C++ code
	cd ${CPP_LINT_PATHS} && find . -iname "*.?pp" -or -iname "*.h" | xargs clang-format -verbose -i

matlab-format:
	cissy run -au -add-pr matlab2025b_acad -kp mbeautifier matlab_acad -batch "MBeautify.formatFiles(\"matlab\", \"*.m\", true, false); exit;"

check-codestyle:
	# Sort imports
	ruff check --select I ${LINT_PATHS}
	# Check formatting
	ruff format --check ${LINT_PATHS}

iwyu:
	mkdir -p build/iwyu
	cd build/iwyu && cmake -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DWITH_OPENMP=OFF ../../cpp
	cd build/iwyu && iwyu_tool -j8 -p . -- -Xiwyu --mapping_file=../../cpp/iwyu.imp -Xiwyu --check_also="**/pglfd/**/*.h" -Xiwyu --transitive_includes_only -Xiwyu --no_fwd_decls -Xiwyu --cxx17ns

commit-checks: format type lint

SOURCEDIR         = docs
BUILDDIR          = build
GH_PAGES_DIR_NAME = gh-pages
BRANCH_DIR_NAME   = $(subst /,_,$(BRANCH_NAME))
DOCS              = "$(GH_PAGES_DIR_NAME)/$(BRANCH_DIR_NAME)"
SPHINXOPTS        ?=
SPHINXBUILD       ?= python3 -m venv .venv/ && source .venv/bin/activate && pip3 install -r docs/requirements.txt && sphinx-build

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

doc:
	@cd docs && doxygen Doxyfile.in
	@make html

pdf-doc:
	@cd docs && doxygen Doxyfile.in
	@make latexpdf

gh-pages:
	@rm -rf "$(GH_PAGES_DIR_NAME)"
	@git clone `git config --get remote.origin.url` --branch gh-pages --depth 1 gh-pages
	@rm -rf "$(BUILDDIR)/html"
	@rm -rf "$(DOCS)"
	@make doc
	@mv "$(BUILDDIR)/html" $(DOCS)
	@cd "$(GH_PAGES_DIR_NAME)" && python3 ../docs/create_directory_listing.py
	# add updated branch listing
	@git -C $(GH_PAGES_DIR_NAME) add index.html
	@git -C $(GH_PAGES_DIR_NAME) add "$(BRANCH_DIR_NAME)/*"
	@-git -C $(GH_PAGES_DIR_NAME) commit -m 'Automatic update of GH pages'
	# rebase as builds on other branches might have changed things in the meantime
	# todo: make sure we really cannot get race conditions by putting the pull in a loop in case push fails?
	# todo: do we need to handle merges? can conflicts happen in with the index.html listing the branches?
	@git -C $(GH_PAGES_DIR_NAME) pull --rebase
	@git -C $(GH_PAGES_DIR_NAME) push

clean:
	cd docs && make clean

.PHONY: clean spelling doc lint format check-codestyle commit-checks
