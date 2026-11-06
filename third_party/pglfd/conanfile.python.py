import os

from conan.tools.build import can_run
from conan.tools.files import chdir

from metadata import PglfdMetaConan


class PglfdPythonConan(PglfdMetaConan):
    package_type = "application"
    python_requires = "cissy_conantools/[~1]@tools/stable"

    @property
    def cissy_conantools(self):
        return self.python_requires["cissy_conantools"].module

    # ######################################################### #
    #                package meta-information
    # ######################################################### #

    # * package name (string) [required]
    #       name of the package
    name = "pglfd"

    # * description (string) [recommended]
    description = (
        "Set of python utils for end-effector data manipulation: loading, preprocessing, so(3)/se(3) operations, etc."
    )

    # * topics (tuple of strings) [optional]:
    #       tags that describe the package
    #       e.g. topics = ("<Put some tag here>", "<here>", "<and here>")
    topics = ("python", "manifolds", "probabilistic lfd")

    # * requires (list) [optional]
    #       package dependencies
    #       use conan syntax, e.g. "OtherLib/2.1@otheruser/testing"
    #
    requires = []

    # * tool_requires (list) [optional]
    #       build tool requirements, i.e.
    #       all packages that are needed to create the package (e.g. dev tools, compilers,
    #       build systems, code analyzers)
    #
    #       Attention: Those will not be passed downstream. Never attempt to link to a library
    #       coming from a tool requirement, even not a static one. Never read build artifacts
    #       from a tool requirement.
    #
    #       use conan syntax, e.g. "OtherLib/2.1@otheruser/testing"
    tool_requires = []

    # ######################################################### #
    #               build system information
    # ######################################################### #

    # * options (dict of <option key : <value range>) [optional]:
    #       defines options that are made public
    #       e.g. options = {"shared": [True, False]}
    # default: empty
    options = {
        "cpp_impl": [False, True],
    }

    # * default_options (dict of <option key> ; <default value>) [optional]:
    #       specifies the default value for all options
    #       e.g. default_options = {"shared": False}
    # default: empty
    default_options = {
        "cpp_impl": False,
    }

    # * settings (tuple of strings) [recommended]:
    #       defines what settings will trigger a package rebuild
    # default: ???
    settings = "os", "compiler", "build_type", "arch"

    # * export_sources (array of filesU/pattern) [required]:
    #       defines all files that are part of the source package
    # default: export everything in the directory that is not excluded by .gitignore
    exports_sources = ["python/*", "!python/test_package", "Makefile"]

    #
    # * no_copy_source (boolean) [optional]
    # by default the collected source files (from export_sources) are
    # copied to a source directory in your conan cache
    # and copied again to the build directory.
    # turning on this option, disables the second copy to the build directory.
    # this can be useful if the sources are quite large and the build system supports
    # a build not in source directory.
    #
    # no_copy_source = True

    # * source_dir (path):
    #       path to sources
    source_dir = "python"

    # ######################################################### #
    #            common functions for building and packaging
    # ######################################################### #
    def configure(self):
        if self.options.get_safe("cpp_impl"):
            self.options["pglfd_cpp"].pybind = True

    def source(self):
        """source function:
        this function is used for providing and manipulating the source files
        using cissy.conantools, the source is already available in a
        subfolder, defined by 'self.source_dir' (default folder is 'source').
        (see https://docs.conan.io/2/reference/conanfile/methods/source.html)
        """

        pass

    def package_info(self):
        """package_info function:
        Finally, the package_info() method defines
        e.g. that a consumer must link with a certain library when using this package.
        Other information as include or lib paths can be defined as well.
        (see https://docs.conan.io/2/reference/conanfile/methods/package_info.html)
        """

        # auto-set self.cpp_info and self.runenv_info / self.buildenv_info
        # (see cissy.conantools.autoset_package_info for details)
        # if you want to set individual environments or cpp variables,
        # see https://docs.conan.io/2/reference/conanfile/attributes.html#cpp-info
        # and https://docs.conan.io/2/reference/conanfile/attributes.html#runenv-info
        # and https://docs.conan.io/2/reference/conanfile/attributes.html#buildenv-info
        #
        self.cissy_conantools.autoset_package_info(self)

        if "python" in os.listdir(self.package_folder):
            # probably installed as editable
            self.runenv_info.append_path("PYTHONPATH", os.path.join(self.package_folder, "python"))

    # necessary generators:
    #       AutotoolsDeps generates an shell environment with all neccessary flags to compile C extensions
    #       (see https://docs.conan.io/2/reference/tools/gnu/autotoolsdeps.html)
    generators = "VirtualRunEnv", "AutotoolsDeps"

    def requirements(self):
        # Require C++ implementations if needed
        if self.options.get_safe("cpp_impl"):
            self.requires(f"pglfd_cpp/{self.version or 'master'}@{self.user}/{self.channel}")

    def build(self):
        """build function:
        the build() should configure and build the project
        (see https://docs.conan.io/2/reference/conanfile/methods/build.html)
        """

        # call "pip install"
        with chdir(self, self.source_dir):
            self.run(
                f"python3 -m pip install --no-deps --ignore-installed --prefix {self.package_folder} .",
                env="conanbuild",
            )

        # Run tests only if not disabled
        if can_run(self) and int(os.getenv("CONAN_RUN_TESTS", True)):
            # Run linting
            try:
                self.run("make lint", env="conanbuild")
                self.output.success("Code linting finished successfully.")
            except Exception as e:
                self.output.error("Please check the code linting!")
                raise e

            # run mypy
            try:
                self.run(f"HAVE_CPP={self.options.cpp_impl} make mypy", env="conanrun")
                self.output.success("Typing completed succesfully!")
            except Exception:
                self.output.error("Please check code typing.")

            # run mypy for missing annotations
            try:
                self.run("make missing-annotations", env="conanrun")
                self.output.success("No missing annotations!")
            except Exception:
                self.output.error("Please check for missing annotations.")

            # run pytest
            try:
                self.run("make pytest", env="conanrun")
                self.output.success("Unit tests completed succesfully!")
            except Exception as e:
                self.output.error("Some unit tests failed.")
                raise e

    def package(self):
        """package function:
        The package() method should copy artifacts (headers, libs)
        from the build folder to the final package folder.
        (see https://docs.conan.io/2/reference/conanfile/methods/package.html)
        """

        # package is already installed by pip
        pass
