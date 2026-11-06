import os

from conan.tools.build import can_run
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import replace_in_file

from metadata import PglfdMetaConan


class PglfdCppConan(PglfdMetaConan):
    package_type = "shared-library"
    python_requires = "cissy_conantools/[~1]@tools/stable"

    @property
    def cissy_conantools(self):
        return self.python_requires["cissy_conantools"].module

    # ######################################################### #
    #                package meta-information
    # ######################################################### #

    # * package name (string) [required]
    #       name of the package
    name = "pglfd_cpp"

    # * description (string) [recommended]
    description = "<Description of Testproject here>"

    # * topics (tuple of strings) [optional]:
    #       tags that describe the package
    #       e.g. topics = ("<Put some tag here>", "<here>", "<and here>")
    topics = ()

    # * requires (list) [optional]
    #       package dependencies
    #       use conan syntax, e.g. "OtherLib/2.1@otheruser/testing"
    #
    requires = []

    # * tool_requires (list) [optional]
    #       package build dependencies, i.e.
    #       all packages that are needed to create the package (e.g. dev tools, compilers,
    #       build systems, code analyzers, testing libraries)
    #
    #       Attention: build dependencies will not be passed downstream, i.e.
    #       to generators or packages requiring this one.
    #
    #       use conan syntax, e.g. "OtherLib/2.1@otheruser/testing"
    #
    tool_requires = []

    # ######################################################### #
    #               build system information
    # ######################################################### #

    # * settings (tuple of strings) [recommended]:
    #       defines ???
    # default: ???
    settings = "os", "compiler", "build_type", "arch"

    # * exports_sources (array of files/pattern) [required]:
    #       defines all files that are part of the source package
    # default: export everything in the directory that is not excluded by .gitignore
    exports_sources = ["cpp/*", "!cpp/test_package", "Makefile"]

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

    # * options (dict of <option key : <value range>) [optional]:
    #       defines options that are made public
    #       e.g. options = {"shared": [True, False]}
    # default: empty
    options = {"pybind": [True, False]}

    # * default_options (dict of <option key> ; <default value>) [optional]:
    #       specifies the default value for all options
    #       e.g. default_options = {"shared": False}
    # default: empty
    default_options = {"pybind": False}

    # * source_dir (path):
    #       path to sources
    source_dir = "cpp"

    # ######################################################### #
    #            common functions for building and packaging
    # ######################################################### #

    # source function:
    #       this function is used for providining and manipulating the source files
    #       using the cissy-3rdpyart-cli, the source is already available in a
    #       subfolder, defined by 'self.source_dir' (default folder is 'source').
    #       the functions automatically applies all patches that are found in a
    #       'patches' subfolder.
    #       (see https://docs.conan.io/en/latest/reference/conanfile/methods.html#source)
    #
    def source(self):
        if self.cissy_conantools.Version(self.version).is_semantic_version():
            # Write version info into CMake
            replace_in_file(self, os.path.join(self.source_folder, self.source_dir, "CMakeLists.txt"), "0.0.0", self.version)

        # Run linting
        try:
            self.run("make cpp-lint", env="conanbuild")
            self.output.success("Code linting finished successfully.")
        except Exception as e:
            self.output.error("Please check the code linting!")
            raise e

    # package_info function:
    #       Finally, the package_info() method defines
    #       e.g. that a consumer must link with a certain library when using this package.
    #       Other information as include or lib paths can be defined as well.
    #       (see https://docs.conan.io/en/latest/reference/conanfile/methods.html#package-info)
    #
    def package_info(self):
        #
        # auto-set self.cpp_info and self.env_info
        # (see cissy.conantools.autoset_package_info for details)
        # if you want to set individual environments or cpp variables,
        # see https://docs.conan.io/en/latest/reference/conanfile/attributes.html#cpp-info
        # and https://docs.conan.io/en/latest/reference/conanfile/attributes.html#env-info
        #
        self.cissy_conantools.autoset_package_info(self, cmake_name="pglfd")

    # ######################################################### #
    #        cmake specific functions and variables
    # ######################################################### #

    #
    # necessary generators:
    #       cmake_path sets module and prefix path (used at build stage)
    #       (see https://docs.conan.io/en/latest/reference/generators/cmake_paths.html)
    #
    generators = "CMakeDeps", "VirtualBuildEnv"

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_PYTHON"] = self.options.pybind
        tc.generate()

    # build function:
    #       the build() should configure and build the project
    #       (see https://docs.conan.io/en/latest/reference/conanfile/methods.html#build)
    #
    def build(self):
        #
        # CMake build
        #
        cmake = CMake(self)
        cmake.configure(build_script_folder=self.source_dir)

        # get number of cores to use to build
        build_args = []
        if self.options.pybind or os.getenv("PGLFD_BUILD_NUM_CORES") is not None:
            build_args = ["-j" + os.getenv("PGLFD_BUILD_NUM_CORES", default="4")]

        # build
        cmake.build(build_tool_args=build_args)
        if can_run(self) and int(os.getenv("CONAN_RUN_TESTS", True)):
            cmake.test()

    # package function:
    #       The package() method should copy artifacts (headers, libs)
    #       from the build folder to the final package folder.
    #       (see https://docs.conan.io/en/latest/reference/conanfile/methods.html#package)
    #
    def package(self):
        #
        # execute 'make install'
        #
        cmake = CMake(self)
        cmake.install()
