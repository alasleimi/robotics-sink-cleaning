from metadata import PglfdMetaConan


class PglfdMatlabConan(PglfdMetaConan):
    python_requires = "conan-matlab-helper/[~3]@tools/stable"
    python_requires_extend = "conan-matlab-helper.MatlabLibConanFile"

    # ######################################################### #
    #                package meta-information
    # ######################################################### #

    # * package name (string) [required]
    #       name of the package
    name = "pglfd_matlab"

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

    # * sfun_requires (list) [optional]
    #       sfunction dependencies; sfunctions are required in build context during compiling but there
    #       might be runtime dependencies required in host context at execution time.
    #       When only providing one reference, it is assumed that a "lib*" package exists providing so-files required during runtime
    #       but you can override that using ("sfun_reference/xx@yy/stable", "lib_reference/xx@yy/stable") or even
    #       omit the lib completely if the sfunction is self-contained: ("sfun_reference/xx@yy/stable", ).
    #
    sfun_requires = []

    # * export_sources (array of filesU/pattern) [required]:
    #       defines all files that are part of the source package
    # default: export everything in the directory that is not excluded by .gitignore
    exports_sources = ["matlab/*"]

    # ######################################################### #
    #       common parameters for matlab / simulink code
    # ######################################################### #

    # * matlab_dir (string) [required]
    #       directory where the library is stored, this folder is copied to the package
    matlab_dir = "matlab/lib"

    # * matlab_lib_dirs (list of strings) [optional]
    #       list of sub-directories of matlab_dir which should be added to the matlab path as well
    matlab_lib_dirs = []

    # * matlab_init_file (string) [optional]
    #       file name of the init script (relative to matlab_dir) which should be used to initialize the Simulink library and m-files
    matlab_init_file = "init_pglfd"

    # * matlab_test_file (string) [optional]
    #       file name of the test script (relative to package root) which should be used to test the Simulink library and m-files
    matlab_test_file = "matlab/tests/test_pglfd.m"
