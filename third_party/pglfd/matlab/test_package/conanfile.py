import os

from conan import ConanFile


class PglfdTestConan(ConanFile):
    python_requires = "conan-matlab-helper/[~3]@tools/stable"
    python_requires_extend = "conan-matlab-helper.MatlabConanFile"
    test_type = "explicit"

    def requirements(self) -> None:
        self.tool_requires(self.tested_reference_str)

    def build(self) -> None:
        matlab_script = os.path.join(self.recipe_folder, "load_pglfd.m")
        self.run_matlab(f"run('{matlab_script}');")

    def test(self) -> None:
        # nothing left to be done
        pass
