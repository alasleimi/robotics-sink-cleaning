from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.files import copy


class PglfdTestConan(ConanFile):
    test_type = "explicit"
    settings = "os", "compiler", "build_type", "arch"
    generators = "VirtualRunEnv"

    def requirements(self) -> None:
        self.requires(self.tested_reference_str)

    def build(self) -> None:
        if self.source_folder != self.build_folder:
            copy(self, "load_pglfd.py", self.source_folder, self.build_folder)

    def test(self) -> None:
        if can_run(self):
            self.run("python3 load_pglfd.py", env="conanrun")
