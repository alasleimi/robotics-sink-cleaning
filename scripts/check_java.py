"""Compile both isolated classpaths and run teaching-file regressions."""
from common import gradle, load_config

if __name__ == "__main__":
    gradle(load_config(), "test", "simClasspath")
