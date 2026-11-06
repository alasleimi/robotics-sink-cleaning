"""Check platform routing and extraction boundaries without network or robot access."""

from pathlib import Path
import sys

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from common import platform_key
from setup import validate_zip_member


@pytest.mark.parametrize("system,machine,expected", [
    ("Windows", "AMD64", "windows-x64"),
    ("Darwin", "arm64", "mac-arm64"),
    ("Darwin", "x86_64", "mac-x64"),
])
def test_supported_platforms(system, machine, expected):
    assert platform_key(system, machine) == expected


def test_unsupported_platform_is_explicit():
    with pytest.raises(RuntimeError, match="Unsupported platform"):
        platform_key("Windows", "ARM64")


@pytest.mark.parametrize("member", ["../escape", "dir/../../escape", "dir\\..\\..\\escape"])
def test_reject_archive_traversal(tmp_path, member):
    with pytest.raises(ValueError, match="Unsafe archive"):
        validate_zip_member(tmp_path, member)


def test_accept_nested_archive_member(tmp_path):
    validate_zip_member(tmp_path, "app/Contents/MacOS/executable")
