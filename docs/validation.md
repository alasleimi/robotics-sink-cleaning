# Setup validation

## 6 November 2026

Checked locally on Windows x64:

- `uv run --locked scripts/setup.py` completed, including Java compilation and SDK geometry/motion-object construction. Setup was rerun successfully using cached installations.
- Python imports passed for NumPy, Matplotlib, PGLfD geometry/KMP, our package, and the CoppeliaSim ZeroMQ client.
- Eight setup-helper tests and the supplied PGLfD geometry test passed.
- CoppeliaSim Edu 4.10.0 rev0 launched in headless mode and answered the local ZeroMQ API with version `41000`.
- Loaded the iiwa 7 R800 model, found seven joints, selected MuJoCo, saved `simulation/scenes/iiwa7.ttt`, and advanced physics for ten steps (0.50 seconds).
- Reopened the saved scene through the same command-line loading mechanism used by the launcher; seven joints and MuJoCo selection were retained. Added the Windows Start menu shortcut **CoppeliaSim - Sink Cleaning**. Temporary check processes were closed.
- All 519 supplied PGLfD files matched the local Moodle extraction by SHA-256 before committing.
- Git tracks 501 PGLfD source/data files; the other 18 are supplied Ruff cache files. Whitespace warnings in the preserved vendor documentation were not rewritten.

**All three GitHub Actions jobs passed:** Windows, Intel macOS, and Apple Silicon macOS. [Successful run 42155500504](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42155500504) tested code revision `46ed7215129a8022cb25ee5f10c86537122e5e9e`.

The workflow checked installation, Java compilation/SDK objects, Python imports, and tests on each platform. Mac simulator GUI/dynamics are not covered by that workflow.

No hardware connection or motion was attempted. SDK object checks do not establish full KUKA native-library compatibility, and the Sunrise simulation adapter remains unimplemented. The starter scene has not been calibrated to the physical robot/tool/sink.
