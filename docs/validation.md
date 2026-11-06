# Validation history

## Robosuite port — 6 November 2026

Robosuite 1.5.2, MuJoCo 3.3.7 and NumPy 1.26.4 are installed through the locked uv environment. Setup/SDK checks and Java teaching-file checks passed on Windows. Fifteen Python checks cover setup/PGLfD plus actual model Jacobians, saved-pose IK, joint-limit sweeps, compliant force response, Cartesian stiffness, teaching target mapping and the robosuite reset/step interface. The native Windows viewer opened and closed successfully; its rendered scene was visually inspected. The full original exercise sequence and Java motion/force/logging assertions passed locally (`.tools/sim-jppzq3ok/report.json`), including Exercise 1 saving F0–F2/J0–J2, Exercise 2 replay, Exercise 6 force break and Exercise 7 singular postures. Exercises 3/4 remain student scaffolds and 8 covers its supplied approach. Full headless CI passed on Windows x64, Intel Mac and Apple Silicon Mac: [run 42198672432](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42198672432), tested revision `53a0590`. This includes setup, fifteen Python checks, seven Java teaching-file tests and the complete exercise integration run. The Mac desktop viewer was not manually tested. Hardware has not been connected.


## Previous CoppeliaSim setup — 6 November 2026

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

No hardware connection or motion was attempted. SDK object checks do not establish full KUKA native-library compatibility, and the initial setup checks did not cover the adapter. The starter scene has not been calibrated to the physical robot/tool/sink.

## Adapter validation â€” 6 November 2026

Local Windows runs used the real CoppeliaSim/MuJoCo runtime, not a fake bridge:

- Original Exercise 1 ran its front movement and zero-stiffness loop. Three actual poses were taught and saved through the same DataHandler methods as the GUI buttons. Original Exercise 2 loaded F0-F2/J0-J2 and replayed F0 successfully.
- Every supplied exercise main completed, or ran a bounded number of iterations for intentional loops. Exercises 3/4 remain empty templates; Exercise 8 only implements its approach/start sequence.
- Additional physical assertions covered world/tool-relative LIN, Cartesian and joint PTP, TCP/flange offset, motion batches, async completion/timeout, pause/resume/cancel, continuous blended motion, compliant displacement under applied force, force-break condition identity and logged state samples.
- All Exercise 7 singular-posture motions completed. A taught `wireloop_start` supported the original Exercise 8 approach.
- Seven teaching-file regressions passed: F0/J0 starts, numbering gaps, missing/malformed files, units, named redundancy and simulator joint-seed round-trips. Nine existing Python/setup/PGLfD tests also passed.
- Unsupported controller calls fail explicitly; a non-loopback socket attempt was blocked inside the simulator JVM. Original exercise bodies, robot configuration and PGLfD files are unchanged.
- The Java GUI launched without exceptions. Automated desktop clicking could not be checked because the computer-use helper failed to initialize (`failed to write kernel assets`); save methods and name-field updates were checked programmatically.

**Final adapter CI passed on all three platforms:** Windows x64, Intel macOS, and Apple Silicon macOS. [Run 42163084493](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42163084493) tested revision `b2b2c97` with installation, seven Java/data regressions, nine Python/setup/PGLfD tests, and full CoppeliaSim integration. It also checked persistent HTTP reuse and continued physics while a connection was idle. Local negative checks confirmed a clear missing-Exercise-2-prerequisite error and that a second exercise cannot terminate the first. Desktop GUI clicking and physical hardware remain unverified; dynamics are not calibrated to the lab.
