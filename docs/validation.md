# Validation history

## Optional demonstration telemetry and recorder cleanup — 10 November 2026

Split the recorder into the workflow (`Record`), an explicit SDK channel list (`RecordingTelemetry`), and optional-read handling (`OptionalTelemetry`). Native hand-guiding and the shared real/simulator loop remain. New takes keep the 22 CSV columns and use schema 2 to allow NaN in missing joint/wrench cells; measured tool pose and timing remain required. Extra readings and per-channel timing/errors go to `telemetry.jsonl`. Setup configuration is copied when available. Optional SDK failures disable only that channel; reads over 100 ms are retained once then disabled. A blocked SDK call still relies on the SDK returning; this is not a hard timeout.

Local Windows validation passed:

- **25 Python checks and 16 Java tests**: legacy/new formats, missing optional values, rejection of missing required pose, unavailable SDK methods, null/invalid values, disabling slow channels, and preservation of disk-write failures.
- Full recorder/replayer integration (`.tools/sim-lu7ecw8t/report.json`): extended telemetry, original recording/replay, switch/Enter/time stopping, forced missing joint/wrench/torque APIs and missing calibration, successful pose-only replay, cancellation and connection loss. Faults were injected only into the simulator's mocked SDK methods; the actual recorder executed unchanged.
- KMP/TP-KMP training plus raw and learned path replay passed (`.tools/sim-9b361gxr/learning-report.json`). A separate training run on schema-2 copies of all five examples with every joint/wrench cell set to NaN produced valid KMP and TP-KMP predictions (`.tools/optional-kmp-ibqr4lba`).
- The complete original exercise/contact regression also passed (`.tools/sim-mnyyh4r3/report.json`), including saved poses, E1, motion batches, force breaks and wire contact. Student exercise sources and hardware configuration are unchanged.

No real robot was connected. Real sensor availability/rates remain unverified. Simulator temperature/safety/commanded-pose channels that it does not implement are reported unavailable, not filled with guessed values. The new code's cross-platform CI is pending.

## Recording controls and failure handling — 10 November 2026

The recording panel now selects the green target automatically. Releasing hand-guiding, clicking **Stop & save**, or pressing Enter saves the take and leaves the scene open. **Close simulator** closes it explicitly. The actual Sunrise Java recorder is shared by both backends; the simulator UI supplies the same stop flag as the console.

Checked KUKA's bundled javadocs and corrected an adapter mismatch: `await()` and `validate()` now throw the SDK's cancellation/invalid-command exceptions. Previously cancellation could incorrectly mark a take complete. Metadata is now marked complete only after the CSV has closed successfully and motion cleanup has succeeded.

Windows validation passed: **24 Python checks**, **11 Java tests**, the original exercise/contact suite (`.tools/sim-t6ep7cxz/report.json`), and recorder/replayer integration (`.tools/sim-6do5apjd/report.json`). The latter also verifies that external cancellation and connection loss leave an incomplete take. Native GUI clicking confirmed that releasing the checkbox and **Stop & save** each save while both windows remain open, and **Close simulator** ends the session. The button test saved 584 samples in `.tools/recording-stop-button-check`.

The reported green-sphere drop was **not reproduced** in two observed double-click tests; target and robot poses stayed unchanged. Automatic selection removes the need for that gesture, but does not establish the original cause. No hardware was connected: controller communication, actual enabling switches, TCP/load configuration and recording timing still need a lab check.

The preceding revision's [CI run](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42878507535) timed out in the Windows negative replay-start check; its cause is unconfirmed. That check passes locally, and now includes the Java log in timeout failures. [Cross-platform validation of this update](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42882972430), code revision `f1a8530`, passed on Windows, Intel Mac and Apple Silicon Mac.

## Responsive mouse teaching — 10 November 2026

Reproduced an interactive freeze using the unreachable pose from the user's simulator log. Each 20 ms update repeatedly attempted full IK, including its expensive fallback: median **414 ms**, maximum **640 ms**. Mouse following now takes one bounded differential IK step from the actual arm. Commanded PTP/LIN retain their existing full solver and E1 checks; recorder guiding speeds are unchanged.

Windows validation passed:

- The same headless reproduction: median **3.4 ms**, maximum **4.3 ms** per update. Local before/after reports: `.tools/teaching-before.json`, `.tools/teaching-after.json`.
- With the washbasin viewer open and hand-guiding enabled: median **7.5 ms**, 95th percentile below **14 ms**; 10 seconds of physics took 10.24 seconds. The arm followed a reachable target, stayed responsive at the original unreachable pose, and returned to within **0.01 mm** of the starting position. These are scripted marker updates with the real viewer, not automated mouse gestures. Report: `.tools/teaching-viewer-after.json`.
- **24 Python tests** passed, including pose/orientation following, recovery from an unreachable marker, guiding speed limits, and switch release. The regression rejects any call to full IK during mouse following.
- Full exercise/contact integration (`.tools/sim-r5v7r32j/report.json`) and Java recorder/replayer integration (`.tools/sim-i2k2ciyd/report.json`) passed. No hardware was connected.

## Hand-guiding, washbasin and KMP — 6 November 2026

Implemented the small `record.py` / `replay.py` launchers and Sunrise Java mains. They share one CSV/JSON format between hardware and simulation. Hardware uses the original SDK/classpath; Mockito and simulator classes are excluded. No robot was connected, so controller connectivity and real hand-guiding/replay remain untested.

Windows checks passed:

- **24 Python checks**, including PGLfD geometry, and **11 Java tests**. Tests reject incomplete/bad recordings and cover transforms, resampling, quaternion conversion, and hand-guiding enable/release.
- `check_simulation.py`: full original exercise/force/contact regression suite still passes; exercise source files and hardware configuration are unchanged. Local report: `.tools/sim-8ipclhee/report.json`.
- `check_recording.py`: the actual Java recorder waits for guiding enable, records physical movement, ends on switch release, and the same file replays through the actual Java replayer. Final checks also cover refusal of a distant start without `--approach` and successful saving after either Enter or a time limit (`.tools/sim-iwupucvj/report.json`). Cleanup handles the SDK's documented cancellation exception.
- Five scripted simulator takes were recorded in the white washbasin scene. Each had physical contact; these are **synthetic**, not human or real-robot demonstrations. The example data is in `data/examples/sink`. Recorded sampling was roughly 30–40 Hz against a 50 Hz target; actual times and missed deadlines are retained.
- PGLfD KMP and TP-KMP trained on takes 1–4, with take 5 held out. Position RMSE: **9.99 mm / 0.415 mm**; orientation RMSE: **0.224° / 0.011°**. TP-KMP additionally receives the held-out start/end frames, so this is a pipeline demonstration with different conditioning, not a universal comparison of the methods. [Plot and reference data](../data/examples/sink/models).
- The recorded stroke and both learned paths replayed in the basin. Final-position error was below **0.2 mm** for all three in the initial run (`.tools/sim-2xnophzr/sink-report.json`). The reproducible `check_learning.py` subsequently passed retraining plus position/orientation replay checks (`.tools/sim-67ss5prd/learning-report.json`). This measures endpoint tracking in simulation, not error along the whole path or hardware accuracy. The normal command-line TP-KMP replay with `--headless --approach` also completed.

The scene uses an approximate O.novo-sized basin at an 850 mm rim height and a sponge contact box; its render was visually inspected. No pressure regulation or cleaning-success metric is implemented. Replay follows a slow geometric path, without original timing or force reproduction. Training uses aligned manifold statistics as KMP references rather than an extra GMM/GMR stage.

**All three CI jobs passed:** Windows, Intel Mac and Apple Silicon Mac, [run 42279138533](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42279138533), tested code revision `4fceea5`. Each ran setup, 24 Python checks, 11 Java tests, the original exercise/contact suite, recorder/replayer integration, and `check_learning.py` (retrain and replay raw/KMP/TP-KMP). Reports and plots are in the run artifacts. This verifies headless operation across platforms; Mac GUI interaction and hardware remain untested.

## Completed exercise rehearsal — 6 November 2026

Temporarily completed the actual Exercise 2/3/4/8 Java sources and ran their original entry points through the adapter on Windows. Exercises 1, 5a, 5b, 6 and 7 already contain runnable implementations and were exercised directly. This goes beyond launching empty student scaffolds.

- **1 → 2:** taught and saved F0–F2/J0–J2 in an isolated data directory, then used those files for joint/frame PTP, asynchronous blending, LIN, tool-relative LIN and a motion batch. Verified endpoints and a moving blend handoff.
- **3:** restored a taught elbow configuration, changed E1 at the same TCP, then restored the saved E1 with LIN.
- **4:** executed all three requested blended batches (joint PTP, frame PTP, frame LIN). Each produced finite, nonzero translation, rotation and joint-distance logs and reached its final frame.
- **5a/5b:** applied external force during the actual exercise loops; checked physical displacement and return after releasing it. **6:** its 10 N force condition fired at 12 N. **7:** all nine supplied joint motions completed, including singular postures.
- **8:** the temporary solution used two force probes per step to find the wire centre, without reading fixture coordinates. It followed a bounded 280 mm horizontal section through every horizontal bend: 56 steps, 112 physical-contact breaks, maximum lateral centre error **0.89 mm**. This does not cover traversing the fixture's vertical end/support. The initial relative-step solution accumulated drift; using fixed height/progress targets and recentering corrected it. No adapter changes were needed.

All **nine exercise files were restored byte-for-byte**, verified against their pre-edit SHA-256 hashes and Git. The restored sources were recompiled so temporary solved bytecode is no longer active. `uv run --locked scripts/check_java.py` passed afterward. Student TODOs remain available to Ala and Canel; existing teaching files and hardware configuration were untouched.

Local evidence: `.tools/exercise-rehearsal/report-core.json`, `report-wire.json`, trace files, and `original-sha256.json`; process logs in `.tools/sim-ha0806f5` and `.tools/sim-b8bhm7qx`. Temporary solutions stay in that ignored rehearsal directory, outside the active source tree. This rehearsal was local Windows simulation, not a hardware or cross-platform run. Coverage follows the supplied source/comments; the referenced `practical.pdf` remains unavailable.

## E1 and exercise requirements — 6 November 2026

Implemented E1-constrained IK/PTP/LIN, Status handling, partial redundancy persistence, and measured/FK frame metadata. The model's flange now matches Arne's independent kinematics (zero, front, and 30 varied joint configurations). New teaching files use `data/simulation/robosuite-sunrise`; earlier recordings remain untouched.

Local Windows integration passed (`.tools/sim-6h8qba48/report.json`): Exercise 1 → 2 teaching/replay; explicit E1 changes at fixed TCP; taught elbow restoration; FK/IK; invalid E1 rejection; all three Exercise 4 batch types and distance logs; force coordinate conversion and a 10 N threshold firing at 12 N; and all original exercise mains/bounded loops. A separate generic wire scene passed the original Exercise 8 approach and an actual ring/wire contact triggering a Java force break. The scene was rendered and visually inspected. Student TODOs and hardware configuration are unchanged.

All 21 Python checks and nine Java teaching-file tests passed, together with the complete exercise and contact integration suite, on Windows x64, Intel Mac and Apple Silicon Mac: [CI run 42204256716](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42204256716), tested code revision `85184c3`. Regressions include joint-3 constraints, Status branches, continuous LIN, partial metadata defaults, independent flange kinematics, upper-arm external torque, and full/partial/previous-format teaching files.

These checks validate the simulated exercise API. Complete Sunrise planning/singularity behaviour, calibrated dynamics and lab wire dimensions remain unverified. The course README mentions `practical.pdf`, but that handout is absent from the supplied checkout; the coverage audit uses the exercise source and its comments.

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

## Adapter validation — 6 November 2026

Local Windows runs used the real CoppeliaSim/MuJoCo runtime, not a fake bridge:

- Original Exercise 1 ran its front movement and zero-stiffness loop. Three actual poses were taught and saved through the same DataHandler methods as the GUI buttons. Original Exercise 2 loaded F0-F2/J0-J2 and replayed F0 successfully.
- Every supplied exercise main completed, or ran a bounded number of iterations for intentional loops. Exercises 3/4 remain empty templates; Exercise 8 only implements its approach/start sequence.
- Additional physical assertions covered world/tool-relative LIN, Cartesian and joint PTP, TCP/flange offset, motion batches, async completion/timeout, pause/resume/cancel, continuous blended motion, compliant displacement under applied force, force-break condition identity and logged state samples.
- All Exercise 7 singular-posture motions completed. A taught `wireloop_start` supported the original Exercise 8 approach.
- Seven teaching-file regressions passed: F0/J0 starts, numbering gaps, missing/malformed files, units, named redundancy and simulator joint-seed round-trips. Nine existing Python/setup/PGLfD tests also passed.
- Unsupported controller calls fail explicitly; a non-loopback socket attempt was blocked inside the simulator JVM. Original exercise bodies, robot configuration and PGLfD files are unchanged.
- The Java GUI launched without exceptions. Automated desktop clicking could not be checked because the computer-use helper failed to initialize (`failed to write kernel assets`); save methods and name-field updates were checked programmatically.

**Final adapter CI passed on all three platforms:** Windows x64, Intel macOS, and Apple Silicon macOS. [Run 42163084493](https://github.com/alasleimi/robotics-sink-cleaning/actions/runs/42163084493) tested revision `b2b2c97` with installation, seven Java/data regressions, nine Python/setup/PGLfD tests, and full CoppeliaSim integration. It also checked persistent HTTP reuse and continued physics while a connection was idle. Local negative checks confirmed a clear missing-Exercise-2-prerequisite error and that a second exercise cannot terminate the first. Desktop GUI clicking and physical hardware remain unverified; dynamics are not calibrated to the lab.
