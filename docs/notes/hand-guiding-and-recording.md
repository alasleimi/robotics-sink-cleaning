# Hand-guiding and recording

Implemented on 6 November 2026. See the [record/replay guide](../demonstrations.md) for commands, calibration and saved format, and [validation](../validation.md) for measured results. Real hardware remains untested.

The recorder uses **KUKA's dedicated `HRCMotions.handGuiding()`**, as requested. The bundled API documents guidance while the configured device switch is pressed; releasing it ends the motion. It is distinct from the course's zero-stiffness `gravCompImpedance()` / `gravCompImpedanceLoop()` helpers.

The simulator implements the recorder's API subset with an enable checkbox, compliant motion and approximate opposing damping above guiding velocity limits. It does not reproduce KUKA safety monitoring. The lab guiding device/configuration and sponge TCP/load still need checking.

The earlier proposal included separate Start/Keep/Discard controls, a writer thread and GMM/GMR preprocessing. The implemented version is smaller: one command per take, Enter/release to finish, one buffered writer, and direct aligned manifold statistics for KMP references. Completed raw takes stay unchanged; remove unwanted directories explicitly.

Sources bundled in the repository:

- `lib/KUKAJavaLib/com.kuka.roboticsAPI.handGuiding_1.16.0.9-javadoc.jar`: `HRCMotions` and `HandGuidingMotion`.
- `src/main/java/robot/SunriseConnector.java`: original gravity-compensation helpers and tool setup.
- `src/main/java/utility/FileLogger.java`: existing telemetry example. Its orientation getters return radians despite older comments saying degrees.
- `docs/arne-exercises.md`: lab operation requirements.
