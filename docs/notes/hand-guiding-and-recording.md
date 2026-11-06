# Hand-guiding and recording demonstrations

Status: source and documentation inspected; no robot connection, code execution, or demonstration recorded.

## What we want

Move the arm by hand while its powered controller supports its weight, and record the motion over time. This is called hand-guiding or kinesthetic teaching.

## Two mechanisms found

1. **KUKA's dedicated hand-guiding motion.** The bundled KUKA hand-guiding 1.16.0.9 API documentation exposes `HRCMotions.handGuiding()`, returning a `HandGuidingMotion`. It describes guiding while the configured hand-guiding device switch is pressed; releasing it ends the motion. The lab's enabling device and safety configuration have not been checked. Compatibility with the exact installed Sunrise OS version has not been tested.
2. **The course's gravity-compensation approximation.** `SunriseConnector.gravCompImpedance()` uses joint impedance control with zero joint stiffness and nonzero damping, held through `positionHold`. The GUI's `GRAV_COMP` button calls it. A separate `gravCompImpedanceLoop()` uses repeated joint motions and is called by `Task_01_TeachIn`.

These are distinct implementations. Do not assume the custom impedance helper includes the dedicated hand-guiding motion's behavior or safeguards.

## Recording

- The teaching GUI can save individual tool poses and joint configurations.
- The repo also contains `FileLogger`, which can continuously record timestamps, joint angles, tool position and orientation, and estimated external forces/torques.
- `Task_01_TeachIn` does not currently start that logger. A demonstration recorder would need to combine the selected guiding method with logging and a deliberate start/stop lifecycle.
- Record the actual tool frame and the sink-to-robot transform so demonstrations can later be expressed relative to the sink.
- Units need attention: `FileLogger` comments describe orientation as degrees, but its implementation writes `getAlphaRad()`, `getBetaRad()`, and `getGammaRad()`. Those values are radians. The point-saving method in `DataHandler` converts orientation to degrees. Do not mix the two formats.

## Check before using the example on the robot

- `Task_01_TeachIn` first calls `goFront()`, which commands a preset posture. It is not a start-guiding-in-place application.
- Both custom gravity-compensation helpers are explicitly documented as having no joint-limit avoidance. This describes the helper, not the absence of controller-level safety monitoring.
- The connector loads `EXERCISE_TOOL`. Its tool geometry, mass, and center of mass must match the actual attached equipment; the sponge setup has not been checked.
- The course README requires the lab safety introduction and T1 operation; changes of operating mode are to be discussed with Max or Arne.

Next practical question for Arne: Which guiding method is approved for demonstrations on our current robot and tool setup: built-in hand-guiding or the course's gravity-compensation helper?

## Sources in the project

- `src/main/java/exercises/Task_01_TeachIn.java`, lines 42-46.
- `src/main/java/robot/SunriseConnector.java`, lines 147-240 and 295-298.
- `src/main/java/robot/ControlGUI.java`, lines 58-89.
- `src/main/java/utility/FileLogger.java`, fields at lines 23-73, logging entry points at 115-149, and orientation output at 314-318.
- `src/main/java/utility/DataHandler.java`, lines 57-72.
- `lib/KUKAJavaLib/com.kuka.roboticsAPI.handGuiding_1.16.0.9-javadoc.jar`: `HRCMotions.html` and `HandGuidingMotion.html`.
- `docs/arne-exercises.md`, safety section.
