# Running the course exercises in simulation

From the repository root:

```sh
uv run --locked scripts/run.py exercise 01
```

This builds the simulator runtime, opens an owned robosuite/MuJoCo viewer, and calls Arne's unchanged exercise main. Close the viewer or a Java control window, or press Ctrl+C in the terminal, to stop the session and its processes. Output is also saved in `.tools/sim-*/exercise.log`.

## Teach first, then replay

1. Start `exercise 01` and let the initial front movement finish.
2. In **Simulation — teach joints / apply force**, click **Read current joints**, change joint angles, then **Move teaching target**. Wait for the arm to settle. Alternatively, click **Follow scene TeachTarget**, which selects the green sphere automatically. **Ctrl + Shift + right-drag** moves it horizontally; **Ctrl + right-drag** moves it vertically; **Ctrl + left-drag** rotates it. Dragging without Ctrl moves the camera. Direct mouse forces on the arm are also available after releasing the teaching target.
3. In Arne's **RobotControl**, press **SAVE_FRAME** and **SAVE_JOINT_POS**. The first names are **F0** and **J0**. Repeat at two other reachable poses to save F1/J1 and F2/J2.
4. Close Exercise 1, then run `uv run --locked scripts/run.py exercise 02`.

Saves contain the actual robot state. Files persist between sessions in `data/simulation/robosuite-sunrise/frames` and `data/simulation/robosuite-sunrise/joint_positions`. Frames retain E1, Status, Turn, and seven joint angles as an IK seed. Returning to a taught frame restores its elbow configuration.

Mouse following takes a small IK step each update, so dragging outside the reachable workspace does not stall physics or input. The arm follows as far as it can; bring the marker back if it cannot reach it. This interactive approximation is separate from the full E1-constrained solver used by PTP/LIN.

Existing files are never renumbered automatically. If old files start at F1/J1, explicitly teach and save F0/J0 using the name fields. Automatic naming continues after the largest existing numeric name. Entering an existing name deliberately replaces that file.

## Exercise coverage

| Exercise | What runs | Prerequisite / scope |
| --- | --- | --- |
| 01 | Front movement and zero-stiffness teaching loop; original save buttons | First session starts with F0/J0 |
| 02 | Original F0 replay; adapter also supports its commented PTP/LIN/relative/async/batch examples | Requires **all six** F0–F2 and J0–J2 files |
| 03 | Taught elbow replay, explicit E1-constrained PTP/LIN, FK/IK with redundancy | API and physical behaviour tested separately; student template stays unchanged |
| 04 | Joint PTP, frame PTP, and LIN batches with Cartesian blending and distance logging | All three batch/log workflows tested; student template stays unchanged |
| 05a | Repeated joint-impedance motions | Apply force using the simulation panel |
| 05b | Repeated Cartesian-impedance motions, including null-space posture term | Gains approximate behaviour, not KUKA controller tuning |
| 06 | Repeated LIN motions and Y-force break condition | Threshold is 10 N in the moving frame's Y direction; explicit reference frames also work |
| 07 | All supplied joint motions through singular configurations | Uses joint motion, including at singular poses |
| 08 | Move up, approach, LIN to taught start, and physical ring/wire contact | Optional `--scene wire`; teach `wireloop_start` first; exploration algorithm remains a student TODO |

Exercises 01, 05a, 05b, and 06 intentionally loop until the session is closed. PAUSE also freezes teaching controls. ABORT cancels the queue and stays paused until RESUME. Completing the adapter does not complete the student exercises.

## Supported subset

- Joint PTP/home, Cartesian PTP, LIN, relative LIN, MotionBatch, move/moveAsync, positionHold.
- Scalar relative joint velocity/acceleration, Cartesian velocity/acceleration, queued PTP/LIN blending.
- Await/timeout, completion/error state, pause/resume, cancel/abort, force and joint-torque break conditions with AND/OR. As in the bundled SDK, `await()` and `validate()` throw `CancelledException` after cancellation and `CommandInvalidException` on invalid motions; cancelled containers are finished but `hasError()` remains false.
- Actual joint/TCP/flange state, measured motor torque, tool-wrench feedback, joint impedance and Cartesian impedance.
- Teaching files, Arne's FileLogger, and the project demonstration recorder/replayer. XYZ translations use mm in Java and metres in the bridge; angles use radians except the degree-formatted `.frm` pose line.
- `HRCMotions.handGuiding()` with joint/Cartesian guiding velocity limits: enable in the recording panel; releasing ends the motion and saves the take, while the viewer stays open. Zero-stiffness dynamics and opposing damping approximate hand-guiding. Custom guiding joint-limit settings are rejected; controller safety services are not simulated.
- E1-constrained IK, Cartesian PTP and LIN; Status selects the PTP configuration. Frames without redundancy retain starting E1/Status. LIN interpolates E1 and retains its starting Status. Turn is stored but ignored for the iiwa's joints, which cannot rotate beyond 180 degrees. See [E1 details and example](../docs/e1.md).

Use `--headless --fast` for automated runs, or `--data PATH` for a separate teaching set. `uv run --locked scripts/check_simulation.py` runs original entry points plus physical assertions using temporary data. It creates and closes its own simulator.

## Limits that matter

This is a custom adapter using genuine SDK motion/geometry objects. It is **not Sunrise.OS running inside a simulator**. Java 17 instrumentation is isolated from the hardware build; the simulator JVM also blocks non-loopback connections.

The scene is a robosuite `CourseEnvironment` with its **iiwa7** model, a fixed base, and the course XML's 202 mm TCP, mass and centre of mass. The hand attachment uses Sunrise's flange convention and matches Arne's independent forward kinematics. The default tool uses a contact sphere and an approximate inertia tensor because the course XML supplies zero inertias. The optional wire scene uses a ring. The `sink` scene adds an approximate white bathroom washbasin and a sponge contact box; see the [demonstration guide](../docs/demonstrations.md). Dynamics and tool/fixture geometry are not calibrated to the lab.

Joint and Cartesian feedback run every 2 ms through a robosuite controller extension. Unlike robosuite's acceleration gains, Sunrise stiffness is interpreted as N/m or Nm/rad. Damping uses the model's joint/task inertia; gravity/bias compensation and null-space projection use robosuite/MuJoCo dynamics. This remains an approximation of KUKA's controller. IK uses MuJoCo Jacobians with bounded numerical solving at joint limits; blended paths retain the custom planner.

Tool forces come from a simulated flange sensor with static tool-weight correction; dynamic residuals differ from KUKA's external-force estimator. Joint external torques include physical contacts and applied forces across all links, so pushing the upper arm can trigger a joint-torque condition. Mouse forces and panel forces act on the physical model.

E1 is enforced as a joint-3 constraint, with joint limits and configuration checks. Infeasible targets and discontinuous LIN solutions fail explicitly. Sunrise's complete path planner and singularity handling have not been reproduced or verified on hardware. Custom moving destination frames, triggers, overlays, splines, and unimplemented controller/safety services still fail explicitly. Real-robot execution uses the original Sunrise entry points and lab procedure.

Force readings default to the measurement frame's orientation; a supplied orientation frame overrides it. Force-condition `tolerance` describes estimation uncertainty, not an addition to the force threshold. The simulator does not model KUKA's uncertainty estimator.

## Wire-loop practice

```sh
uv run --locked scripts/run.py exercise 01 --scene wire
uv run --locked scripts/run.py exercise 08 --scene wire
```

The fixture has a 22 mm radius ring and a 2 mm radius wire, with an open end near XYZ = (405, 200, 350) mm. Teach `wireloop_start` at approximately (400, 200, 350) mm with ABC = (180, 0, 180) degrees. This permits the original approach from above; moving in +X threads the ring onto the wire. The wire bends in Y, producing real MuJoCo contact forces for probing and break conditions. These are generic practice dimensions. The fixture does not supply the student's exploration solution.

## Python experiments

The task in `simulation/adapter/environment.py` is a robosuite `ManipulationEnv` with `reset()` and `step(action)`; actions are seven absolute joint angles in radians. Its reward is zero: this is the exercise sandbox, not a completed sink-cleaning learning task. The Sunrise bridge uses the same model with its physical-unit impedance extension.

## Migrating from CoppeliaSim

Run setup again. CoppeliaSim, ZeroMQ and Lua are no longer required. Old `data/simulation/frames` and `joint_positions` are preserved. The first robosuite port used `data/simulation/robosuite`; it also remains untouched. After correcting the flange convention, new teaching data goes under `data/simulation/robosuite-sunrise`. Re-teach Cartesian frames for the corrected model. F0/J0 naming is retained, and the reader accepts both previous full redundancy metadata and the new format supporting partially specified E1/Status/Turn.
