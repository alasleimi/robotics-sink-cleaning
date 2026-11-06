# Running the course exercises in simulation

From the repository root:

```sh
uv run --locked scripts/run.py exercise 01
```

This builds the simulator runtime, opens an owned robosuite/MuJoCo viewer, and calls Arne's unchanged exercise main. Close the viewer or a Java control window, or press Ctrl+C in the terminal, to stop the session and its processes. Output is also saved in `.tools/sim-*/exercise.log`.

## Teach first, then replay

1. Start `exercise 01` and let the initial front movement finish.
2. In **Simulation — teach joints / apply force**, click **Read current joints**, change joint angles, then **Move teaching target**. Wait for the arm to settle. Alternatively, click **Follow scene TeachTarget** and select the green sphere in the MuJoCo viewer (double-click), then Ctrl + right-drag to translate it or Ctrl + left-drag to rotate it. Direct mouse forces on the arm are also available after releasing the teaching target.
3. In Arne's **RobotControl**, press **SAVE_FRAME** and **SAVE_JOINT_POS**. The first names are **F0** and **J0**. Repeat at two other reachable poses to save F1/J1 and F2/J2.
4. Close Exercise 1, then run `uv run --locked scripts/run.py exercise 02`.

Saves contain the actual robot state, not the desired teaching target. Files persist between sessions in `data/simulation/robosuite/frames` and `data/simulation/robosuite/joint_positions`. Simulator frames also retain seven joint angles as an IK seed, so returning to a saved pose can recover its elbow posture.

Existing files are never renumbered automatically. If old files start at F1/J1, explicitly teach and save F0/J0 using the name fields. Automatic naming continues after the largest existing numeric name. Entering an existing name deliberately replaces that file.

## Exercise coverage

| Exercise | What runs | Prerequisite / scope |
| --- | --- | --- |
| 01 | Front movement and zero-stiffness teaching loop; original save buttons | First session starts with F0/J0 |
| 02 | Original F0 replay; adapter also supports its commented PTP/LIN/relative/async/batch examples | Requires **all six** F0–F2 and J0–J2 files |
| 03 | Original main and saved-frame/joint APIs | Elbow experiment is a student TODO; use saved simulator joint seeds or explicit joints |
| 04 | Original main and joint/Cartesian motion APIs | Experiment is a student TODO |
| 05a | Repeated joint-impedance motions | Apply force using the simulation panel |
| 05b | Repeated Cartesian-impedance motions, including null-space posture term | Gains approximate behaviour, not KUKA controller tuning |
| 06 | Repeated LIN motions and Y-force break condition | Apply more than 10 N in world Y; clear force afterward |
| 07 | All supplied joint motions through singular configurations | Uses joint motion, including at singular poses |
| 08 | Move up, approach, and LIN to taught start | Teach a frame named `wireloop_start` in 01 first; wire-following remains a TODO |

Exercises 01, 05a, 05b, and 06 intentionally loop until the session is closed. PAUSE also freezes teaching controls. ABORT cancels the queue and stays paused until RESUME. Completing the adapter does not complete the student exercises.

## Supported subset

- Joint PTP/home, Cartesian PTP, LIN, relative LIN, MotionBatch, move/moveAsync, positionHold.
- Scalar relative joint velocity/acceleration, Cartesian velocity/acceleration, queued PTP/LIN blending.
- Await/timeout, completion/error state, pause/resume, cancel/abort, force and joint-torque break conditions with AND/OR.
- Actual joint/TCP/flange state, measured motor torque, tool-wrench feedback, joint impedance and Cartesian impedance.
- Teaching files and Arne's FileLogger. XYZ translations use mm in Java and metres in the bridge; angles use radians except the degree-formatted `.frm` pose line.

Use `--headless --fast` for automated runs, or `--data PATH` for a separate teaching set. `uv run --locked scripts/check_simulation.py` runs original entry points plus physical assertions using temporary data. It creates and closes its own simulator.

## Limits that matter

This is a custom adapter using genuine SDK motion/geometry objects. It is **not Sunrise.OS running inside a simulator**. Java 17 instrumentation is isolated from the hardware build; the simulator JVM also blocks non-loopback connections.

The scene is a robosuite `CourseEnvironment` with its **iiwa7** model, a fixed base, and the course XML's 202 mm TCP, mass and centre of mass. The tool uses a sphere for contact and an approximate inertia tensor because the course XML supplies zero inertias. The sink, sponge and wire course are not modeled. Link/tool dynamics and contact are not calibrated to the lab.

Joint and Cartesian feedback run every 2 ms through a robosuite controller extension. Unlike robosuite's acceleration gains, Sunrise stiffness is interpreted as N/m or Nm/rad. Damping uses the model's joint/task inertia; gravity/bias compensation and null-space projection use robosuite/MuJoCo dynamics. This remains an approximation of KUKA's controller. IK uses MuJoCo Jacobians with bounded numerical solving at joint limits; blended paths retain the custom planner.

Tool forces come from a simulated flange sensor with static tool-weight correction; dynamic residuals and contact elsewhere on the arm differ from KUKA's external-force estimator. Joint external torques are estimated from that tool wrench. Mouse forces and panel forces act on the physical model.

Saved simulator joint seeds preserve a demonstrated IK branch. Exact KUKA E1/status/turn constraints, custom moving destination frames, triggers, overlays, splines, and unimplemented controller/safety services fail explicitly. Do not interpret successful simulation as validation for physical motion. Real-robot execution still uses the original Sunrise entry points and lab procedure.

## Python experiments

The task in `simulation/adapter/environment.py` is a robosuite `ManipulationEnv` with `reset()` and `step(action)`; actions are seven absolute joint angles in radians. Its reward is zero: this is the exercise sandbox, not a completed sink-cleaning learning task. The Sunrise bridge uses the same model with its physical-unit impedance extension.

## Migrating from CoppeliaSim

Run setup again. CoppeliaSim, ZeroMQ and Lua are no longer required by the repository. The old local application may remain installed. Old `data/simulation/frames` and `joint_positions` are preserved; new teaching data goes under `data/simulation/robosuite` to avoid silently replaying poses from a different model. F0/J0 naming and the original file format are retained.
