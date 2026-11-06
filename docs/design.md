# Simulator architecture

Original Java exercises run through the separate `simulation/java` runtime using genuine KUKA motion/geometry objects. Java 17 instrumentation intercepts hardware calls; the simulator JVM blocks non-loopback connections. The original hardware source set and course configuration remain separate.

An authenticated loopback HTTP bridge queues requests to one physics thread. That thread owns robosuite's `CourseEnvironment`, the MuJoCo model/data, trajectory queue and optional native viewer. Closing the viewer closes the session. macOS GUI sessions use `mjpython`; headless tests use normal Python.

`CourseEnvironment` composes robosuite's iiwa7, NullMount and a custom course tool from `RoboticsAPI.data.xml`. The Python task can also be used through robosuite's reset/step interface. The Java bridge supplies PTP/LIN/relative trajectories, queues, approximate blending, pause/cancel and force break conditions.

The controller extends robosuite's joint-position controller state/dynamics interface. Its physical-unit springs preserve the meaning of Sunrise stiffness (N/m, Nm/rad), unlike directly copying those numbers into acceleration-based robosuite gains. Damping uses model inertia; Cartesian null-space projection uses robosuite's operational-space matrices. Feedback runs at 500 Hz. Flange force/torque sensors include static tool-weight compensation. This is not KUKA's proprietary controller.

Numerical IK uses MuJoCo Jacobians and a bounded least-squares fallback at joint limits. Saved joint seeds retain taught posture; exact KUKA E1/status/turn constraints still fail explicitly. Pose units at the Java boundary remain mm/radians; `.frm` files store XYZ mm/ABC degrees and `.jnt` files store radians.

Teaching data defaults to `data/simulation/robosuite`; hardware still uses `data`. Existing CoppeliaSim data is preserved separately. F0/J0 numbering, persistence and validation are unchanged.

See [exercise coverage and limitations](../simulation/README.md) and [validation](validation.md). Implementation references: [robosuite controllers](https://robosuite.ai/docs/modules/controllers.html), [MuJoCo Python/viewer](https://mujoco.readthedocs.io/en/3.3.7/python.html), [robosuite source](https://github.com/ARISE-Initiative/robosuite/tree/v1.5.2).
