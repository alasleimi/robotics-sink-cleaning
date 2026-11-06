# Adapter implementation

The hardware `main` source set still uses the course SDK. A separate `simulator` source set adds the launcher, Mockito instrumentation and Gson; these dependencies are absent from the hardware runtime.

`simproxy.Launcher` calls the original exercise main inside an intercepted `RoboticsAPIContext`. SDK Frame, JointPosition, Tool geometry, conditions and motion objects remain genuine SDK objects. Controller/device calls and Tool motion dispatch are redirected to an authenticated loopback HTTP bridge. Unsupported calls throw rather than returning dummy success or measurements. A Java 17 network guard also blocks non-loopback connections from this JVM.

The Python bridge owns the ZeroMQ client and simulation clock on one thread. It serializes queued motions, performs IK and path interpolation, and sends setpoints to Lua callbacks. The Lua joint callback runs feedback at MuJoCo's physics step rate, including joint/Cartesian stiffness, damping, null-space posture and torque saturation. Sensor readings and saved demonstrations come from the actual physics state.

Async blending overlaps adjacent trajectories with a smooth crossfade; a successor must already be queued. This approximates Sunrise blending. Position-control endpoints settle to a joint tolerance, whereas impedance motions can finish while displaced by an external load. Pause holds the current joint posture and freezes path time; cancellation stops the affected motion.

`DataHandler` starts empty teaching directories at F0/J0, validates names and values, writes atomically and reports missing prerequisites clearly. The default hardware data path remains `data`; the simulation launcher selects `data/simulation`. Named KUKA redundancy metadata round-trips in teaching files, but exact KUKA E1/status/turn-constrained planning is not emulated. Simulator frames instead store an explicit joint seed.

See [usage, exercise coverage and limits](../simulation/README.md) and [validation](validation.md).

Implementation references: [ZeroMQ API](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm), [joint callbacks](https://manual.coppeliarobotics.com/en/jointCallbackFunctions.htm), [simIK](https://manual.coppeliarobotics.com/en/simIK.htm), [force sensors](https://manual.coppeliarobotics.com/en/forceSensors.htm). Installed 4.10.0 manuals were used to match the pinned runtime.
