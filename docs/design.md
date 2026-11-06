# Simulator adapter

## Requirement

Keep the KUKA Sunrise API required by the course. Aim to run Arne's exercise bodies with minimal changes, selecting either the physical robot or CoppeliaSim. Arne said no simulator is supplied; this custom adapter is still to be built.

## Current evidence

Exercises create `RoboticsAPIContext` before calling `SunriseConnector`. `ExecutionController` calls Sunrise execution services directly. Swapping only the controller IP or connector is insufficient.

The bundled SDK exposes overridable `Tool.move` and `moveAsync` methods. Intercepting these while retaining SDK geometry/motion objects is a candidate, not a demonstrated solution. Some `LBR` measurement methods are final, so a broad mock subclass is not sufficient.

## First proof

Adapt shared startup and support code so `Task_02_BasicMotions` can send a PTP target to simulation while retaining `tool.move(BasicMotions.ptp(F0))`. Then add LIN, joint-state feedback, and explicit backend selection. Fail clearly on unsupported operations.

Later features need separate validation: async queues/cancellation, blending, elbow/null-space choices, joint/Cartesian impedance, force break conditions, and hand-guiding. Run impedance feedback at the simulator's physics-step rate; Java can send targets and gains. A virtual force sensor is not identical to KUKA's external-force estimate.

## Simulator

CoppeliaSim 4.10.0 rev0 includes the iiwa 7 R800 model. Use its MuJoCo engine and ZeroMQ API. The starter scene is a model/environment baseline, not a calibrated digital copy of the lab setup. The exercise tool/TCP and sponge have not been added.

Sources: [Java/ZeroMQ API](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm), [joint callbacks](https://manual.coppeliarobotics.com/en/jointCallbackFunctions.htm), [IK](https://manual.coppeliarobotics.com/en/kinematics.htm), [force sensors](https://manual.coppeliarobotics.com/en/forceSensors.htm).
