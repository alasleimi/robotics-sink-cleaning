# Simulation

Run `uv run --locked scripts/run.py sim` from the repository root.

`scenes/iiwa7.ttt` contains the CoppeliaSim 4.10.0 iiwa 7 R800 model, its default surroundings, and MuJoCo selected as the physics engine. The installed application also contains the original model under `models/robots/non-mobile`.

The scene is for developing the adapter. The course tool/TCP, sink, sponge, contact parameters, and control behaviour have not been calibrated or added. Mouse-guided recording and the Sunrise adapter are not implemented yet.

Use metres/radians in simulator code; the course SDK uses millimetres for frame translations. Record actual joint/TCP state when implementing demonstrations, rather than only the dragged target.
