# Lab recordings — 11 November 2026

24 recordings from Ala and Ecem's `iiwa-test` download are preserved byte-for-byte in `data/lab/2026-11-11/raw`. The recording/replay sources and tool configuration are preserved beside them in `source`; they are reference snapshots, not compiled project entry points.

## Watch a recording

From the repository folder, after the normal `uv run --locked scripts/setup.py` setup:

```sh
uv run --locked scripts/play_lab.py 18-40-01 --cleaning-only
```

**Space:** pause/resume. **R:** restart. **Left/right:** scrub one second. The window stays open at the end. Omit `--cleaning-only` to see the approach and entire recording. Replace the time with another filename's time, or pass a full JSON path. `--speed 0.5` plays at half speed. No Gradle build is needed for this viewer. macOS selects `mjpython` automatically; only Windows has been checked for this new workflow.

This viewer places the arm at the recorded joint angles, interpolated using the actual timestamps. It is kinematic playback: gravity, contact forces and controller tracking are not simulated during playback. Task 10 instead downsampled the path and sent blended PTP moves, ignoring the recorded timing. The two playback speeds should not be expected to match.

## Classification

| Cleaning class | Recordings | Interpretation |
|---|---:|---|
| `rim_inner_y_minus` | 4 | Inner observed arc band, +Y toward −Y |
| `rim_inner_y_plus` | 3 | Same band, reverse direction |
| `rim_outer_y_minus` | 3 | Outer observed arc band, +Y toward −Y |
| `rim_outer_y_plus` | 2 | Same band, reverse direction |
| `bowl_loop` | 2 | Counterclockwise loop with inward finish |

**The half-arcs are intentional edge cleaning**, confirmed by Ala. Inner/outer band labels and bowl-loop labels are geometric interpretations. The two directions are learned separately, without reversing the recorded orientation or motion.

Ten files are kept out of these five cleaning models: one essentially stationary take; three short/irregular early tests; three early large motions inconsistent with the later sink placement; one wrist-rotation test; and two partial/local edge motions. Early setup/intent is unconfirmed, so those files are not declared corrupt or deleted. The wrist test reaches A7 ≈178.35°, exceeding this model's nominal 175° limit; inspection preserves its measurements without clipping.

No malformed JSON, nonfinite required joint samples, or non-increasing timestamps were found. Actual median sample spacing is approximately 19–25 ms, not exactly the requested 15/20 ms. The maximum observed gap is 53 ms. All 24 start with an approach or test posture; only the selected cleaning intervals enter training.

[Trajectory overview](../data/lab/2026-11-11/analysis/trajectories.png) · [Per-file decisions and intervals](../data/lab/2026-11-11/analysis/recordings.json) · [Editable classification](../data/lab/2026-11-11/classification.json)

## Sink placement and robot geometry

The simulator uses the supplied **NAURU 575 × 425 × 170 mm STEP**, not the earlier generic washbasin. Its original bottom-centre origin is approximately **(0.712, −0.042, −0.009) m** in robot-base coordinates, with **89.67° yaw**. The inferred rim height is **0.161 m above the robot base**. Positive STEP Y faces the robot; the tap-hole side faces away.

Placement fits the central parts of the two observed arc bands to the CAD inner/outer front rim. Median horizontal residual is about **12 mm**, with 90% below **32 mm**. This is an estimate, not calibration: exact sponge contact offset, compression, and setup changes remain unknown. The floor/pedestal use an illustrative 850 mm installation height; their lab heights were not measured.

The last six files include Task 11 TCP measurements. Reconstructed TCP agrees with these at median **0.05–0.10 mm**; each file's 95th percentile is below **0.38 mm** (maximum individual discrepancy about 1.8 mm). This supports the joint axes, robot-base convention, 202 mm TCP and degree/radian conversion. It does not validate physical contact, the sponge TCP, or robot control.

The mesh retains the STEP's appearance. Collision geometry is a roughly 10 mm upper-surface height field that keeps the bowl open; undercuts and the overflow tunnel are omitted. The course sponge/load remain approximate; the lab source used a 0.25 kg tool template versus the repository's original course load. Kinematic playback is unaffected by that mass difference.

## Learned paths

```sh
uv run --locked scripts/play_lab.py data/lab/2026-11-11/models/rim_inner_y_minus/kmp
uv run --locked scripts/play_lab.py data/lab/2026-11-11/models/rim_inner_y_minus/tpkmp
```

Replace the motion directory with any class from the table. The saved paths use simulated IK; their joints and timing are not recorded robot measurements. The existing Java Cartesian replay can also read the generated folders (`scripts/replay.py ... --scene nauru --approach`), but commands motion through the approximate controller and is a different check from this viewer. No new hardware motion was run.

Both models use the **unmodified PGLfD library**, learning position and orientation on R3 × SO(3). Cleaning intervals are aligned by smoothed TCP arc length, so operator pauses and speed are not learned. Ordinary KMP uses a fixed reference orientation to avoid the SO(3) 180° logarithm boundary. TP-KMP combines models in two task frames: origins at the stroke's start/end positions, with axes fixed relative to the sink. Tool roll is learned rather than being used to rotate the task geometry.

Final models use all accepted recordings. Evaluation holds out a whole recording at a time, separately for each class. TP-KMP receives the held-out start/end positions; KMP does not. The shared estimated sink placement uses this session's data, and this is exploratory evaluation, not an independent test of transfer to another setup.

| Class | KMP position RMSE | TP-KMP position RMSE |
|---|---:|---:|
| Inner edge, −Y | 33 mm | 56 mm |
| Inner edge, +Y | 60 mm | 26 mm |
| Outer edge, −Y | 39 mm | 64 mm |
| Outer edge, +Y | 61 mm | 52 mm |
| Bowl loop | 47 mm | 41 mm |

These are means of held-out-fold RMSE, **not** final-path fit errors or contact accuracy. Two-example classes have only one training example per fold. TP-KMP is not consistently better. More demonstrations with consistent start/end positions and an actual sink/TCP calibration are the next useful improvement.

Each model directory contains `report.json`, `paths.png`, saved NPZ parameters and both replayable predictions. Models learn pose only; they do not learn contact forces. Task 11 raw force/torque channels remain in the JSON files, including guiding-hand loads and their original unspecified reference frame.

## Reproduce or change labels

```sh
uv run --locked scripts/prepare_lab.py
uv run --locked scripts/train_lab.py --output data/recordings/lab-models-new
```

`prepare_lab.py` regenerates imported CSVs under `data/recordings/lab-2026-11-11`, reports and estimated scene placement. Edit `classification.json` to change labels or explicit `interval_s` ranges; raw JSONs remain untouched. Rim intervals default to the smoothed Y extrema. Metadata marks absent wrench/read-duration values as unavailable; raw Task 11 telemetry is never invented or discarded.

Saved parameters can be used without retraining:

```sh
uv run --locked scripts/predict_lab.py data/lab/2026-11-11/models/rim_inner_y_minus data/recordings/new-prediction
```

Add `--frames frames.json` to supply TP-KMP task frames. The JSON has `start` and `end`, each `[x,y,z,qx,qy,qz,qw]` **in sink coordinates**; copy the model report's `task_frames_in_sink` as a starting point. Changing frames does not guarantee contact or reachability. The viewer checks IK before showing a learned robot path.

CAD conversion is optional; the meshes are committed and ordinary playback installs no CAD software. To regenerate them:

```sh
uv run scripts/convert_nauru.py simulation/assets/nauru/NAURU_575x425_v4_photo_constrained.step
```

The script's isolated dependency declaration installs CadQuery, following its [STEP importer](https://cadquery.readthedocs.io/en/latest/importexport.html). Collision data follows MuJoCo's [height-field format](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#asset-hfield).
