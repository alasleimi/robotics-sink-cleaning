# Ala and Ecem — 11 November 2026

- `raw/`: 24 original JSON files, unchanged from the supplied Downloads folder.
- `source/`: original Task 9/10/11, connector and tool XML snapshots, not compiled by Gradle.
- `classification.json`: reviewable decisions; the half-arcs are confirmed edge cleaning.
- `analysis/`: per-file checks, retained intervals, sink estimate and plots.
- `analysis/audit/`: independent band clustering, temporal split fits, 3D CAD distances, approach flags and contact-offset sensitivity; see [audit conclusions](../../../docs/lab-audit.md).
- `models/`: PGLfD KMP and TP-KMP parameters/predictions for five cleaning classes.

The seven early recordings (17:37–17:51) are confirmed tests on a different robot, excluded from sink fitting and learning. Their robot/tool are unknown; Cartesian reconstructions using the sink robot are illustrative only.

Start: `uv run --locked scripts/play_lab.py 18-40-01 --cleaning-only`

See [instructions and limitations](../../../docs/lab-recordings.md). No file was deleted from Downloads. JSON filename times are generated when the recording is saved, not when it starts. Samples use their actual elapsed `t`; `sampleMs` is only the requested sleep interval.

Task 9 records joints using the course zero-stiffness helper. Task 11 adds measured TCP/torque/wrench channels. Task 10 runs downsampled, blended PTP motions and ignores timing. These differ from the repository's separate native-hand-guiding recorder.
