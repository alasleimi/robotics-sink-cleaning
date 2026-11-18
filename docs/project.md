# Project state

## Goal

Locate the sink using markers, record demonstrations, learn a cleaning baseline, and evaluate it. Adding sink geometry is the optional extension. Original discussion: [28 October meeting](notes/2026-10-28-meeting.md).

## Next tasks

| Task | State | Next action |
| --- | --- | --- |
| Shared repository and repeatable setup | Implemented; validation recorded below | Canel clones after being invited |
| Sunrise simulation adapter | E1, taught redundancy, force fixes and wire fixture implemented; full Windows/Intel Mac/Apple Silicon CI passed | Re-teach in the corrected flange convention; see `simulation/README.md` |
| Robot/API exercises | Completed temporary solutions tested on Windows, including force-guided wire traversal; all templates restored byte-for-byte | Teach F0-F2/J0-J2 in 01, then work through the restored exercises; see `validation.md` |
| AprilTag sink localisation | Unstarted | Detect a printed marker and define transforms |
| Demonstration recording and replay | Ala/Ecem supplied 24 real Task 9/11 recordings from 11 November; Task 10 hardware replay reported. New simulator viewer preserves measured joints/timestamps | Review the [classified lab recordings](lab-recordings.md); separate native hand-guiding recorder remains hardware-untested |
| KMP and TP-KMP baseline | Both trained with PGLfD for five motion classes from 14 real recordings; all ten final paths pass simulated IK | Review 3–6 cm mean held-out errors; collect more consistent takes and measure sink/TCP calibration |
| PushWiper/adaptive wiping | Reading pending | Summarise their inputs, outputs, and feedback |
| Sink reconstruction | NAURU STEP placement passes retrospective temporal-split/bowl checks at centimetre scale; exact calibration unresolved | Measure three rim points and sponge contact offset; see [audit](lab-audit.md) |

The [18 November data audit](lab-audit.md) supports all 12 edge-band/direction labels and the two lower bowl loops. Some selected intervals include approach; `18-40-38` is an outer-band outlier. Ala is unsure whether the later setup changed. Existing learning outputs remain the baseline, not validated contact trajectories.

Owners and dates have not been assigned. Keep ideas with their evidence, current conclusion, and next action here or in a linked short note.

## Robot configuration

The smartPAD photo confirms iiwa **7 R800**, Sunrise.OS **1.16.1.9**, station IP **172.31.1.147**. [Photo](notes/images/smartpad-station-information.jpg).

The exercises use the same address and attach `EXERCISE_TOOL` with TCP Z = 202 mm. The provided mass/centre-of-mass describe that tool; a sponge attachment needs its own check.

`StationSetup.cat` declares a **14 R820** topology despite the actual 7 R800. Whether this is an unused historical template is unresolved. The connector discovers the robot dynamically. No hardware configuration was changed.

## Validation

See [setup validation](validation.md) for what has actually been checked. Compiling and creating SDK objects do not validate controller connectivity or real motion.
