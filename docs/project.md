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
| Demonstration recording and replay | Sunrise hand-guiding recorder and path replay implemented; simulator integration tested | Check actual sponge TCP/load and guiding setup, then record a short lab take; see `demonstrations.md` |
| KMP and TP-KMP baseline | PGLfD pipelines trained on five scripted simulator takes | Replace synthetic takes with human demonstrations and evaluate on new strokes |
| PushWiper/adaptive wiping | Reading pending | Summarise their inputs, outputs, and feedback |
| Sink reconstruction | Unstarted | Photograph sink and assess geometry capture |

Owners and dates have not been assigned. Keep ideas with their evidence, current conclusion, and next action here or in a linked short note.

## Robot configuration

The smartPAD photo confirms iiwa **7 R800**, Sunrise.OS **1.16.1.9**, station IP **172.31.1.147**. [Photo](notes/images/smartpad-station-information.jpg).

The exercises use the same address and attach `EXERCISE_TOOL` with TCP Z = 202 mm. The provided mass/centre-of-mass describe that tool; a sponge attachment needs its own check.

`StationSetup.cat` declares a **14 R820** topology despite the actual 7 R800. Whether this is an unused historical template is unresolved. The connector discovers the robot dynamically. No hardware configuration was changed.

## Validation

See [setup validation](validation.md) for what has actually been checked. Compiling and creating SDK objects do not validate controller connectivity or real motion.
