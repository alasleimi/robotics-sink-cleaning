# Project state

## Goal

Locate the sink using markers, record demonstrations, learn a cleaning baseline, and evaluate it. Adding sink geometry is the optional extension. Original discussion: [28 October meeting](notes/2026-10-28-meeting.md).

## Next tasks

| Task | State | Next action |
| --- | --- | --- |
| Shared repository and repeatable setup | Implemented; validation recorded below | Canel clones after being invited |
| Sunrise simulation adapter | E1, taught redundancy, force-frame/threshold fixes and optional wire fixture implemented | Validate current revision; see `docs/validation.md` |
| Robot/API exercises | Original mains tested in simulation; student TODOs preserved | Teach F0-F2/J0-J2 in 01, then run 02; hardware execution remains untested |
| AprilTag sink localisation | Unstarted | Detect a printed marker and define transforms |
| Demonstration recording | Simulator teaching saves and FileLogger tested | Agree on lab recording fields and setup |
| PushWiper/adaptive wiping | Reading pending | Summarise their inputs, outputs, and feedback |
| Sink reconstruction | Unstarted | Photograph sink and assess geometry capture |

Owners and dates have not been assigned. Keep ideas with their evidence, current conclusion, and next action here or in a linked short note.

## Robot configuration

The smartPAD photo confirms iiwa **7 R800**, Sunrise.OS **1.16.1.9**, station IP **172.31.1.147**. [Photo](notes/images/smartpad-station-information.jpg).

The exercises use the same address and attach `EXERCISE_TOOL` with TCP Z = 202 mm. The provided mass/centre-of-mass describe that tool; a sponge attachment needs its own check.

`StationSetup.cat` declares a **14 R820** topology despite the actual 7 R800. Whether this is an unused historical template is unresolved. The connector discovers the robot dynamically. No hardware configuration was changed.

## Validation

See [setup validation](validation.md) for what has actually been checked. Compiling and creating SDK objects do not validate controller connectivity or real motion.
