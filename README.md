# Sink cleaning with the KUKA iiwa

Shared project for Ala and Canel: complete the course exercises, record demonstrations, and learn sink-cleaning movements.

**Robot:** LBR iiwa 7 R800, Sunrise.OS 1.16.1.9. **Simulator:** robosuite 1.5.2 with MuJoCo 3.3.7 and its iiwa7 model.

## Setup

Install [Git](https://git-scm.com/downloads) and [uv](https://docs.astral.sh/uv/getting-started/installation/) once, then:

```sh
git clone https://github.com/alasleimi/robotics-sink-cleaning.git
cd robotics-sink-cleaning
uv run --locked scripts/setup.py
```

The same command installs robosuite, MuJoCo, Python dependencies and a local Java 17 JDK; the committed Gradle Wrapper supplies Gradle 8.14.3. Python packages and hashes are locked in `uv.lock`; JDK downloads are pinned in `tool-versions.json`. Running setup again reuses them. No separate simulator download, administrator rights or global PATH changes are needed.

Supported targets: **Windows x64, Intel Mac and Apple Silicon Mac**. The launcher selects `mjpython` automatically for the Mac viewer. NumPy 1.26.4 and Numba 0.61.2 are pinned for robosuite compatibility and Intel Mac binary packages; PGLfD stays in the same environment.

## Daily commands

```sh
uv run --locked scripts/run.py sim      # Open the robosuite iiwa7 viewer
uv run --locked scripts/run.py exercise 01  # Run original Exercise 1 in simulation
uv run --locked scripts/run.py exercise 02  # Replay poses saved in Exercise 1
uv run --locked scripts/run.py build    # Compile exercises and check SDK objects locally
uv run --locked scripts/doctor.py --sim # Run an isolated physics check
uv run --locked pytest -q               # Check the setup helpers
```

`build` does not connect to the robot. In IntelliJ, open this folder as a Gradle project and select the JDK path recorded in `.tools/local.json`. Original exercise entry points still connect to the real robot; follow [Arne's lab instructions](docs/arne-exercises.md).

**Use `exercise 01` to start teaching.** It opens its own simulator and runs Arne's original Java entry point through the adapter. Save three frames (`F0`–`F2`) and joint positions (`J0`–`J2`) before Exercise 2. Simulation files live in `data/simulation/robosuite-sunrise`; the original hardware entry points keep using `data`.

See the short [exercise guide](simulation/README.md) for teaching controls, prerequisites, supported API calls, and simulator limits. The adapter approximates motion planning and impedance; it does not reproduce the Sunrise controller. Exercises 3/4 and the wire-following part of 8 remain student TODOs.

E1-constrained IK/PTP/LIN and taught elbow replay are supported. The flange convention matches Arne's supplied kinematics. Existing CoppeliaSim and first-port robosuite recordings remain in their previous directories; re-teach Cartesian poses after this correction. Add `--scene wire` for Exercise 8's generic contact practice fixture.

## Record and learn demonstrations

```sh
uv run --locked scripts/record.py data/recordings/take-01
uv run --locked scripts/replay.py data/recordings/take-01 --approach
uv run --locked scripts/train_kmp.py data/examples/sink/demos data/recordings/my-models
```

Recording opens the white bathroom washbasin scene and a small recording panel. Enable hand-guiding; the green target is selected automatically. Drag it, then release the checkbox, click **Stop & save**, or press Enter to save. The panel shows **Saved** and keeps the scene open until you close it. Both commands default to simulation; add `--robot` to use Sunrise on the actual arm. `--approach` allows a direct slow move to the first pose; it does not plan around the basin.

The [short demonstration guide](docs/demonstrations.md) covers hardware use, controls, sink calibration, replay and KMP/TP-KMP. Five scripted simulator takes and their learned paths are included in `data/examples/sink`. Hardware recording/replay has not been tested in the lab.

## Where things belong

| Folder | Contents |
| --- | --- |
| `src/main/java/exercises`, `robot`, `utility` | Arne's exercises and support code |
| `src/main/java/sinkcleaning` | Our Java project code and local setup check |
| `learning/sinkcleaning` | Our Python learning code |
| `third_party/pglfd` | Unmodified Moodle snapshot; import as `pglfd` |
| `simulation` | Robosuite task, physics adapter, Java bridge and run guide |
| `docs` | Project notes, tasks, robot configuration facts |
| `data` | Data format notes and small examples |
| `.tools`, `.venv` | Local installations; ignored by Git |

Keep large demonstration recordings in `data/recordings/` (ignored) until we choose shared storage. Personal planning is outside this repository.

## Working together

Both teammates clone this private repo. Use short task branches, then merge working changes into `main`. Canel still needs a repository invitation once her GitHub username is known.

Arne's history is retained. To fetch his future changes, add the upstream remote on a new clone:

```sh
git remote add upstream https://gitlab.lrz.de/arne/iiwa-exercises.git
git fetch upstream
```

See [source provenance](docs/sources.md). The course's bundled KUKA SDK stays under its original terms; this project does not grant a new licence for it.
