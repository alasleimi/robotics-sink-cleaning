# Sink cleaning with the KUKA iiwa

Shared project for Ala and Canel: complete the course exercises, record demonstrations, and learn sink-cleaning movements.

**Robot:** LBR iiwa 7 R800, Sunrise.OS 1.16.1.9. **Simulator:** CoppeliaSim Edu 4.10.0 rev0, with MuJoCo selected in the starter scene.

## Setup

Install [Git](https://git-scm.com/downloads) and [uv](https://docs.astral.sh/uv/getting-started/installation/) once, then:

```sh
git clone https://github.com/alasleimi/robotics-sink-cleaning.git
cd robotics-sink-cleaning
uv run --locked scripts/setup.py
```

The same command installs Python dependencies, a local Java 17 JDK, and CoppeliaSim; the committed Gradle Wrapper supplies Gradle 8.14.3. Downloads are pinned by SHA-256 in `tool-versions.json`. Running setup again reuses them. Nothing needs administrator rights or global Java/PATH changes.

Supported packages: **Windows x64**, **Intel Mac with macOS 13+**, and **Apple Silicon Mac with macOS 15+**. On macOS, first launch may require the normal system approval for a downloaded application. CoppeliaSim Edu is for educational use by students and teachers at schools/universities, not commercial use; see [the publisher's terms](https://www.coppeliarobotics.com/#download).

## Daily commands

```sh
uv run --locked scripts/run.py sim      # Open the iiwa starter scene
uv run --locked scripts/run.py build    # Compile exercises and check SDK objects locally
uv run --locked scripts/doctor.py --sim # Check a running simulator connection
uv run --locked pytest -q               # Check the setup helpers
```

`build` does not connect to the robot. In IntelliJ, open this folder as a Gradle project and select the JDK path recorded in `.tools/local.json`. Original exercise entry points still connect to the real robot; follow [Arne's lab instructions](docs/arne-exercises.md).

**The Sunrise-to-simulator adapter is not implemented yet.** The simulator scene and connection are a starting point. They do not make the original exercise applications simulator-compatible. See [next tasks](docs/project.md) and [adapter design](docs/design.md).

## Where things belong

| Folder | Contents |
| --- | --- |
| `src/main/java/exercises`, `robot`, `utility` | Arne's exercises and support code |
| `src/main/java/sinkcleaning` | Our Java project code and local setup check |
| `learning/sinkcleaning` | Our Python learning code |
| `third_party/pglfd` | Unmodified Moodle snapshot; import as `pglfd` |
| `simulation` | Starter scene and simulation documentation |
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
