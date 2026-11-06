# Project continuity

Read README.md and docs/project.md first. Keep replies and project notes concise and concrete.

Use the KUKA Sunrise API required by the course. The original exercise files and robot configuration are the baseline; simulator compatibility is not yet implemented. Never treat successful simulation as a validated hardware motion.

Use `uv run --locked scripts/setup.py` for setup, `uv run --locked scripts/run.py build` for Java compilation/local SDK checks, and `uv run --locked pytest -q` for setup-helper tests. Keep installed tools out of Git.

Preserve `third_party/pglfd` as the supplied snapshot. Put integration code outside it. Project notes belong in docs; personal planning does not belong in this repository. Record what was proposed separately from what was tested.
