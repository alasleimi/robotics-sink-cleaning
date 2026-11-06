"""Run original exercise mains and physical adapter contracts in an owned simulator."""
from __future__ import annotations

import json
import math
from pathlib import Path
import time

from sim_session import Session


def wait_until(predicate, timeout=60, description="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError(f"Timed out waiting for {description}")


def main():
    with Session(headless=True, fast=True) as s:
        data = s.folder / "teaching"
        report = {"logs": str(s.folder), "checks": []}
        print(f"Integration logs: {s.folder}", flush=True)

        def complete(main, *args, label, timeout=120):
            p = s.java(main, *args, data=data, label=label)
            try:
                result = p.wait(timeout=timeout)
            except BaseException:
                p.terminate()
                p.wait(timeout=5)
                raise
            if result:
                raise AssertionError((s.folder / f"{label}.log").read_text())
            print(f"PASS {label}", flush=True)
            report["checks"].append(label)

        def stop(p):
            if p.poll() is None:
                p.terminate()
                p.wait(timeout=5)
            s.call("abort")
            s.call("resume")
            s.call("force", force=[0, 0, 0])

        def events():
            return s.call("trace")["events"]

        # Exercise 1 itself runs the original front movement and grav-comp loop.
        p = s.java("simproxy.Launcher", "01", data=data, label="exercise-01")
        try:
            def teaching_ready():
                if p.poll() is not None:
                    raise AssertionError((s.folder / "exercise-01.log").read_text())
                return sum(e["event"] == "finish" for e in events()) >= 3
            wait_until(teaching_ready, description="Exercise 1 teaching mode")
            s.call("pause")
            paused = s.call("state")["q"]
            changed = list(paused)
            changed[0] += .15
            s.call("teach", q=changed)
            time.sleep(.2)
            assert max(abs(a-b) for a, b in zip(paused, s.call("state")["q"])) < .003, "Pause must also freeze teaching controls"
            s.call("resume")
            for index in range(3):
                q = [.06*index, math.pi/6, .02*index, -math.pi/3, 0, math.pi/4, 0]
                s.call("teach", q=q)
                wait_until(lambda: max(abs(a-b) for a, b in zip(q, s.call("state")["q"])) < .001,
                           description=f"teaching pose {index}")
                complete("simproxy.ContractCheck", "save", label=f"save-{index}")
                assert (data / f"frames/F{index}.frm").is_file()
                assert (data / f"joint_positions/J{index}.jnt").is_file()
        finally:
            stop(p)
        report["checks"].append("original Exercise 1 + F0-F2/J0-J2 teaching")
        complete("simproxy.Launcher", "02", label="exercise-02")
        q0 = [float(v) for v in (data / "joint_positions/J0.jnt").read_text().split()]
        assert max(abs(a-b) for a, b in zip(q0, s.call("state")["q"])) < .015
        complete("simproxy.ContractCheck", label="motion-force-logging-contracts")
        for number in ("03", "04"):
            complete("simproxy.Launcher", number, label=f"exercise-{number}-scaffold")
        for number in ("05a", "05b", "06"):
            s.call("abort")
            baseline = len(events())
            p = s.java("simproxy.Launcher", number, data=data, label=f"exercise-{number}")
            try:
                def ran():
                    if p.poll() is not None:
                        raise AssertionError((s.folder / f"exercise-{number}.log").read_text())
                    recent = events()[baseline:]
                    errors = [e for e in recent if e["event"] == "error"]
                    if errors:
                        raise AssertionError(errors)
                    return sum(e["event"] == "finish" for e in recent) >= 3
                wait_until(ran, timeout=120, description=f"Exercise {number} repeated movement")
                if number == "06":
                    s.call("force", force=[0, 25, 0])
                    wait_until(lambda: any(e.get("fired") for e in events()[baseline:]), description="Exercise 6 force break")
                report["checks"].append(f"original Exercise {number} bounded loop")
                print(f"PASS exercise-{number}", flush=True)
            finally:
                stop(p)
        complete("simproxy.Launcher", "07", label="exercise-07", timeout=180)
        complete("simproxy.ContractCheck", "wire", label="teach-wire-start")
        complete("simproxy.Launcher", "08", label="exercise-08-startup-scaffold")
        path = s.folder / "report.json"
        path.write_text(json.dumps(report, indent=2))
        print(f"All simulation checks passed. Report: {path}", flush=True)


if __name__ == "__main__":
    main()
