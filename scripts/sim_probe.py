"""Probe an owned robosuite scene, without opening a viewer or connecting to hardware."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation/adapter"))
from model import Model

m = Model()
try:
    initial = m.state()
    for _ in range(10):
        m.step()
    state = m.state()
    assert state["time"] > initial["time"]
    assert np.max(np.abs(np.array(state["q"])-initial["q"])) < .001
    assert abs(np.linalg.norm(np.array(state["tcp"][:3])-state["flange"][:3])-.202) < 1e-6
    print("Robosuite iiwa7 physics and 202 mm TCP: OK")
finally:
    m.close()
