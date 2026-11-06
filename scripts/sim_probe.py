"""Read the simulator version over loopback ZeroMQ. No simulation or robot motion."""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(host="127.0.0.1", port=23000)
sim = client.require("sim")
print("Simulator API version:", sim.getInt32Param(sim.intparam_program_version))
print("Simulator connection OK.")
