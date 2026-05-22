import math
import time

import numpy as np
import plotly.graph_objects as go
from labauto import MuJoCoMechanicalSystem


model_name = "crane"
xml_path = f"{model_name}/model.xml"

# Simulation parameters
simulation_time = 10.0
impulse_area = 1.0
max_force = 100.0


def get_mujoco_data(robot):
    if hasattr(robot, "data"):
        return robot.data
    if hasattr(robot, "_data"):
        return robot._data
    if hasattr(robot, "mj_data"):
        return robot.mj_data

    raise RuntimeError("MuJoCo data not found in the robot object.")


def read_theta(robot):
    data = get_mujoco_data(robot)

    try:
        return float(data.joint("joint_2").qpos[0])
    except Exception:
        return float(data.qpos[1])


# Create simulator for the crane
robot = MuJoCoMechanicalSystem(
    xml_path=xml_path,
    motor_actuators=["motor_1"],
    motor_joints=["joint_1"],
    spring_joints=[],
    ee_site="payload"
)

robot.show()
robot.initialize()
dof = robot.get_input_number()

# Set the cycle time
Tc = robot.get_sampling_period()

# A perfect impulse cannot be simulated, so we use a short rectangular pulse.
# The area of the pulse is equal to 1 N*s.
impulse_steps = math.ceil(impulse_area / (max_force * Tc))
impulse_force = impulse_area / (impulse_steps * Tc)

print(f"Tc = {Tc}")
print(f"Impulse force = {impulse_force}")
print(f"Impulse steps = {impulse_steps}")
print(f"Impulse area = {impulse_force * impulse_steps * Tc}")


# Simulation loop
t = []
theta = []
actual_time = 0.0
simulation_steps = int(simulation_time / Tc)

for k in range(simulation_steps):
    loop_t0 = time.perf_counter()

    if k < impulse_steps:
        force = impulse_force
    else:
        force = 0.0

    # Apply the force to the motor-side actuator.
    robot.write_actuator_value(np.array([force] * dof))

    t.append(actual_time)
    theta.append(read_theta(robot))

    actual_time += Tc
    robot.simulate()

    # run close to real-time for teaching demos
    computation_time = time.perf_counter() - loop_t0
    time.sleep(max(0.0, Tc - computation_time))


t = np.array(t)
theta = np.array(theta)

# Estimate damping ratio with logarithmic decrement.
theta_no_offset = theta - np.mean(theta)
peaks = []

# Find local maximum in the theta response.
for i in range(1, len(theta_no_offset) - 1):
    if theta_no_offset[i] > theta_no_offset[i - 1] and theta_no_offset[i] > theta_no_offset[i + 1]:
        if theta_no_offset[i] > 0:
            peaks.append(i)

# Estimate xi using the first 5 peaks (if available) using the logarithmic decrement method and calculate the mean xi value.
if len(peaks) >= 2:
    theta_1 = abs(theta_no_offset[peaks[0]])
    max_n = min(4, len(peaks) - 1)
    xi_values = []

    print(f"theta_1 = {theta_1}")

    for n in range(1, max_n + 1):
        theta_n = abs(theta_no_offset[peaks[n]])

        delta = (1 / n) * np.log(theta_1 / theta_n)
        xi = delta / np.sqrt(4 * np.pi**2 + delta**2)

        Td = (t[peaks[n]] - t[peaks[0]]) / n
        wd = 2 * np.pi / Td

        xi_values.append(xi)

        print(f"peak 1 -> peak {n + 1}")
        print(f"theta_n = {theta_n}")
        print(f"logarithmic decrement = {delta}")
        print(f"xi = {xi}")
        print(f"Td = {Td}")
        print(f"wd = {wd}")

    print(f"xi mean = {np.mean(xi_values)}")
else:
    print("Not enough peaks to estimate xi.")


# Plot theta response
fig = go.Figure()
fig.add_trace(
    go.Scatter(
        x=t,
        y=theta,
        name="theta",
        mode="lines"
    )
)

fig.update_layout(
    title="Impulse response: theta",
    xaxis_title="Time (s)",
    yaxis_title="theta (rad)",
    height=700,
    width=1000,
)

fig.update_xaxes(showgrid=True)
fig.update_yaxes(showgrid=True)

fig.show()

robot.close()
