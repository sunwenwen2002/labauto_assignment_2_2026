
import time
import yaml

from scipy.io import savemat
from datetime import datetime

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from labauto import MuJoCoMechanicalSystem
from labauto import TrapezoidalMotionLaw
from labauto import loadController
from labauto import loadInstructions

from input_shaper_filter import InputShaperFilter, ShaperType

model_name = "crane"  # folder containing model.xml + control_config.yaml + motion program
program_name = "test_trj1"

# Load controller parameters and dynamic parameters
with open(f'{model_name}/control_config.yaml', 'r') as file:
    params_yaml = yaml.safe_load(file)
    controller_params = params_yaml['controller']
    dynamic_params = np.array(params_yaml['model_parameters'])

# Create simulator for Gantry SEA robot (MuJoCo)
xml_path = f"{model_name}/model.xml"
robot = MuJoCoMechanicalSystem(xml_path=xml_path, motor_actuators=["motor_1"], motor_joints=["joint_1"], spring_joints=[],ee_site="payload")
robot = MuJoCoMechanicalSystem(xml_path=xml_path)
robot.show()
robot.initialize()
dof = robot.get_input_number()

# Set the cycle time (sampling time) for motion law updates
Tc = robot.get_sampling_period()

# Load the tuned controller using parameters from YAML
decentralized_ctrl=loadController(Tc,controller_params,dynamic_params,model_name)
decentralized_ctrl.initialize()
decentralized_ctrl.set_umax(robot.get_umax())


# read from sensors
# compute control action
# write to actuators

# Initial reference is equal to the initial state of the robot
measured_output = robot.read_sensor_value()
q0 = measured_output[:dof]
Dq0 = measured_output[dof:]
DDq0 = np.zeros(dof)
initial_reference = np.concatenate((q0, Dq0, DDq0))

# Create the input shaper filter
input_shaper = InputShaperFilter(Tc=Tc, filter_type=ShaperType.ZVD, initial_reference=initial_reference)

# Define the Motion Law
max_Dq = np.array([5.5]*dof)
max_DDq = np.array([5.0]*dof)
motion_law_params=dict()
motion_law_params['max_velocity']=max_Dq
motion_law_params['max_acceleration']=max_DDq
ml = TrapezoidalMotionLaw(motion_law_params, Tc) # crea legge di moto
ml.set_initial_condition(q0)

# Define a sequence of motion instructions
instructions = loadInstructions(f'{model_name}/{program_name}.txt')
ml.add_instructions(instructions)

# Read the initial force (motor-side actuators)
joint_torque = robot.read_actuator_value()
feedforward_action = np.array([0.0]*dof)

print(f"joint_torque={joint_torque}, initial_reference={initial_reference}, measured_output={measured_output}")

decentralized_ctrl.starting(initial_reference, measured_output, joint_torque, feedforward_action)


# Simulation loop
t, measured_signal, control_action, reference_signal, link_position = [], [], [], [], []
actual_time = 0.0

while ml.depending_instructions():
    loop_t0 = time.perf_counter()
    target_q, target_Dq, target_DDq = ml.compute_motion_law()

    target_q_is = target_q[0]
    target_Dq_is = target_Dq[0]
    target_DDq_is = target_DDq[0]

    reference = np.array([target_q_is, target_Dq_is, target_DDq_is])
    # New reference after input shaping
    reference = input_shaper.filter(reference)
    measured_output = robot.read_sensor_value()

    # Controller computes desired actuator force (N) for the 3 motor actuators
    joint_torque = decentralized_ctrl.compute_control_action(reference, measured_output, feedforward_action)
    robot.write_actuator_value(joint_torque)

    # Store data (before stepping)
    t.append(actual_time)
    measured_signal.append(measured_output)
    control_action.append(joint_torque)
    reference_signal.append(reference)
    link_position.append(robot.link_position())
    actual_time += Tc

    # Step MuJoCo
    robot.simulate()

    # run close to real-time for teaching demos
    computation_time = time.perf_counter() - loop_t0
    time.sleep(max(0.0, Tc - computation_time))

t = np.array(t)
measured_signal = np.array(measured_signal)
control_action = np.array(control_action)
reference_signal = np.array(reference_signal)
link_position = np.array(link_position)

# Post-processing
joint_position = measured_signal[:, :dof]
joint_velocity = measured_signal[:, dof:]

reference_position = reference_signal[:, :dof]
reference_velocity = reference_signal[:, dof:2*dof]
reference_acceleration = reference_signal[:, 2*dof:]

# Save test data
timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
test_data = {
    "reference_position": reference_position,
    "reference_velocity": reference_velocity,
    "reference_acceleration": reference_acceleration,
    "joint_position": joint_position,
    "joint_velocity": joint_velocity,
    "joint_torque": control_action,
    "link_position": link_position,
    "time": t,
    "name": "test"
}
savemat(f"{model_name}/tests/{program_name}_{timestamp}.mat", {key: test_data[key] for key in test_data})



labels = ["x"]

# --- Figure 1: position / velocity / control (3x3) ---
fig1 = make_subplots(
    rows=3, cols=3,
    shared_xaxes=True,
    subplot_titles=[f"Position {a}" for a in labels] +
                   [f"Velocity {a}" for a in labels] +
                   [f"Actuator force {a} (motor-side)" for a in labels]
)

for i, a in enumerate(labels):
    col = i + 1

    # Position
    fig1.add_trace(go.Scatter(x=t, y=joint_position[:, i], name=f"q_{a}", legendgroup=f"pos_{a}"),
                   row=1, col=col)
    fig1.add_trace(go.Scatter(x=t, y=reference_position[:, i], name=f"qref_{a}",
                              legendgroup=f"pos_{a}", line=dict(dash="dash")),
                   row=1, col=col)

    # Velocity
    fig1.add_trace(go.Scatter(x=t, y=joint_velocity[:, i], name=f"dq_{a}", legendgroup=f"vel_{a}"),
                   row=2, col=col)
    fig1.add_trace(go.Scatter(x=t, y=reference_velocity[:, i], name=f"dqref_{a}",
                              legendgroup=f"vel_{a}", line=dict(dash="dash")),
                   row=2, col=col)

    # Control
    fig1.add_trace(go.Scatter(x=t, y=control_action[:, i], name=f"F_{a}", legendgroup=f"u_{a}"),
                   row=3, col=col)

fig1.update_xaxes(title_text="Time (s)", row=3, col=2)
fig1.update_layout(
    title="Tracking: position / velocity / control",
    height=900, width=1200,
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
)
fig1.update_xaxes(showgrid=True)
fig1.update_yaxes(showgrid=True)

# --- Errors ---
position_error = reference_position - joint_position
velocity_error = reference_velocity - joint_velocity
# MAE per axis (x,y,z)
mae_pos = np.mean(np.abs(position_error), axis=0)   # shape (3,)
mae_vel = np.mean(np.abs(velocity_error), axis=0)   # shape (3,)

subplot_titles = (
    [f"Position error {labels[i]} (MAE={mae_pos[i]:4.3f})" for i in range(dof)] +
    [f"Velocity error {labels[i]} (MAE={mae_vel[i]:4.3f})" for i in range(dof)] +
    [f"Actuator force {labels[i]} (motor-side)" for i in range(dof)]
)

# --- Figure 2: position error / velocity error / control (3x3) ---
fig2 = make_subplots(
    rows=3, cols=3,
    shared_xaxes=True,
    subplot_titles=subplot_titles
)

for i, a in enumerate(labels):
    col = i + 1

    # Position error
    fig2.add_trace(go.Scatter(x=t, y=position_error[:, i], name=f"e_q_{a}", legendgroup=f"ep_{a}"),
                   row=1, col=col)

    # Velocity error
    fig2.add_trace(go.Scatter(x=t, y=velocity_error[:, i], name=f"e_dq_{a}", legendgroup=f"ev_{a}"),
                   row=2, col=col)

    # Control (again, convenient to correlate with error)
    fig2.add_trace(go.Scatter(x=t, y=control_action[:, i], name=f"F_{a}", legendgroup=f"u_{a}"),
                   row=3, col=col)

fig2.update_xaxes(title_text="Time (s)", row=3, col=2)
fig2.update_layout(
    title="Errors: position error / velocity error / control",
    height=900, width=1200,
    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="left", x=0),
)
fig2.update_xaxes(showgrid=True)
fig2.update_yaxes(showgrid=True)

# Show both windows
fig1.show()
fig2.show()

robot.close()
