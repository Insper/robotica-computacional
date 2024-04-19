import numpy as np
import matplotlib.pyplot as plt
from ipywidgets import interact, FloatSlider
import time

# Simulation parameters
setpoint = 1.0  # Desired value
initial_value = 0.0  # Initial actual value

def proportional_control(setpoint, measured, Kp):
    """
    Proportional control logic.
    
    :param setpoint: Desired set point.
    :param measured: Current measured value.
    :param Kp: Proportional gain.
    :return: control signal.
    """
    error = setpoint - measured
    control_signal = Kp * error
    return control_signal

def system_response(control_signal, current_value, update_time):
    """
    Simple system response simulation.
    
    :param control_signal: Control signal from the controller.
    :param current_value: Current value of the system.
    :param update_time: Time interval for update.
    :return: Updated system value.
    """
    # Simple model where the system response is proportional to the control signal
    new_value = current_value + control_signal * update_time
    return new_value

def run_simulation(Kp, update_time):
    """
    Run the simulation with the given Kp and update_time.
    
    :param Kp: Proportional gain.
    :param update_time: Update interval.
    """
    times = np.arange(0, 10, update_time)  # Simulate for 10 seconds
    values = []
    measured_value = initial_value
    
    for t in times:
        control_signal = proportional_control(setpoint, measured_value, Kp)
        measured_value = system_response(control_signal, measured_value, update_time)
        values.append(measured_value)
    
    plt.figure(figsize=(10, 5))
    plt.plot(times, values, label='System Response')
    plt.plot(times, [setpoint] * len(times), 'r--', label='Setpoint')
    plt.title(f'System Response with Kp={Kp} and Update Time={update_time}s')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Value')
    plt.legend()
    plt.grid(True)
    plt.show()

# Interactive widget
interact(run_simulation, 
         Kp=FloatSlider(value=1.0, min=0.0, max=10.0, step=0.1, description='Kp:'),
         update_time=FloatSlider(value=1.0, min=0.1, max=2.0, step=0.1, description='Update Time:'))
