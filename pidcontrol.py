class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint  # Desired value
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        """
        Update the PID control output based on the current value.
        
        :param current_value: The current measurement of the value to control.
        :param dt: Time interval in seconds.
        :return: Control variable output.
        """
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

# Example usage
def simulate_pid_control():
    import numpy as np
    import matplotlib.pyplot as plt
    
    # Simulation settings
    total_time = 100  # total simulation time in seconds
    dt = 1            # time step in seconds
    target_position = 100  # target position in meters
    current_position = 0   # initial position of the ship
    
    # PID Controller setup
    pid = PIDController(kp=0.2, ki=0.05, kd=0.1, setpoint=target_position)
    
    # Simulation loop
    positions = [current_position]
    times = np.arange(0, total_time, dt)
    
    for t in times[1:]:
        control_action = pid.update(current_position, dt)
        # Update the position (simple model: position change is proportional to the control action)
        current_position += control_action * dt
        positions.append(current_position)
    
    # Plot results
    plt.figure(figsize=(10, 5))
    plt.plot(times, positions, label='Ship Position')
    plt.axhline(target_position, color='r', linestyle='--', label='Target Position')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position (meters)')
    plt.title('PID Control of Ship Position for Berthing')
    plt.legend()
    plt.grid(True)
    plt.show()

simulate_pid_control()