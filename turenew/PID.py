
from math import sin, cos, radians

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, error, setpoint, dt):
        """
        Calculate PID control output for a given error and setpoint.

        :param error: The current error.
        :param setpoint: The desired setpoint.
        :param dt: Time step in seconds.
        :return: Control output.
        """
        # Proportional term
        p = self.kp * error

        # Integral term
        self.integral += error * dt
        i = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / dt
        d = self.kd * derivative

        # Update previous error
        self.previous_error = error

        # Compute final output
        output = p + i + d
        return output


class Ship:
    def __init__(self, position, heading, mass, I_zz, velocity):
        self.position = list(position)  # position will be a tuple (x, y), convert to list for mutability
        self.heading = heading
        self.mass = mass
        self.I_zz = I_zz
        self.velocity = list(velocity)  # velocity will be a tuple (vx, vy), convert to list for mutability



class ShipSimulation:
    def __init__(self, ship, pid_heading, pid_position, pid_speed=None):
        """
        Initialize the ship simulation with optional speed control.

        :param ship: A Ship object.
        :param pid_heading: A PIDController for heading control.
        :param pid_position: A PIDController for position control.
        :param pid_speed: An optional PIDController for speed control.
        """
        self.ship = ship
        self.pid_heading = pid_heading
        self.pid_position = pid_position
        self.pid_speed = pid_speed

    def update_ship_state(self, target_position, target_heading, dt, target_speed=None):
        """
        Update the ship state considering optional speed control.

        :param target_position: The target position as a tuple (x, y).
        :param target_heading: The target heading in degrees.
        :param dt: Time step in seconds.
        :param target_speed: Optional target speed for the ship.
        :return: Tuple of (force_x, force_y, torque, [optional] ship's velocity)
        """
        # Compute position and heading errors
        pos_error_x = target_position[0] - self.ship.position[0]
        pos_error_y = target_position[1] - self.ship.position[1]
        position_error = (pos_error_x**2 + pos_error_y**2)**0.5
        heading_error = (target_heading - self.ship.heading + 180) % 360 - 180

        # Compute thrust and torque without mode determination
        thrust = self.pid_position.compute(position_error, 0, dt)
        torque = self.pid_heading.compute(heading_error, 0, dt)

        if self.pid_speed and target_speed is not None:
            current_speed = (self.ship.velocity[0]**2 + self.ship.velocity[1]**2)**0.5
            speed_error = target_speed - current_speed
            speed_adjustment = self.pid_speed.compute(speed_error, 0, dt)
            thrust += speed_adjustment

        force_x = thrust * cos(radians(self.ship.heading))
        force_y = thrust * sin(radians(self.ship.heading))

        # Apply forces and update the ship's state
        self.ship.velocity[0] += force_x * dt / self.ship.mass
        self.ship.velocity[1] += force_y * dt / self.ship.mass
        self.ship.position[0] += self.ship.velocity[0] * dt
        self.ship.position[1] += self.ship.velocity[1] * dt
        self.ship.heading += torque * dt / self.ship.I_zz
        self.ship.heading %= 360  # Normalize heading

        if self.pid_speed:
            return (force_x, force_y, torque, self.ship.velocity)
        else:
            return (force_x, force_y, torque)




def control_foreces(initial_position:tuple, initial_heading, initial_velocity:tuple, mass, I_zz,
                        pid_parameters,# List of tuples for PID parameters
                        target_position, target_heading, target_speed, steps, dt):
    # Setup PID controllers
    pid_heading  = PIDController(*pid_parameters[0])
    pid_position = PIDController(*pid_parameters[1])
    pid_speed    = PIDController(*pid_parameters[2])

    # Initialize the ship
    ship = Ship(position=initial_position, heading=initial_heading, mass=mass, velocity=initial_velocity, I_zz=I_zz)

    # Create the simulation
    simulation = ShipSimulation(ship=ship, pid_heading=pid_heading, pid_position=pid_position, pid_speed=pid_speed)

    # Simulation loop
    results = []
    for _ in range(steps):
        output = simulation.update_ship_state(target_position, target_heading, dt, target_speed)
        results.append(output)
    return results







'''
# test
pid_heading = PIDController(kp=0.1, ki=0.01, kd=0.05)
pid_position = PIDController(kp=0.2, ki=0.02, kd=0.1)
pid_speed = PIDController(kp=0.1, ki=0.01, kd=0.02)


initial_position = (0, 0)  # Starting at the origin
initial_heading = 90  # Facing east
initial_velocity = (0, 0)  # Stationary at the start
ship_mass = 1000  # Mass of the ship in kg

ship = Ship(position=initial_position, heading=initial_heading, mass=ship_mass, velocity=initial_velocity)


simulation = ShipSimulation(ship=ship, pid_heading=pid_heading, pid_position=pid_position, pid_speed=pid_speed)


dt = 0.1  # Time step for the simulation in seconds
target_position = (100, 100)  # Where you want the ship to go
target_heading = 0  # Desired heading in degrees
target_speed = 5  # Target speed in units per second

for _ in range(100):  # Run the simulation for a number of steps
    outputs = simulation.update_ship_state(target_position, target_heading, dt, target_speed)
    print(outputs)
'''