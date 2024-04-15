import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def setup_tug_control():
    # Setup fuzzy control system for tug operation
    distance = ctrl.Antecedent(np.arange(0, 500, 1), 'distance')
    velocity = ctrl.Antecedent(np.arange(0, 10, 1), 'velocity')
    tug_force = ctrl.Consequent(np.arange(0, 100, 1), 'tug_force')

    # Auto-membership function population is possible with .automf(3, 5, or 7)
    distance.automf(3)
    velocity.automf(3)

    # Custom membership functions can be built interactively with a GUI
    tug_force['low'] = fuzz.trimf(tug_force.universe, [0, 25, 50])
    tug_force['medium'] = fuzz.trimf(tug_force.universe, [25, 50, 75])
    tug_force['high'] = fuzz.trimf(tug_force.universe, [50, 75, 100])

    # Fuzzy rules
    rule1 = ctrl.Rule(distance['poor'] | velocity['high'], tug_force['high'])
    rule2 = ctrl.Rule(distance['average'], tug_force['medium'])
    rule3 = ctrl.Rule(distance['good'] & velocity['low'], tug_force['low'])

    tug_control = ctrl.ControlSystem([rule1, rule2, rule3])
    tug = ctrl.ControlSystemSimulation(tug_control)
    return tug

def get_tug_force(tug, dist, vel):
    tug.input['distance'] = dist
    tug.input['velocity'] = vel
    tug.compute()
    return tug.output['tug_force']