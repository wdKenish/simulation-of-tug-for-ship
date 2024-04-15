import tkinter as tk
from tkinter import simpledialog, messagebox

# Function to handle the start of simulation
def start_simulation(ship_params, tug_params, initial_state, target_position):
    # Placeholder for the actual simulation call
    messagebox.showinfo("Simulation Started", "Simulation has started with the provided parameters.")

# Main application window
class ShipSimulationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ship Berthing Simulation")
        self.geometry("400x300")
        
        # Entry fields setup
        self.entries = {}
        self.create_entry("Ship Length", "L_pp", "180.0")
        self.create_entry("Ship Beam", "B", "30.0")
        self.create_entry("Initial Position X", "initial_x", "0.0")
        self.create_entry("Initial Position Y", "initial_y", "0.0")
        self.create_entry("Target Position X", "target_x", "200")
        self.create_entry("Target Position Y", "target_y", "50")
        
        # Start button
        start_btn = tk.Button(self, text="Start Simulation", command=self.on_start)
        start_btn.pack(pady=20)

    def create_entry(self, label_text, entry_key, default_value):
        frame = tk.Frame(self)
        frame.pack(padx=10, pady=5)
        label = tk.Label(frame, text=label_text)
        label.pack(side=tk.LEFT)
        entry = tk.Entry(frame)
        entry.pack(side=tk.RIGHT)
        entry.insert(0, default_value)
        self.entries[entry_key] = entry

    def on_start(self):
        ship_params = {
            'L_pp': float(self.entries['L_pp'].get()),
            'B': float(self.entries['B'].get())
        }
        initial_state = {
            'position': [float(self.entries['initial_x'].get()), float(self.entries['initial_y'].get())]
        }
        target_position = {
            'x': float(self.entries['target_x'].get()),
            'y': float(self.entries['target_y'].get())
        }
        tug_params = {}  # Add similar inputs for tug parameters if necessary
        start_simulation(ship_params, tug_params, initial_state, target_position)

# Run the application
if __name__ == "__main__":
    app = ShipSimulationApp()
    app.mainloop()
    
import tkinter as tk
from tkinter import simpledialog, messagebox
import matplotlib.pyplot as plt
import numpy as np

# Assuming this function is part of the simulation logic
def run_simulation(params):
    # Simulation placeholder
    # Returns positions and velocities for demonstration
    positions = np.cumsum(np.random.rand(100, 2) - 0.5, axis=0)  # Random walk for demonstration
    velocities = np.random.rand(100, 2) - 0.5
    return positions, velocities

def plot_results(positions, velocities):
    plt.figure(figsize=(10, 5))
    plt.quiver(positions[:, 0], positions[:, 1], velocities[:, 0], velocities[:, 1], scale=5)
    plt.title('Ship Trajectory and Velocity Vectors')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    plt.show()

class ShipSimulationApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ship Berthing Simulation")
        self.geometry("500x600")
        
        self.entries = {}
        self.setup_entries()

        start_btn = tk.Button(self, text="Start Simulation", command=self.on_start)
        start_btn.pack(pady=20)

    def setup_entries(self):
        labels = [
            ("Ship Length", "L_pp", "180.0"),
            ("Ship Beam", "B", "30.0"),
            ("Initial Position X", "initial_x", "0.0"),
            ("Initial Position Y", "initial_y", "0.0"),
            ("Target Position X", "target_x", "200"),
            ("Target Position Y", "target_y", "50"),
            ("Wind Speed (m/s)", "wind_speed", "15"),
            ("Wind Angle (degrees)", "wind_angle", "45"),
            ("Wave Amplitude (m)", "wave_amplitude", "2"),
            ("Wave Direction (degrees)", "wave_direction", "30"),
            ("Current Speed (m/s)", "current_speed", "2"),
            ("Current Angle (degrees)", "current_angle", "60")
        ]
        for text, key, default in labels:
            self.create_entry(text, key, default)

    def create_entry(self, label_text, entry_key, default_value):
        frame = tk.Frame(self)
        frame.pack(padx=10, pady=5, fill=tk.X)
        label = tk.Label(frame, text=label_text)
        label.pack(side=tk.LEFT)
        entry = tk.Entry(frame)
        entry.pack(side=tk.RIGHT, expand=True, fill=tk.X)
        entry.insert(0, default_value)
        self.entries[entry_key] = entry

    def on_start(self):
        params = {key: float(entry.get()) for key, entry in self.entries.items()}
        positions, velocities = run_simulation(params)
        plot_results(positions, velocities)

if __name__ == "__main__":
    app = ShipSimulationApp()
    app.mainloop()