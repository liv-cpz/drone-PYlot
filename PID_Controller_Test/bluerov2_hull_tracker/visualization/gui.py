#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')  # Set backend before importing pyplot
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# Import from local package using relative imports
from ..dynamics.marine_dynamics import MarineDynamics
from ..control.pid_controller import PIDController
from ..environment.hull_profiles import HullProfiles

class BlueROV2HullTrackerGUI:
    """Interactive GUI for BlueROV2 hull tracking with PID control"""

    def __init__(self):
        self.dynamics = MarineDynamics()
        self.pid = PIDController()
        self.hull_profiles = HullProfiles()

        # Simulation settings
        self.dt = 0.1
        self.sim_time = 0
        self.max_time = 60.0
        self.trajectory = []
        self.target_idx = 0

        # Store multiple trajectories
        self.logged_trajectories = []
        self.color_cycle = plt.rcParams['axes.prop_cycle'].by_key()['color']
        self.color_index = 0

        self._init_ui()

    def _init_ui(self):
        self.fig, (self.ax, self.dev_ax) = plt.subplots(2, 1, figsize=(10, 8))
        plt.subplots_adjust(bottom=0.35)

        self.hull_points = self.hull_profiles.tanker_hull(length=10.0)
        self.target_point = np.array([*self.hull_points[0], 0, 0, 0, 0])

        self.hull_line, = self.ax.plot(self.hull_points[:, 0], self.hull_points[:, 1], 'b-', linewidth=2, label='Ship Hull')
        self.rov_pos, = self.ax.plot([], [], 'ro', markersize=8, label='ROV Position')
        self.target_pos, = self.ax.plot([], [], 'gx', markersize=10, label='Target')
        self.traj_line, = self.ax.plot([], [], 'r--', linewidth=1, label='Current Trajectory')

        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-1.0, 1.0)
        self.ax.set_xlabel('X Position [m]')
        self.ax.set_ylabel('Y Position [m]')
        self.ax.set_title('BlueROV2 Ship Hull Tracking')
        self.ax.grid(True)
        self.ax.legend()

        self.dev_ax.set_title("ROV Deviation from Hull Over Time")
        self.dev_ax.set_xlabel("Time [s]")
        self.dev_ax.set_ylabel("Deviation [m]")
        self.dev_ax.grid(True)

        self._create_widgets()

    def _create_widgets(self):
        axcolor = 'lightgoldenrodyellow'
        slider_ax = [
            plt.axes([0.25, 0.28, 0.65, 0.03], facecolor=axcolor),
            plt.axes([0.25, 0.23, 0.65, 0.03], facecolor=axcolor),
            plt.axes([0.25, 0.18, 0.65, 0.03], facecolor=axcolor)
        ]

        self.sliders = [
            Slider(slider_ax[0], 'Kp', 0.1, 10.0, valinit=self.pid.Kp[0]),
            Slider(slider_ax[1], 'Ki', 0.0, 5.0, valinit=self.pid.Ki[0]),
            Slider(slider_ax[2], 'Kd', 0.0, 5.0, valinit=self.pid.Kd[0])
        ]
        for slider in self.sliders:
            slider.on_changed(self._update_pid_gains)

        # Reset button
        self.reset_ax = plt.axes([0.25, 0.08, 0.2, 0.05])
        self.reset_button = Button(self.reset_ax, 'Reset', color=axcolor)
        self.reset_button.on_clicked(self.reset_simulation)

        # Update button
        self.update_ax = plt.axes([0.55, 0.08, 0.2, 0.05])
        self.update_button = Button(self.update_ax, 'Update', color=axcolor)
        self.update_button.on_clicked(self._update_trajectory_once)

    def _update_pid_gains(self, val):
        self.pid.Kp = np.array([self.sliders[0].val] * 2 + [1.0, 0.5, 0.5, 1.0])
        self.pid.Ki = np.array([self.sliders[1].val] * 2 + [0.05, 0.01, 0.01, 0.05])
        self.pid.Kd = np.array([self.sliders[2].val] * 2 + [0.5, 0.1, 0.1, 0.5])

    def _update_trajectory_once(self, event=None):
        """Simulate and plot ROV trajectory with current PID gains."""
        self.reset_simulation()  # Clears previous trajectory

        while self.sim_time < self.max_time:
            current_pos = self.dynamics.eta[:2]
            if np.linalg.norm(current_pos - self.target_point[:2]) < 0.2:
                self.target_idx = min(self.target_idx + 1, len(self.hull_points) - 1)
                self.target_point = np.array([*self.hull_points[self.target_idx], 0, 0, 0, 0])

            control = self.pid.compute(self.target_point, self.dynamics.eta)
            tau = np.zeros(6)
            tau[:2] = control[:2]
            self.dynamics.update(tau, self.dt)
            self.trajectory.append(self.dynamics.eta.copy())
            self.sim_time += self.dt

        traj = np.array(self.trajectory)
        color = self.color_cycle[self.color_index % len(self.color_cycle)]
        label = f"Kp={self.sliders[0].val:.1f}, Ki={self.sliders[1].val:.1f}, Kd={self.sliders[2].val:.1f}"

        self.ax.plot(traj[:, 0], traj[:, 1], linestyle='-', color=color, label=label)
        self.ax.legend()
        self.color_index += 1

        # Plot deviation from hull
        deviations = []
        for eta in self.trajectory:
            rov_pos = eta[:2]
            dists = np.linalg.norm(self.hull_points - rov_pos, axis=1)
            deviations.append(np.min(dists))
        time_vector = np.arange(len(deviations)) * self.dt
        self.dev_ax.plot(time_vector, deviations, color=color, label=label)
        self.dev_ax.legend()

        self._update_plot()

    def _update_plot(self):
        if self.trajectory:
            traj = np.array(self.trajectory)
            self.rov_pos.set_data(traj[-1, 0], traj[-1, 1])
            self.target_pos.set_data(self.target_point[0], self.target_point[1])
            self.traj_line.set_data(traj[:, 0], traj[:, 1])
        self.fig.canvas.draw_idle()

    def reset_simulation(self, event=None):
        """Reset ROV and trajectory but keep logged paths."""
        self.dynamics.reset()
        self.pid.reset()
        self.sim_time = 0
        self.target_idx = 0
        self.target_point = np.array([*self.hull_points[0], 0, 0, 0, 0])
        self.trajectory = []
        self.traj_line.set_data([], [])
        self.rov_pos.set_data([], [])
        self.target_pos.set_data([], [])
        self.fig.canvas.draw_idle()

    def run(self):
        plt.show()

if __name__ == "__main__":
    gui = BlueROV2HullTrackerGUI()
    gui.run()
