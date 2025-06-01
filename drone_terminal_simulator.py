import numpy as np
import matplotlib.pyplot as plt


class LogPolynomialDescent:
    def __init__(self, z0, zf, total_time):
        self.z0 = z0
        self.zf = zf
        self.T = total_time

        A = np.array([
            [np.log(1), 0, 1],
            [np.log(self.T + 1), self.T**3, 1],
            [1 / (self.T + 1), 3 * self.T**2, 0]
        ])
        B = np.array([z0, zf, 0])
        self.a, self.b, self.c = np.linalg.solve(A, B)

    def position(self, t):
        return self.a * np.log(t + 1) + self.b * t**3 + self.c

    def velocity(self, t):
        return self.a / (t + 1) + 3 * self.b * t**2


class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(-np.inf, np.inf), integral_limits=(-10, 10)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        self.integral_limits = integral_limits

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, *self.integral_limits)
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        return np.clip(output, *self.output_limits)


class DroneSimulator:
    def __init__(self, z0=0.6, zf=0.0, duration=10.0, dt=0.01):  # <-- z0 changed to 0.6
        self.z0 = z0
        self.zf = zf
        self.dt = dt
        self.total_time = duration

        self.z = z0
        self.vz = 0.0
        self.mass = 1.0
        self.gravity = 9.81

        self.trajectory = LogPolynomialDescent(z0, zf, duration)

        # === PID gains ===
        kp = 120
        ki = 1
        kd = 15
        self.controller = PIDController(kp, ki, kd, dt,
                                        output_limits=(-20, 20),
                                        integral_limits=(-5, 5))

        self.time_log = []
        self.z_log = []
        self.z_des_log = []
        self.err_log = []
        self.control_log = []

        self.kp, self.ki, self.kd = kp, ki, kd  # Store for printing

    def run(self):
        t = 0.0
        while t <= self.total_time:
            z_des = self.trajectory.position(t)
            error = z_des - self.z
            u = self.controller.compute(z_des, self.z)

            # Vertical dynamics
            az = (u - self.mass * self.gravity) / self.mass
            self.vz += az * self.dt
            self.z += self.vz * self.dt

            self.time_log.append(t)
            self.z_log.append(self.z)
            self.z_des_log.append(z_des)
            self.err_log.append(error)
            self.control_log.append(u)

            t += self.dt

    def print_metrics(self):
        err_arr = np.abs(np.array(self.err_log))
        max_err = np.max(err_arr)
        final_error = err_arr[-1]
        overshoot = np.max(np.array(self.z_log)) - self.z0
        tolerance = 0.02  # meters

        settling_time = None
        for i in range(len(err_arr) - 1, -1, -1):
            if err_arr[i] > tolerance:
                settling_time = self.time_log[i + 1] if i + 1 < len(self.time_log) else self.total_time
                break

        rise_time = None
        for i, val in enumerate(self.z_log):
            if abs(val - self.zf) <= tolerance:
                rise_time = self.time_log[i]
                break

        print("\n=== PID Controller Gains ===")
        print(f"Kp = {self.kp}, Ki = {self.ki}, Kd = {self.kd}")

        print("\n=== Simulation Metrics ===")
        print(f"Final Error       : {final_error:.4f} m")
        print(f"Max Overshoot     : {overshoot:.4f} m")
        print(f"Max Absolute Error: {max_err:.4f} m")
        print(f"Rise Time         : {rise_time:.2f} s" if rise_time else "Rise Time         : N/A")
        print(f"Settling Time     : {settling_time:.2f} s" if settling_time else "Settling Time     : N/A")

    def plot(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_log, self.z_log, label="Actual Altitude", linewidth=2)
        plt.plot(self.time_log, self.z_des_log, label="Desired Altitude", linestyle='--')
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (m)")
        plt.title("Drone Landing Simulation (Log-Polynomial + PID)")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    sim = DroneSimulator(z0=0.6, zf=0.0, duration=10.0, dt=0.01)
    sim.run()
    sim.print_metrics()
    sim.plot()
