#!/usr/bin/env python3

from bluerov2_hull_tracker.visualization.terminal_simulator import TerminalSimulator
from bluerov2_hull_tracker.control.pid_sweeper import run_sweeper
from bluerov2_hull_tracker.visualization.plotter import plot_results

def run_terminal_simulator():
    print("BlueROV2 Hull Tracking - PID Controller Comparison (Terminal Mode)")
    simulator = TerminalSimulator()
    num_controllers = int(input("How many controllers do you want to compare? (1-5) ") or "3")
    num_controllers = max(1, min(5, num_controllers))
    results, hull_points = simulator.run_comparison(num_controllers)
    plot_results(results, hull_points)

def run_pid_sweeper():
    print("Running PID Parameter Sweeper...")

    try:
        kp_min = float(input("Enter minimum Kp value (default -10): ") or "-10")
        kp_max = float(input("Enter maximum Kp value (default 10): ") or "10")
        ki_min = float(input("Enter minimum Ki value (default -10): ") or "-10")
        ki_max = float(input("Enter maximum Ki value (default 10): ") or "10")
        kd_min = float(input("Enter minimum Kd value (default -10): ") or "-10")
        kd_max = float(input("Enter maximum Kd value (default 10): ") or "10")
        num_samples = int(input("Enter number of gain samples per axis (default 20): ") or "20")
        max_sim_time = float(input("Enter maximum simulation time in seconds (default 30): ") or "30")
        offset_distance = float(input("Enter Y-offset distance for hull (default 0): ") or "0.0")
    except ValueError:
        print("Invalid input(s). Using default ranges and values.")
        kp_min, kp_max = -10, 10
        ki_min, ki_max = -10, 10
        kd_min, kd_max = -10, 10
        num_samples = 20
        max_sim_time = 30.0
        offset_distance = 0.0

    results, hull_points, offset = run_sweeper(
        kp_min=kp_min, kp_max=kp_max,
        ki_min=ki_min, ki_max=ki_max,
        kd_min=kd_min, kd_max=kd_max,
        num_samples=num_samples,
        max_sim_time=max_sim_time,
        offset_distance=offset_distance
    )
    plot_results(results, hull_points, offset=offset_distance)

def main():
    print("BlueROV2 PID Controller Tuning")
    print("1. Terminal PID Simulator (compare specific controllers)")
    print("2. Optimized PID Sweeper (find best parameters)")

    choice = input("Enter choice (1/2): ").strip()
    if choice == '1':
        run_terminal_simulator()
    elif choice == '2':
        run_pid_sweeper()
    else:
        print("Invalid choice. Exiting.")

if __name__ == "__main__":
    main()
