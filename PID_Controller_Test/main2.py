#!/usr/bin/env python3

def main():
    """Main entry point for the application"""
    try:
        from bluerov2_hull_tracker.visualization.gui import BlueROV2HullTrackerGUI
        print("Starting BlueROV2 Hull Tracker GUI...")
        app = BlueROV2HullTrackerGUI()
        app.run()
    except ImportError as e:
        print(f"Error: {e}")
        print("Please make sure all package dependencies are installed")
        print("Try running: pip install numpy matplotlib")

if __name__ == "__main__":
    main()