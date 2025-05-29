#!/bin/bash

# Navigate to your project directory
cd ~/Documents/Thesis/PID_Controller_Test

# Create directory structure
mkdir -p bluerov2_hull_tracker/{dynamics,control,environment,visualization,utils}

# Create __init__.py files
touch bluerov2_hull_tracker/__init__.py
touch bluerov2_hull_tracker/dynamics/__init__.py
touch bluerov2_hull_tracker/control/__init__.py
touch bluerov2_hull_tracker/environment/__init__.py
touch bluerov2_hull_tracker/visualization/__init__.py
touch bluerov2_hull_tracker/utils/__init__.py

# Create empty Python files
touch bluerov2_hull_tracker/dynamics/bluerov_model.py
touch bluerov2_hull_tracker/dynamics/marine_dynamics.py
touch bluerov2_hull_tracker/control/pid_controller.py
touch bluerov2_hull_tracker/environment/hull_profiles.py
touch bluerov2_hull_tracker/visualization/gui.py
touch bluerov2_hull_tracker/utils/helpers.py

echo "Directory structure and empty Python files created successfully."
