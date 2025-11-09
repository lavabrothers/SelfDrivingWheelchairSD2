#!/bin/bash

# Activate the virtual environment
echo "Activating virtual environment..."
source /home/lavabrothers/Documents/SelfDrivingWheelchairSD2/pi-code/.venv/bin/activate

# Change to the project directory
echo "Changing directory to pi-code/Refactoring..."
cd /home/lavabrothers/Documents/SelfDrivingWheelchairSD2/pi-code/Refactoring

# Start a new interactive shell in the current environment
echo "Development environment is ready. Type 'exit' to return."
exec bash
