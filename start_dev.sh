#!/bin/bash

# Get the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define paths relative to the script's directory
VENV_PATH="${SCRIPT_DIR}/pi-code/.venv/bin/activate"
PROJECT_DIR="${SCRIPT_DIR}/pi-code/Refactoring"

# Activate the virtual environment
echo "Activating virtual environment..."
source "$VENV_PATH"

# Change to the project directory
echo "Changing directory to ${PROJECT_DIR}..."
cd "$PROJECT_DIR"

# Start a new interactive shell in the current environment
echo "Development environment is ready. Type 'exit' to return."
exec bash
