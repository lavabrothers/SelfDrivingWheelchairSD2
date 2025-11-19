#!/bin/bash
#
# This script manages the mainflow systemd service.
# It must be run with sudo privileges.
#
# Usage: sudo ./manage_service.sh [start|stop|restart|install]
#

# --- Configuration ---
SERVICE_NAME="mainflow.service"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SOURCE_FILE="${SCRIPT_DIR}/${SERVICE_NAME}"
DEST_PATH="/etc/systemd/system/${SERVICE_NAME}"

# --- Sudo Check ---
# Check if the script is being run as root (with sudo)
if [ "$EUID" -ne 0 ]; then
  echo "Error: This script must be run with sudo."
  echo "Usage: sudo $0 [start|stop|restart|install]"
  exit 1
fi

# --- Helper Function ---
# This function will be called if the user gives no input or bad input
usage() {
  echo "Usage: sudo $0 [action]"
  echo
  echo "Available actions:"
  echo "  start    - Starts the ${SERVICE_NAME}"
  echo "  stop     - Stops the ${SERVICE_NAME}"
  echo "  restart  - Restarts the ${SERVICE_NAME}"
  echo "  install  - Installs and enables the ${SERVICE_NAME}"
  echo "  status   - Checks the status of the ${SERVICE_NAME}"
  exit 1
}

# Get the action from the first command-line argument
ACTION="$1"

# --- Main Logic ---
# Use a case statement to decide which action to perform
case "$ACTION" in
  start)
    echo "Starting ${SERVICE_NAME}..."
    systemctl start "$SERVICE_NAME"
    echo "Done."
    ;;

  stop)
    echo "Stopping ${SERVICE_NAME}..."
    systemctl stop "$SERVICE_NAME"
    echo "Done."
    ;;

  restart)
    echo "Restarting ${SERVICE_NAME}..."
    systemctl restart "$SERVICE_NAME"
    echo "Done."
    ;;

  install)
    echo "--- Installing ${SERVICE_NAME} ---"
    
    # Check if the source service file exists
    if [ ! -f "$SOURCE_FILE" ]; then
      echo "Error: ${SOURCE_FILE} not found in the current directory."
      echo "Please run this script from the same directory as your .service file."
      exit 1
    fi

    # Copy the service file to the systemd directory
    # Using 'cp' is safer than 'mv' so you keep your original file
    echo "Copying ${SOURCE_FILE} to ${DEST_PATH}..."
    cp "$SOURCE_FILE" "$DEST_PATH"

    # Reload the systemd daemon to recognize the new service
    echo "Reloading systemd daemon..."
    systemctl daemon-reload

    # Enable the service (so it starts on boot)
    echo "Enabling ${SERVICE_NAME} to start on boot..."
    systemctl enable "$SERVICE_NAME"

    # Restart the service
    echo "Restarting ${SERVICE_NAME}..."
    systemctl restart "$SERVICE_NAME"

    echo
    echo "Installation complete."
    echo "You can check its status with: sudo systemctl status ${SERVICE_NAME}"
    ;;

  status)
    echo "Checking status of ${SERVICE_NAME}..."
    # 'status' doesn't need sudo, but it's fine since the script requires it.
    # We use 'systemctl status' and don't exit, so the script can continue.
    systemctl status "$SERVICE_NAME"
    ;;

  *)
    # If the action is empty or unknown, show the help message
    echo "Error: Invalid action '$ACTION'."
    echo
    usage
    ;;
esac

exit 0
