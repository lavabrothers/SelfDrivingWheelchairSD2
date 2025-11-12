#!/bin/bash

# This script installs and enables the mainflow systemd service.
# It must be run with sudo privileges.

# Move the service file to the systemd directory
echo "Moving mainflow.service to /etc/systemd/system..."
mv mainflow.service /etc/systemd/system/mainflow.service

# Reload the systemd daemon to recognize the new service
echo "Reloading systemd daemon..."
systemctl daemon-reload

# Restart the service
echo "Restarting mainflow service..."
systemctl restart mainflow.service

echo "Installation complete."
echo "The mainflow service has been updated and restarted."
echo "You can check its status with: sudo systemctl status mainflow.service"
