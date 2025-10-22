#!/bin/bash

# Step 1: Launch the ROS2 launch file in a new terminal
gnome-terminal -- bash -c "source /home/hitesh/Desktop/ROS2projects/ROS-Car/ros2_workspace/install/setup.bash; ros2 launch esp32_car launch_car.py; exec bash"

# Step 2: Wait for ngrok to start
sleep 5

# Step 3: Function to fetch the ngrok URL
fetch_ngrok_url() {
  local retries=10
  local delay=2
  for ((i=1; i<=retries; i++)); do
    response=$(curl -s http://127.0.0.1:4040/api/tunnels)
    if [[ $? -eq 0 ]]; then
      public_url=$(echo "$response" | jq -r '.tunnels[0].public_url')
      if [[ "$public_url" != "null" ]]; then
        echo "Ngrok URL: $public_url"
        return 0
      fi
    fi
    echo "Retrying to fetch ngrok URL ($i/$retries)..."
    sleep $delay
  done
  echo "Failed to fetch ngrok URL after $retries attempts."
  return 1
}

# Step 4: Call the function to fetch the ngrok URL
fetch_ngrok_url