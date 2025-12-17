# ai_model_client
This ROS2 node provides a http interface for AI model cloud-based object detection. It captures the most recent frame from a camera stream and, upon request, uploads it to a remote AI service via http.

## Installation
```bash
sudo apt update && sudo apt install -y python3-requests python3-opencv python3-flask
cd ~/ros2_ws && colcon build --symlink-install --packages-select ai_model_client
source install/setup.bash
```

## Commands
**Run Mock Server:**
```bash
python3 mock_server.py
```

**Run ROS 2 Node:**
```bash
ros2 run ai_model_client image_service --ros-args   -p server_url:="http://127.0.0.1:5000/predict"  -p image_topic:="/camera/image_raw"  -p service_name:="/trigger_ai_detection"
```

**Trigger Service:**
```bash
ros2 service call /trigger_ai_detection std_srvs/srv/Trigger {}
```

## Configuration
- \`server_url\`: HTTP Endpoint
- \`image_topic\`: Camera Topic
- \`service_name\`: ROS Service Name
