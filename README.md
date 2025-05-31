# ros-web-browser
A simple ROS platform using web page

# File Structure

```bash
web-robot-simulator/
├── docker-compose.yml
├── Dockerfile.web-bridge
├── web/
│   ├── index.html
│   ├── js/
│   │   ├── robot-simulator.js
│   │   ├── ros-bridge.js
│   │   └── three-robot.js
│   ├── css/
│   │   └── style.css
│   └── models/
│       └── robot.urdf
├── bridge/
│   └── rosbridge_config.yaml
└── robots/
    └── virtual_robot.py
```