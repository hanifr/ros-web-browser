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
│    └── virtual_robot.py
└── launch/
    └── robot_bringup.launch.py
```

# To deploy
```bash
docker-compose up --build
```

# To stop deployment
```bash
docker-compose down
```

# the file structure of web page
```bash
web
├── index.html                          # Main navigation page
├── css/
│   ├── style.css                       # Your existing corporate styles
│   ├── navigation.css                  # Navigation system styles
│   └── components.css                  # Component-specific styles
├── js/
│   ├── ros-bridge.js                   # Your existing ROS bridge
│   ├── three-robot.js                  # Your existing 3D robot
│   ├── robot-simulator.js              # Your existing simulator
│   ├── platform-core.js                # Platform integration script
│   ├── algorithm-comparison.js         # Algorithm comparison logic
│   ├── obstacle-avoidance.js           # Obstacle avoidance system
│   ├── analytics-engine.js             # Analytics and reporting
│   └── config-manager.js               # Configuration management
├── pages/
│   ├── control.html                    # Robot control interface
│   ├── comparison.html                 # Algorithm comparison
│   ├── obstacles.html                  # Obstacle avoidance
│   ├── analytics.html                  # Real-time analytics
│   └── config.html                     # Configuration management
└── assets/
    ├── presets/                        # Configuration presets
    ├── templates/                      # Excel/report templates
    └── documentation/                  # User guides
```

aneh2killa/ros2-web-based