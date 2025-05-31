#!/bin/bash

# Web Robot Simulator Setup Script
echo "🚀 Setting up Web Robot Simulator..."

# Create directory structure
echo "📁 Creating directory structure..."
mkdir -p web/{js,css,models} bridge robots launch .colcon

# Set permissions
echo "🔐 Setting permissions..."
chmod +x robots/virtual_robot.py
chmod +x launch/robot_bringup.launch.py

# Create .env file if it doesn't exist
if [ ! -f .env ]; then
    echo "⚙️ Creating .env file..."
    cat > .env << EOF
# ROS Configuration
ROS_DISTRO=humble
ROS_DOMAIN_ID=42

# Display for GUI applications
DISPLAY=:0

# Network Configuration
ROBOT_IP=localhost
WEB_PORT=8080
WEBSOCKET_PORT=9090
EOF
fi

# Build Docker containers
echo "🐳 Building Docker containers..."
docker-compose build

echo "✅ Setup complete!"
echo ""
echo "🎯 To start the simulator:"
echo "   docker-compose up"
echo ""
echo "🌐 Then open your browser to:"
echo "   http://localhost:8080"
echo ""
echo "🔧 For troubleshooting, check:"
echo "   docker-compose logs"