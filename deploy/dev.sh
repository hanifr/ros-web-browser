#!/bin/bash

echo "🛠️ Starting Web Robot Simulator (Development)"

# Development environment setup
export COMPOSE_FILE=docker-compose.yml
export COMPOSE_PROJECT_NAME=robot_simulator_dev

# Enable file watching for development
export CHOKIDAR_USEPOLLING=true

# Start development environment
echo "🐳 Starting development environment..."
docker-compose up --build

echo "🎯 Development server started!"
echo "   Web interface: http://localhost:8080"
echo "   ROS bridge: ws://localhost:9090"
echo "   Hot reload enabled for web files"
echo ""
echo "Press Ctrl+C to stop"