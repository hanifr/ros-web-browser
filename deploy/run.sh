#!/bin/bash

echo "🚀 Starting Web Robot Simulator (Production)"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null; then
    echo "❌ docker-compose not found. Please install docker-compose."
    exit 1
fi

# Create necessary directories
mkdir -p deploy/ssl
mkdir -p logs

# Set permissions
chmod +x robots/*.py
chmod +x tools/*.py
chmod +x launch/*.py

# Copy environment configuration
if [ ! -f .env ]; then
    echo "⚙️ Creating default .env file..."
    cp .env.example .env
fi

# Build and start services
echo "🐳 Building Docker containers..."
docker-compose -f deploy/docker-compose.prod.yml build

echo "🚀 Starting services..."
docker-compose -f deploy/docker-compose.prod.yml up -d

# Wait for services to be ready
echo "⏳ Waiting for services to start..."
sleep 10

# Check service health
echo "🏥 Checking service health..."
if curl -s http://localhost/health > /dev/null; then
    echo "✅ Web interface is ready at http://localhost"
else
    echo "⚠️ Web interface may not be ready yet. Check logs with: docker-compose -f deploy/docker-compose.prod.yml logs"
fi

if curl -s http://localhost:9090 > /dev/null; then
    echo "✅ ROS bridge is ready at ws://localhost:9090"
else
    echo "⚠️ ROS bridge may not be ready yet. Check logs with: docker-compose -f deploy/docker-compose.prod.yml logs ros2-web-bridge-prod"
fi

echo "📊 Monitoring dashboard available at http://localhost:3000 (admin/admin)"
echo ""
echo "🎯 To stop the system:"
echo "   docker-compose -f deploy/docker-compose.prod.yml down"
echo ""
echo "📋 To view logs:"
echo "   docker-compose -f deploy/docker-compose.prod.yml logs -f"