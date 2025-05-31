// ROS Bridge Connection Handler
class ROSBridge {
    constructor(url) {
        this.url = url;
        this.ros = null;
        this.subscribers = {};
        this.publishers = {};
        this.isConnected = false;
        this.reconnectInterval = null;
        this.connectionAttempts = 0;
        this.maxReconnectAttempts = 10;
        
        this.connect();
    }
    
    connect() {
        console.log(`🔌 Attempting to connect to ROS bridge at ${this.url}...`);
        
        try {
            this.ros = new ROSLIB.Ros({
                url: this.url
            });
            
            this.ros.on('connection', () => {
                console.log('✅ Connected to ROS bridge');
                this.isConnected = true;
                this.connectionAttempts = 0;
                this.updateConnectionStatus('Connected', '#4CAF50');
                this.setupPublishers();
                this.clearReconnectInterval();
            });
            
            this.ros.on('error', (error) => {
                console.error('❌ ROS connection error:', error);
                this.isConnected = false;
                this.updateConnectionStatus('Error', '#f44336');
                this.scheduleReconnect();
            });
            
            this.ros.on('close', () => {
                console.log('🔌 Disconnected from ROS bridge');
                this.isConnected = false;
                this.updateConnectionStatus('Disconnected', '#ff9800');
                this.scheduleReconnect();
            });
            
        } catch (error) {
            console.error('❌ Failed to create ROS connection:', error);
            this.updateConnectionStatus('Failed', '#f44336');
            this.scheduleReconnect();
        }
    }
    
    scheduleReconnect() {
        if (this.reconnectInterval) return;
        
        if (this.connectionAttempts < this.maxReconnectAttempts) {
            const delay = Math.min(5000, 1000 * Math.pow(2, this.connectionAttempts));
            console.log(`🔄 Scheduling reconnect in ${delay}ms (attempt ${this.connectionAttempts + 1}/${this.maxReconnectAttempts})`);
            
            this.reconnectInterval = setTimeout(() => {
                this.connectionAttempts++;
                this.clearReconnectInterval();
                this.connect();
            }, delay);
        } else {
            console.error('❌ Max reconnection attempts reached');
            this.updateConnectionStatus('Failed - Max attempts reached', '#f44336');
        }
    }
    
    clearReconnectInterval() {
        if (this.reconnectInterval) {
            clearTimeout(this.reconnectInterval);
            this.reconnectInterval = null;
        }
    }
    
    reconnect() {
        this.clearReconnectInterval();
        this.connectionAttempts = 0;
        
        if (this.ros) {
            this.ros.close();
        }
        
        setTimeout(() => {
            this.connect();
        }, 1000);
    }
    
    updateConnectionStatus(status, color) {
        const statusElement = document.getElementById('connection-status');
        const statusDot = document.getElementById('status-dot');
        
        if (statusElement) {
            statusElement.textContent = status;
        }
        
        if (statusDot) {
            statusDot.style.backgroundColor = color;
            statusDot.classList.remove('pulse');
            
            if (status === 'Connected') {
                statusDot.classList.add('pulse');
            }
        }
        
        // Update WebSocket latency display
        this.updateLatencyDisplay();
    }
    
    updateLatencyDisplay() {
        // Simple latency measurement
        if (this.isConnected) {
            const startTime = Date.now();
            
            // Create a simple echo service call to measure latency
            const pingService = new ROSLIB.Service({
                ros: this.ros,
                name: '/rosapi/get_time',
                serviceType: 'rosapi/GetTime'
            });
            
            const request = new ROSLIB.ServiceRequest({});
            
            pingService.callService(request, (result) => {
                const latency = Date.now() - startTime;
                document.getElementById('ws-latency').textContent = `${latency} ms`;
            }, (error) => {
                document.getElementById('ws-latency').textContent = '-- ms';
            });
        } else {
            document.getElementById('ws-latency').textContent = '-- ms';
        }
    }
    
    setupPublishers() {
        if (!this.ros) return;
        
        console.log('📤 Setting up ROS publishers...');
        
        // Command velocity publisher
        this.publishers.cmdVel = new ROSLIB.Topic({
            ros: this.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
        
        // Robot mode publisher
        this.publishers.mode = new ROSLIB.Topic({
            ros: this.ros,
            name: '/robot_mode',
            messageType: 'std_msgs/String'
        });
        
        // Initial pose publisher (for reset)
        this.publishers.initialPose = new ROSLIB.Topic({
            ros: this.ros,
            name: '/initialpose',
            messageType: 'geometry_msgs/PoseWithCovarianceStamped'
        });
        
        console.log('✅ Publishers ready');
    }
    
    subscribe(topic, messageType, callback) {
        if (!this.ros) {
            console.warn(`⚠️ Cannot subscribe to ${topic}: ROS not connected`);
            return null;
        }
        
        console.log(`📥 Subscribing to ${topic} (${messageType})`);
        
        const subscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: topic,
            messageType: messageType,
            queue_length: 1,
            throttle_rate: 100  // Throttle to 10Hz for performance
        });
        
        subscriber.subscribe(callback);
        this.subscribers[topic] = subscriber;
        
        return subscriber;
    }
    
    unsubscribe(topic) {
        if (this.subscribers[topic]) {
            this.subscribers[topic].unsubscribe();
            delete this.subscribers[topic];
            console.log(`📥 Unsubscribed from ${topic}`);
        }
    }
    
    publishVelocity(linear, angular) {
        if (!this.publishers.cmdVel) {
            console.warn('⚠️ Cannot publish velocity: publisher not ready');
            return;
        }
        
        const twist = new ROSLIB.Message({
            linear: {
                x: linear,
                y: 0,
                z: 0
            },
            angular: {
                x: 0,
                y: 0,
                z: angular
            }
        });
        
        this.publishers.cmdVel.publish(twist);
    }
    
    setRobotMode(mode) {
        if (!this.publishers.mode) {
            console.warn('⚠️ Cannot set robot mode: publisher not ready');
            return;
        }
        
        const message = new ROSLIB.Message({
            data: mode
        });
        
        this.publishers.mode.publish(message);
        console.log(`🤖 Robot mode set to: ${mode}`);
    }
    
    resetPose(x = 0, y = 0, theta = 0) {
        if (!this.publishers.initialPose) {
            console.warn('⚠️ Cannot reset pose: publisher not ready');
            return;
        }
        
        const pose = new ROSLIB.Message({
            header: {
                stamp: {
                    sec: Math.floor(Date.now() / 1000),
                    nanosec: (Date.now() % 1000) * 1000000
                },
                frame_id: 'map'
            },
            pose: {
                pose: {
                    position: {
                        x: x,
                        y: y,
                        z: 0
                    },
                    orientation: {
                        x: 0,
                        y: 0,
                        z: Math.sin(theta / 2),
                        w: Math.cos(theta / 2)
                    }
                },
                covariance: new Array(36).fill(0)
            }
        });
        
        this.publishers.initialPose.publish(pose);
        console.log(`🔄 Reset robot pose to (${x}, ${y}, ${theta})`);
    }
    
    emergencyStop() {
        console.log('🛑 Emergency stop activated!');
        
        // Stop robot immediately
        this.publishVelocity(0, 0);
        
        // Set robot mode to stop
        this.setRobotMode('stop');
        
        // Visual feedback
        this.updateConnectionStatus('Emergency Stop', '#ff5722');
        
        // Reset status after 3 seconds
        setTimeout(() => {
            if (this.isConnected) {
                this.updateConnectionStatus('Connected', '#4CAF50');
            }
        }, 3000);
    }
    
    // Service call helper
    callService(serviceName, serviceType, request, successCallback, errorCallback) {
        if (!this.ros) {
            console.warn(`⚠️ Cannot call service ${serviceName}: ROS not connected`);
            if (errorCallback) errorCallback('ROS not connected');
            return;
        }
        
        const service = new ROSLIB.Service({
            ros: this.ros,
            name: serviceName,
            serviceType: serviceType
        });
        
        const serviceRequest = new ROSLIB.ServiceRequest(request);
        
        service.callService(serviceRequest, successCallback, errorCallback);
    }
    
    // Get list of available topics
    getTopics(callback) {
        this.callService('/rosapi/topics', 'rosapi/Topics', {}, 
            (result) => {
                console.log('📋 Available topics:', result.topics);
                if (callback) callback(result.topics);
            },
            (error) => {
                console.error('❌ Failed to get topics:', error);
            }
        );
    }
    
    // Get list of available services
    getServices(callback) {
        this.callService('/rosapi/services', 'rosapi/Services', {},
            (result) => {
                console.log('🔧 Available services:', result.services);
                if (callback) callback(result.services);
            },
            (error) => {
                console.error('❌ Failed to get services:', error);
            }
        );
    }
    
    // Clean up on page unload
    cleanup() {
        console.log('🧹 Cleaning up ROS connection...');
        
        this.clearReconnectInterval();
        
        // Unsubscribe from all topics
        Object.keys(this.subscribers).forEach(topic => {
            this.unsubscribe(topic);
        });
        
        // Close ROS connection
        if (this.ros) {
            this.ros.close();
        }
    }
}

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (window.rosConnection) {
        window.rosConnection.cleanup();
    }
});