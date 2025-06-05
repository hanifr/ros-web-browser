/**
 * Obstacle Avoidance System - Real-time obstacle detection, mapping, and avoidance
 * Provides multiple avoidance algorithms and safety systems for autonomous navigation
 */

class ObstacleAvoidance {
    constructor() {
        // System state
        this.isEnabled = false;
        this.isActive = false;
        this.currentAlgorithm = 'POTENTIAL_FIELDS'; // POTENTIAL_FIELDS, DWA, VECTOR_FIELD, EMERGENCY_STOP
        
        // Obstacle detection
        this.obstacles = new Map();
        this.obstacleHistory = [];
        this.detectionRange = 3.0; // meters
        this.safetyRadius = 0.5; // meters around robot
        this.emergencyStopDistance = 0.3; // meters
        
        // Sensor data
        this.sensorData = {
            lidar: null,
            camera: null,
            ultrasonic: null,
            lastUpdate: 0
        };
        
        // Avoidance algorithms
        this.algorithms = {
            potentialFields: new PotentialFieldsAvoidance(),
            dwa: new DynamicWindowApproach(),
            vectorField: new VectorFieldHistogram(),
            emergencyStop: new EmergencyStopSystem()
        };
        
        // Path planning
        this.pathPlanner = new LocalPathPlanner();
        this.currentPath = [];
        this.pathUpdateInterval = null;
        
        // Performance metrics
        this.performance = {
            detectionRate: 0,
            avoidanceRate: 0,
            obstaclesDetected: 0,
            collisionsAvoided: 0,
            emergencyStops: 0,
            processingTime: 0
        };
        
        // Configuration
        this.config = {
            enableLidarDetection: true,
            enableCameraDetection: false,
            enableUltrasonicDetection: false,
            obstacleTimeout: 5000, // ms
            updateRate: 20, // Hz
            emergencyStopEnabled: true,
            debugVisualization: true
        };
        
        // Safety zones
        this.safetyZones = [
            { name: 'critical', radius: 0.3, action: 'EMERGENCY_STOP' },
            { name: 'warning', radius: 0.6, action: 'AVOIDANCE' },
            { name: 'detection', radius: 1.5, action: 'MONITOR' }
        ];
        
        this.initialize();
    }
    
    /**
     * Initialize obstacle avoidance system
     */
    initialize() {
        console.log('🛡️ Initializing Obstacle Avoidance System...');
        
        try {
            // Load configuration
            this.loadConfiguration();
            
            // Initialize algorithms
            this.initializeAlgorithms();
            
            // Setup event listeners
            this.setupEventListeners();
            
            // Initialize path planner
            this.pathPlanner.initialize();
            
            // Start update loop if enabled
            if (ConfigManager.get('safety.obstacleDetectionEnabled', false)) {
                this.enable();
            }
            
            console.log('✅ Obstacle Avoidance System initialized');
            
        } catch (error) {
            console.error('❌ Obstacle Avoidance initialization failed:', error);
        }
    }
    
    /**
     * Load configuration from ConfigManager
     */
    loadConfiguration() {
        this.config.enableLidarDetection = ConfigManager.get('safety.lidarDetection', true);
        this.config.enableCameraDetection = ConfigManager.get('safety.cameraDetection', false);
        this.config.emergencyStopEnabled = ConfigManager.get('safety.emergencyStopEnabled', true);
        this.config.updateRate = ConfigManager.get('safety.updateRate', 20);
        this.detectionRange = ConfigManager.get('safety.detectionRange', 3.0);
        this.safetyRadius = ConfigManager.get('safety.safetyRadius', 0.5);
        this.emergencyStopDistance = ConfigManager.get('safety.emergencyStopDistance', 0.3);
    }
    
    /**
     * Initialize avoidance algorithms
     */
    initializeAlgorithms() {
        console.log('Initializing avoidance algorithms...');
        
        // Initialize each algorithm with configuration
        Object.values(this.algorithms).forEach(algorithm => {
            if (algorithm.initialize) {
                algorithm.initialize(this.config);
            }
        });
        
        console.log('Avoidance algorithms initialized');
    }
    
    /**
     * Setup event listeners
     */
    setupEventListeners() {
        // Listen for sensor updates
        window.addEventListener('laserScanUpdate', (event) => {
            this.processSensorData('lidar', event.detail);
        });
        
        window.addEventListener('robotStateUpdate', (event) => {
            this.updateRobotState(event.detail);
        });
        
        // Listen for navigation events
        window.addEventListener('navigationStarted', (event) => {
            this.onNavigationStarted(event.detail);
        });
        
        window.addEventListener('navigationStopped', () => {
            this.onNavigationStopped();
        });
        
        // Listen for system events
        window.addEventListener('systemStarted', () => {
            if (this.config.enableOnSystemStart) {
                this.enable();
            }
        });
        
        window.addEventListener('emergencyStop', () => {
            this.onEmergencyStop();
        });
        
        // Configuration changes
        ConfigManager.onChange('safety', () => {
            this.loadConfiguration();
        });
    }
    
    /**
     * Enable obstacle avoidance system
     */
    enable() {
        if (this.isEnabled) {
            console.warn('Obstacle avoidance already enabled');
            return;
        }
        
        this.isEnabled = true;
        console.log('🛡️ Obstacle avoidance system enabled');
        
        // Start update loop
        this.startUpdateLoop();
        
        // Emit enabled event
        window.dispatchEvent(new CustomEvent('obstacleAvoidanceEnabled'));
    }
    
    /**
     * Disable obstacle avoidance system
     */
    disable() {
        if (!this.isEnabled) {
            console.warn('Obstacle avoidance already disabled');
            return;
        }
        
        this.isEnabled = false;
        this.isActive = false;
        console.log('🚫 Obstacle avoidance system disabled');
        
        // Stop update loop
        this.stopUpdateLoop();
        
        // Clear obstacles
        this.clearObstacles();
        
        // Emit disabled event
        window.dispatchEvent(new CustomEvent('obstacleAvoidanceDisabled'));
    }
    
    /**
     * Start main update loop
     */
    startUpdateLoop() {
        if (this.updateInterval) return;
        
        const updateRate = 1000 / this.config.updateRate; // Convert Hz to ms
        
        this.updateInterval = setInterval(() => {
            this.update();
        }, updateRate);
        
        console.log(`Update loop started at ${this.config.updateRate}Hz`);
    }
    
    /**
     * Stop update loop
     */
    stopUpdateLoop() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    }
    
    /**
     * Main update function
     */
    update() {
        if (!this.isEnabled) return;
        
        const startTime = performance.now();
        
        try {
            // Process sensor data
            this.processSensorUpdates();
            
            // Detect obstacles
            this.detectObstacles();
            
            // Update obstacle tracking
            this.updateObstacleTracking();
            
            // Check safety zones
            const safetyStatus = this.checkSafetyZones();
            
            // Apply avoidance if needed
            if (safetyStatus.requiresAvoidance) {
                this.applyObstacleAvoidance(safetyStatus);
            }
            
            // Update performance metrics
            this.updatePerformanceMetrics(startTime);
            
            // Update visualization
            if (this.config.debugVisualization) {
                this.updateVisualization();
            }
            
        } catch (error) {
            console.error('Error in obstacle avoidance update:', error);
        }
    }
    
    /**
     * Process sensor data updates
     * @param {string} sensorType - Type of sensor
     * @param {object} data - Sensor data
     */
    processSensorData(sensorType, data) {
        this.sensorData[sensorType] = data;
        this.sensorData.lastUpdate = Date.now();
        
        // Process immediately if system is active
        if (this.isEnabled && this.isActive) {
            this.detectObstaclesFromSensor(sensorType, data);
        }
    }
    
    /**
     * Process all sensor updates
     */
    processSensorUpdates() {
        const now = Date.now();
        const sensorTimeout = 1000; // 1 second timeout
        
        // Check if sensor data is stale
        if (now - this.sensorData.lastUpdate > sensorTimeout) {
            console.warn('Sensor data timeout - obstacle detection may be impaired');
            return;
        }
        
        // Process each active sensor
        if (this.config.enableLidarDetection && this.sensorData.lidar) {
            this.detectObstaclesFromSensor('lidar', this.sensorData.lidar);
        }
        
        if (this.config.enableCameraDetection && this.sensorData.camera) {
            this.detectObstaclesFromSensor('camera', this.sensorData.camera);
        }
        
        if (this.config.enableUltrasonicDetection && this.sensorData.ultrasonic) {
            this.detectObstaclesFromSensor('ultrasonic', this.sensorData.ultrasonic);
        }
    }
    
    /**
     * Detect obstacles from sensor data
     * @param {string} sensorType - Sensor type
     * @param {object} data - Sensor data
     */
    detectObstaclesFromSensor(sensorType, data) {
        switch (sensorType) {
            case 'lidar':
                this.detectObstaclesFromLidar(data);
                break;
            case 'camera':
                this.detectObstaclesFromCamera(data);
                break;
            case 'ultrasonic':
                this.detectObstaclesFromUltrasonic(data);
                break;
        }
    }
    
    /**
     * Detect obstacles from LiDAR data
     * @param {object} lidarData - LiDAR scan data
     */
    detectObstaclesFromLidar(lidarData) {
        if (!lidarData || !lidarData.ranges) return;
        
        const robotState = this.getRobotState();
        const obstacles = [];
        
        for (let i = 0; i < lidarData.ranges.length; i++) {
            const range = lidarData.ranges[i];
            const angle = lidarData.angleMin + i * lidarData.angleIncrement;
            
            // Filter valid ranges
            if (range < lidarData.rangeMin || range > lidarData.rangeMax) continue;
            if (range > this.detectionRange) continue;
            
            // Convert to global coordinates
            const globalAngle = robotState.orientation + angle;
            const obstacleX = robotState.position.x + range * Math.cos(globalAngle);
            const obstacleY = robotState.position.y + range * Math.sin(globalAngle);
            
            // Create obstacle object
            const obstacle = {
                id: `lidar_${i}_${Date.now()}`,
                type: 'point',
                sensor: 'lidar',
                position: { x: obstacleX, y: obstacleY },
                distance: range,
                angle: globalAngle,
                confidence: this.calculateLidarConfidence(range, i),
                timestamp: Date.now(),
                size: 0.1 // Estimated size for point obstacles
            };
            
            obstacles.push(obstacle);
        }
        
        // Cluster nearby points into larger obstacles
        const clusteredObstacles = this.clusterObstacles(obstacles);
        
        // Add to obstacle map
        clusteredObstacles.forEach(obstacle => {
            this.addObstacle(obstacle);
        });
    }
    
    /**
     * Detect obstacles from camera data (placeholder)
     * @param {object} cameraData - Camera data
     */
    detectObstaclesFromCamera(cameraData) {
        // Placeholder for computer vision obstacle detection
        console.log('Camera-based obstacle detection not yet implemented');
    }
    
    /**
     * Detect obstacles from ultrasonic sensors (placeholder)
     * @param {object} ultrasonicData - Ultrasonic sensor data
     */
    detectObstaclesFromUltrasonic(ultrasonicData) {
        // Placeholder for ultrasonic obstacle detection
        console.log('Ultrasonic obstacle detection not yet implemented');
    }
    
    /**
     * Calculate confidence score for LiDAR detection
     * @param {number} range - Range measurement
     * @param {number} index - Beam index
     * @returns {number} Confidence score (0-1)
     */
    calculateLidarConfidence(range, index) {
        // Higher confidence for closer objects and central beams
        const distanceConfidence = Math.max(0, 1 - range / this.detectionRange);
        const angleConfidence = Math.cos(index * 0.01); // Simplified angular confidence
        return Math.min(1, distanceConfidence * angleConfidence);
    }
    
    /**
     * Cluster nearby obstacle points into larger obstacles
     * @param {array} obstacles - Array of point obstacles
     * @returns {array} Clustered obstacles
     */
    clusterObstacles(obstacles) {
        const clusterDistance = 0.3; // meters
        const clustered = [];
        const processed = new Set();
        
        obstacles.forEach((obstacle, index) => {
            if (processed.has(index)) return;
            
            const cluster = [obstacle];
            processed.add(index);
            
            // Find nearby obstacles
            obstacles.forEach((other, otherIndex) => {
                if (processed.has(otherIndex) || index === otherIndex) return;
                
                const distance = Math.sqrt(
                    Math.pow(obstacle.position.x - other.position.x, 2) +
                    Math.pow(obstacle.position.y - other.position.y, 2)
                );
                
                if (distance < clusterDistance) {
                    cluster.push(other);
                    processed.add(otherIndex);
                }
            });
            
            // Create clustered obstacle
            if (cluster.length > 1) {
                const clusterObstacle = this.createClusteredObstacle(cluster);
                clustered.push(clusterObstacle);
            } else {
                clustered.push(obstacle);
            }
        });
        
        return clustered;
    }
    
    /**
     * Create clustered obstacle from multiple points
     * @param {array} cluster - Array of obstacle points
     * @returns {object} Clustered obstacle
     */
    createClusteredObstacle(cluster) {
        // Calculate centroid
        const centroidX = cluster.reduce((sum, obs) => sum + obs.position.x, 0) / cluster.length;
        const centroidY = cluster.reduce((sum, obs) => sum + obs.position.y, 0) / cluster.length;
        
        // Calculate size
        const distances = cluster.map(obs => 
            Math.sqrt(
                Math.pow(obs.position.x - centroidX, 2) + 
                Math.pow(obs.position.y - centroidY, 2)
            )
        );
        const size = Math.max(0.2, Math.max(...distances) * 2);
        
        // Calculate average confidence
        const avgConfidence = cluster.reduce((sum, obs) => sum + obs.confidence, 0) / cluster.length;
        
        return {
            id: `cluster_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            type: 'cluster',
            sensor: 'lidar',
            position: { x: centroidX, y: centroidY },
            distance: Math.sqrt(centroidX * centroidX + centroidY * centroidY),
            confidence: avgConfidence,
            timestamp: Date.now(),
            size: size,
            pointCount: cluster.length
        };
    }
    
    /**
     * Add obstacle to tracking system
     * @param {object} obstacle - Obstacle object
     */
    addObstacle(obstacle) {
        this.obstacles.set(obstacle.id, obstacle);
        this.performance.obstaclesDetected++;
        
        // Add to history
        this.obstacleHistory.push({
            ...obstacle,
            action: 'detected'
        });
        
        // Limit history size
        if (this.obstacleHistory.length > 1000) {
            this.obstacleHistory.shift();
        }
        
        // Emit obstacle detected event
        window.dispatchEvent(new CustomEvent('obstacleDetected', {
            detail: obstacle
        }));
        
        // Add to 3D visualization if available
        if (window.ThreeRobotVisualization) {
            window.ThreeRobotVisualization.addObstacle({
                id: obstacle.id,
                type: 'cylinder',
                x: obstacle.position.x,
                y: obstacle.position.y,
                radius: obstacle.size / 2,
                height: 0.5,
                color: 0xff4444
            });
        }
    }
    
    /**
     * Update obstacle tracking and remove stale obstacles
     */
    updateObstacleTracking() {
        const now = Date.now();
        const timeout = this.config.obstacleTimeout;
        
        // Remove stale obstacles
        const staleObstacles = [];
        this.obstacles.forEach((obstacle, id) => {
            if (now - obstacle.timestamp > timeout) {
                staleObstacles.push(id);
            }
        });
        
        staleObstacles.forEach(id => {
            this.removeObstacle(id);
        });
    }
    
    /**
     * Remove obstacle from tracking
     * @param {string} obstacleId - Obstacle ID
     */
    removeObstacle(obstacleId) {
        const obstacle = this.obstacles.get(obstacleId);
        if (obstacle) {
            this.obstacles.delete(obstacleId);
            
            // Add to history
            this.obstacleHistory.push({
                ...obstacle,
                action: 'removed',
                timestamp: Date.now()
            });
            
            // Remove from 3D visualization
            if (window.ThreeRobotVisualization) {
                window.ThreeRobotVisualization.removeObstacle(obstacleId);
            }
            
            // Emit obstacle removed event
            window.dispatchEvent(new CustomEvent('obstacleRemoved', {
                detail: { id: obstacleId, obstacle }
            }));
        }
    }
    
    /**
     * Check safety zones around robot
     * @returns {object} Safety status
     */
    checkSafetyZones() {
        const robotState = this.getRobotState();
        const status = {
            level: 'SAFE',
            requiresAvoidance: false,
            closestObstacle: null,
            minDistance: Infinity,
            action: 'NONE'
        };
        
        // Check each obstacle against safety zones
        this.obstacles.forEach(obstacle => {
            const distance = Math.sqrt(
                Math.pow(obstacle.position.x - robotState.position.x, 2) +
                Math.pow(obstacle.position.y - robotState.position.y, 2)
            ) - obstacle.size / 2; // Account for obstacle size
            
            if (distance < status.minDistance) {
                status.minDistance = distance;
                status.closestObstacle = obstacle;
            }
            
            // Check against safety zones
            for (const zone of this.safetyZones) {
                if (distance < zone.radius) {
                    if (this.getSeverityLevel(zone.action) > this.getSeverityLevel(status.action)) {
                        status.level = zone.name.toUpperCase();
                        status.action = zone.action;
                        status.requiresAvoidance = true;
                    }
                }
            }
        });
        
        return status;
    }
    
    /**
     * Get severity level for action
     * @param {string} action - Action type
     * @returns {number} Severity level
     */
    getSeverityLevel(action) {
        const levels = {
            'NONE': 0,
            'MONITOR': 1,
            'AVOIDANCE': 2,
            'EMERGENCY_STOP': 3
        };
        return levels[action] || 0;
    }
    
    /**
     * Apply obstacle avoidance
     * @param {object} safetyStatus - Current safety status
     */
    applyObstacleAvoidance(safetyStatus) {
        this.isActive = true;
        this.performance.avoidanceRate++;
        
        switch (safetyStatus.action) {
            case 'EMERGENCY_STOP':
                this.executeEmergencyStop(safetyStatus);
                break;
            case 'AVOIDANCE':
                this.executeAvoidanceManeuver(safetyStatus);
                break;
            case 'MONITOR':
                this.monitorSituation(safetyStatus);
                break;
        }
        
        // Emit avoidance event
        window.dispatchEvent(new CustomEvent('obstacleAvoidanceActive', {
            detail: safetyStatus
        }));
    }
    
    /**
     * Execute emergency stop
     * @param {object} safetyStatus - Safety status
     */
    executeEmergencyStop(safetyStatus) {
        console.warn('🛑 EMERGENCY STOP: Obstacle too close!');
        
        this.performance.emergencyStops++;
        
        // Stop robot immediately
        if (window.ROSBridge) {
            window.ROSBridge.stopRobot();
        }
        
        // Pause system
        if (window.PlatformCore) {
            window.PlatformCore.pauseSystem();
        }
        
        // Log emergency stop
        this.logMessage(`🛑 Emergency stop triggered - obstacle at ${safetyStatus.minDistance.toFixed(2)}m`);
        
        // Emit emergency stop event
        window.dispatchEvent(new CustomEvent('emergencyStop', {
            detail: {
                reason: 'obstacle_too_close',
                distance: safetyStatus.minDistance,
                obstacle: safetyStatus.closestObstacle
            }
        }));
    }
    
    /**
     * Execute avoidance maneuver
     * @param {object} safetyStatus - Safety status
     */
    executeAvoidanceManeuver(safetyStatus) {
        const algorithm = this.algorithms[this.getAlgorithmKey(this.currentAlgorithm)];
        
        if (!algorithm) {
            console.error('Avoidance algorithm not available');
            return;
        }
        
        const robotState = this.getRobotState();
        const obstacles = Array.from(this.obstacles.values());
        
        // Calculate avoidance velocity
        const avoidanceVelocity = algorithm.calculateAvoidanceVelocity(
            robotState,
            obstacles,
            this.getTargetPosition()
        );
        
        if (avoidanceVelocity) {
            // Apply velocity modification
            this.applyVelocityModification(avoidanceVelocity);
            
            this.performance.collisionsAvoided++;
            this.logMessage(`🛡️ Obstacle avoidance active - ${this.currentAlgorithm}`);
        }
    }
    
    /**
     * Monitor situation without active intervention
     * @param {object} safetyStatus - Safety status
     */
    monitorSituation(safetyStatus) {
        // Just monitor and log
        this.logMessage(`👁️ Monitoring obstacle at ${safetyStatus.minDistance.toFixed(2)}m`);
    }
    
    /**
     * Apply velocity modification for avoidance
     * @param {object} velocity - Modified velocity commands
     */
    applyVelocityModification(velocity) {
        // Override or modify current velocity commands
        if (window.ROSBridge && velocity) {
            window.ROSBridge.publishVelocity(velocity.linear, velocity.angular);
        }
    }
    
    /**
     * Get algorithm key from algorithm name
     * @param {string} algorithmName - Algorithm name
     * @returns {string} Algorithm key
     */
    getAlgorithmKey(algorithmName) {
        const keyMap = {
            'POTENTIAL_FIELDS': 'potentialFields',
            'DWA': 'dwa',
            'VECTOR_FIELD': 'vectorField',
            'EMERGENCY_STOP': 'emergencyStop'
        };
        return keyMap[algorithmName] || 'potentialFields';
    }
    
    /**
     * Get current robot state
     * @returns {object} Robot state
     */
    getRobotState() {
        if (window.PlatformCore) {
            const systemState = window.PlatformCore.getSystemState();
            return {
                position: systemState.robotPosition,
                orientation: systemState.robotOrientation,
                velocity: systemState.robotVelocity || { linear: { x: 0, y: 0 }, angular: { z: 0 } }
            };
        }
        
        return {
            position: { x: 0, y: 0 },
            orientation: 0,
            velocity: { linear: { x: 0, y: 0 }, angular: { z: 0 } }
        };
    }
    
    /**
     * Get current target position
     * @returns {object|null} Target position
     */
    getTargetPosition() {
        if (window.PlatformCore) {
            const systemState = window.PlatformCore.getSystemState();
            if (systemState.currentTarget && window.ConfigManager) {
                return window.ConfigManager.getLocation(systemState.currentTarget);
            }
        }
        return null;
    }
    
    /**
     * Update performance metrics
     * @param {number} startTime - Processing start time
     */
    updatePerformanceMetrics(startTime) {
        this.performance.processingTime = performance.now() - startTime;
        this.performance.detectionRate++;
    }
    
    /**
     * Update visualization
     */
    updateVisualization() {
        // Update obstacle visualization in 3D view
        // This is handled in the addObstacle/removeObstacle methods
        
        // Could add additional UI updates here
        this.updateObstacleUI();
    }
    
    /**
     * Update obstacle UI elements
     */
    updateObstacleUI() {
        // Update any UI elements showing obstacle information
        const obstacleCount = this.obstacles.size;
        const isActive = this.isActive;
        
        // Emit UI update event
        window.dispatchEvent(new CustomEvent('obstacleUIUpdate', {
            detail: {
                obstacleCount,
                isActive,
                performance: this.performance
            }
        }));
    }
    
    /**
     * Clear all obstacles
     */
    clearObstacles() {
        this.obstacles.forEach((_, id) => {
            this.removeObstacle(id);
        });
        this.obstacles.clear();
        console.log('All obstacles cleared');
    }
    
    /**
     * Set avoidance algorithm
     * @param {string} algorithm - Algorithm name
     */
    setAvoidanceAlgorithm(algorithm) {
        if (this.algorithms[this.getAlgorithmKey(algorithm)]) {
            this.currentAlgorithm = algorithm;
            console.log(`Avoidance algorithm changed to: ${algorithm}`);
        } else {
            console.error(`Unknown avoidance algorithm: ${algorithm}`);
        }
    }
    
    /**
     * Get obstacle avoidance status
     * @returns {object} Status information
     */
    getStatus() {
        return {
            isEnabled: this.isEnabled,
            isActive: this.isActive,
            currentAlgorithm: this.currentAlgorithm,
            obstacleCount: this.obstacles.size,
            performance: { ...this.performance },
            safetyZones: this.safetyZones,
            config: { ...this.config }
        };
    }
    
    /**
     * Get detected obstacles
     * @returns {array} Array of obstacles
     */
    getObstacles() {
        return Array.from(this.obstacles.values());
    }
    
    /**
     * Event handlers
     */
    updateRobotState(robotState) {
        this.robotState = robotState;
    }
    
    onNavigationStarted(navigationData) {
        this.isActive = this.isEnabled;
        console.log('Navigation started - obstacle avoidance activated');
    }
    
    onNavigationStopped() {
        this.isActive = false;
        console.log('Navigation stopped - obstacle avoidance deactivated');
    }
    
    onEmergencyStop() {
        this.isActive = false;
        this.clearObstacles();
    }
    
    /**
     * Log message to system
     * @param {string} message - Message to log
     */
    logMessage(message) {
        if (window.PlatformCore && window.PlatformCore.logMessage) {
            window.PlatformCore.logMessage(message);
        } else {
            console.log(message);
        }
    }
    
    /**
     * Shutdown obstacle avoidance system
     */
    shutdown() {
        console.log('🛑 Shutting down obstacle avoidance system');
        
        this.disable();
        this.clearObstacles();
        
        console.log('✅ Obstacle avoidance system shutdown completed');
    }
}

/**
 * Potential Fields Avoidance Algorithm
 */
class PotentialFieldsAvoidance {
    constructor() {
        this.attractiveGain = 1.0;
        this.repulsiveGain = 2.0;
        this.repulsiveRange = 1.5;
    }
    
    initialize(config) {
        this.attractiveGain = config.attractiveGain || 1.0;
        this.repulsiveGain = config.repulsiveGain || 2.0;
        this.repulsiveRange = config.repulsiveRange || 1.5;
    }
    
    calculateAvoidanceVelocity(robotState, obstacles, target) {
        if (!target) return null;
        
        let attractiveForceX = 0;
        let attractiveForceY = 0;
        let repulsiveForceX = 0;
        let repulsiveForceY = 0;
        
        // Calculate attractive force to target
        const targetDx = target.x - robotState.position.x;
        const targetDy = target.y - robotState.position.y;
        const targetDistance = Math.sqrt(targetDx * targetDx + targetDy * targetDy);
        
        if (targetDistance > 0.1) {
            attractiveForceX = this.attractiveGain * targetDx / targetDistance;
            attractiveForceY = this.attractiveGain * targetDy / targetDistance;
        }
        
        // Calculate repulsive forces from obstacles
        obstacles.forEach(obstacle => {
            const obstacleDx = robotState.position.x - obstacle.position.x;
            const obstacleDy = robotState.position.y - obstacle.position.y;
            const obstacleDistance = Math.sqrt(obstacleDx * obstacleDx + obstacleDy * obstacleDy);
            
            if (obstacleDistance < this.repulsiveRange && obstacleDistance > 0.01) {
                const repulsiveMagnitude = this.repulsiveGain * 
                    (1 / obstacleDistance - 1 / this.repulsiveRange) / 
                    (obstacleDistance * obstacleDistance);
                
                repulsiveForceX += repulsiveMagnitude * obstacleDx / obstacleDistance;
                repulsiveForceY += repulsiveMagnitude * obstacleDy / obstacleDistance;
            }
        });
        
        // Combine forces
        const totalForceX = attractiveForceX + repulsiveForceX;
        const totalForceY = attractiveForceY + repulsiveForceY;
        
        // Convert to robot local frame
        const cos_theta = Math.cos(robotState.orientation);
        const sin_theta = Math.sin(robotState.orientation);
        
        const localForceX = cos_theta * totalForceX + sin_theta * totalForceY;
        const localForceY = -sin_theta * totalForceX + cos_theta * totalForceY;
        
        // Convert to velocity commands
        const maxLinearVel = 0.5;
        const maxAngularVel = 1.0;
        
        const linear = Math.max(Math.min(localForceX, maxLinearVel), -maxLinearVel);
        const angular = Math.max(Math.min(localForceY, maxAngularVel), -maxAngularVel);
        
        return { linear, angular };
    }
}

/**
 * Dynamic Window Approach Algorithm
 */
class DynamicWindowApproach {
    constructor() {
        this.maxLinearAccel = 0.5;
        this.maxAngularAccel = 1.0;
        this.timeHorizon = 2.0;
        this.velocityResolution = 0.1;
    }
    
    initialize(config) {
        this.maxLinearAccel = config.maxLinearAccel || 0.5;
        this.maxAngularAccel = config.maxAngularAccel || 1.0;
        this.timeHorizon = config.timeHorizon || 2.0;
    }
    
    calculateAvoidanceVelocity(robotState, obstacles, target) {
        // Simplified DWA implementation
        // In a full implementation, this would search through the dynamic window
        // of achievable velocities and find the optimal one
        
        const currentLinear = robotState.velocity.linear.x;
        const currentAngular = robotState.velocity.angular.z;
        
        // Simple obstacle avoidance by reducing speed
        let avoidanceLinear = currentLinear;
        let avoidanceAngular = currentAngular;
        
        const closestObstacle = this.findClosestObstacle(robotState, obstacles);
        if (closestObstacle && closestObstacle.distance < 1.0) {
            avoidanceLinear *= 0.5; // Reduce speed
            
            // Steer away from obstacle
            const obstacleAngle = Math.atan2(
                closestObstacle.position.y - robotState.position.y,
                closestObstacle.position.x - robotState.position.x
            );
            const relativeAngle = obstacleAngle - robotState.orientation;
            
            if (Math.abs(relativeAngle) < Math.PI / 2) {
                avoidanceAngular += relativeAngle > 0 ? -0.5 : 0.5;
            }
        }
        
        return { linear: avoidanceLinear, angular: avoidanceAngular };
    }
    
    findClosestObstacle(robotState, obstacles) {
        let closest = null;
        let minDistance = Infinity;
        
        obstacles.forEach(obstacle => {
            const distance = Math.sqrt(
                Math.pow(obstacle.position.x - robotState.position.x, 2) +
                Math.pow(obstacle.position.y - robotState.position.y, 2)
            );
            
            if (distance < minDistance) {
                minDistance = distance;
                closest = { ...obstacle, distance };
            }
        });
        
        return closest;
    }
}

/**
 * Vector Field Histogram Algorithm
 */
class VectorFieldHistogram {
    constructor() {
        this.histogramResolution = 5; // degrees
        this.threshold = 0.5;
    }
    
    initialize(config) {
        this.histogramResolution = config.histogramResolution || 5;
        this.threshold = config.threshold || 0.5;
    }
    
    calculateAvoidanceVelocity(robotState, obstacles, target) {
        // Simplified VFH implementation
        // Creates a polar histogram and finds free directions
        
        const histogram = this.createPolarHistogram(robotState, obstacles);
        const freeDirections = this.findFreeDirections(histogram);
        
        if (freeDirections.length === 0) {
            return { linear: 0, angular: 0 }; // Stop if no free directions
        }
        
        // Choose direction closest to target
        let bestDirection = freeDirections[0];
        let minAngleDiff = Infinity;
        
        if (target) {
            const targetAngle = Math.atan2(
                target.y - robotState.position.y,
                target.x - robotState.position.x
            );
            
            freeDirections.forEach(direction => {
                const angleDiff = Math.abs(direction - targetAngle);
                if (angleDiff < minAngleDiff) {
                    minAngleDiff = angleDiff;
                    bestDirection = direction;
                }
            });
        }
        
        // Convert to velocity commands
        const headingError = bestDirection - robotState.orientation;
        const normalizedError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
        
        return {
            linear: 0.3,
            angular: normalizedError * 2.0
        };
    }
    
    createPolarHistogram(robotState, obstacles) {
        const histogram = new Array(360 / this.histogramResolution).fill(0);
        
        obstacles.forEach(obstacle => {
            const angle = Math.atan2(
                obstacle.position.y - robotState.position.y,
                obstacle.position.x - robotState.position.x
            ) * 180 / Math.PI;
            
            const distance = Math.sqrt(
                Math.pow(obstacle.position.x - robotState.position.x, 2) +
                Math.pow(obstacle.position.y - robotState.position.y, 2)
            );
            
            const index = Math.floor((angle + 180) / this.histogramResolution);
            if (index >= 0 && index < histogram.length) {
                histogram[index] += 1 / (distance + 0.1); // Inverse distance weighting
            }
        });
        
        return histogram;
    }
    
    findFreeDirections(histogram) {
        const freeDirections = [];
        
        histogram.forEach((value, index) => {
            if (value < this.threshold) {
                const angle = (index * this.histogramResolution - 180) * Math.PI / 180;
                freeDirections.push(angle);
            }
        });
        
        return freeDirections;
    }
}

/**
 * Emergency Stop System
 */
class EmergencyStopSystem {
    initialize(config) {
        this.emergencyDistance = config.emergencyStopDistance || 0.3;
    }
    
    calculateAvoidanceVelocity(robotState, obstacles, target) {
        // Always returns stop command
        return { linear: 0, angular: 0 };
    }
}

/**
 * Local Path Planner for obstacle avoidance
 */
class LocalPathPlanner {
    constructor() {
        this.planningHorizon = 3.0; // meters
        this.pathResolution = 0.1; // meters
        this.maxPlanningTime = 50; // milliseconds
    }
    
    initialize() {
        console.log('Local path planner initialized');
    }
    
    planPath(start, goal, obstacles) {
        // Simplified local path planning
        // In a full implementation, this would use A*, RRT, or similar algorithms
        
        const path = [start, goal];
        
        // Simple obstacle checking
        const hasObstacles = this.checkPathForObstacles(path, obstacles);
        
        if (hasObstacles) {
            // Generate alternative path
            const alternativePath = this.generateAlternativePath(start, goal, obstacles);
            return alternativePath;
        }
        
        return path;
    }
    
    checkPathForObstacles(path, obstacles) {
        // Check if path intersects with any obstacles
        for (let i = 0; i < path.length - 1; i++) {
            const segment = { start: path[i], end: path[i + 1] };
            
            for (const obstacle of obstacles) {
                if (this.segmentIntersectsObstacle(segment, obstacle)) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    segmentIntersectsObstacle(segment, obstacle) {
        // Simplified intersection check
        const segmentLength = Math.sqrt(
            Math.pow(segment.end.x - segment.start.x, 2) +
            Math.pow(segment.end.y - segment.start.y, 2)
        );
        
        // Check points along segment
        const steps = Math.ceil(segmentLength / this.pathResolution);
        
        for (let i = 0; i <= steps; i++) {
            const t = i / steps;
            const pointX = segment.start.x + t * (segment.end.x - segment.start.x);
            const pointY = segment.start.y + t * (segment.end.y - segment.start.y);
            
            const distanceToObstacle = Math.sqrt(
                Math.pow(pointX - obstacle.position.x, 2) +
                Math.pow(pointY - obstacle.position.y, 2)
            );
            
            if (distanceToObstacle < obstacle.size / 2 + 0.2) {
                return true;
            }
        }
        
        return false;
    }
    
    generateAlternativePath(start, goal, obstacles) {
        // Simple alternative path generation
        // In practice, this would be more sophisticated
        
        const midX = (start.x + goal.x) / 2;
        const midY = (start.y + goal.y) / 2;
        
        // Try offset waypoints
        const offsets = [
            { x: midX + 0.5, y: midY },
            { x: midX - 0.5, y: midY },
            { x: midX, y: midY + 0.5 },
            { x: midX, y: midY - 0.5 }
        ];
        
        for (const offset of offsets) {
            const testPath = [start, offset, goal];
            if (!this.checkPathForObstacles(testPath, obstacles)) {
                return testPath;
            }
        }
        
        // Fallback to direct path
        return [start, goal];
    }
}

// Create global instance
window.ObstacleAvoidance = new ObstacleAvoidance();

// Export for module systems
if (typeof module !== 'undefined' && module.exports) {
    module.exports = ObstacleAvoidance;
}