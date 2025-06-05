/**
 * Platform Core - Main integration script that coordinates all platform modules
 * Provides the primary interface for system control, job management, and module coordination
 */

class PlatformCore {
    constructor() {
        // System state
        this.systemState = 'IDLE'; // IDLE, RUNNING, PAUSED, ERROR
        this.isInitialized = false;
        
        // Job management
        this.jobQueue = [];
        this.currentJob = null;
        this.jobsCompleted = 0;
        
        // Robot state
        this.robotPosition = { x: 0, y: 0, z: 0 };
        this.robotOrientation = 0;
        this.robotVelocity = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
        
        // Navigation state
        this.navigationInterval = null;
        this.currentNavigationTarget = null;
        this.navigationStartTime = null;
        
        // Performance monitoring
        this.lastUpdateTime = Date.now();
        this.updateCount = 0;
        this.performanceMetrics = {
            distanceError: 0,
            headingError: 0,
            navigationTime: 0,
            jobCompletionRate: 0
        };
        
        // UI update intervals
        this.uiUpdateInterval = null;
        this.performanceUpdateInterval = null;
        
        // Event handlers
        this.eventHandlers = new Map();
        
        // Module references
        this.modules = {
            config: null,
            rosBridge: null,
            algorithms: null,
            analytics: null,
            robotSimulator: null,
            obstacleAvoidance: null
        };
    }
    
    /**
     * Initialize the platform and all modules
     */
    async initialize() {
        if (this.isInitialized) {
            console.warn('Platform already initialized');
            return;
        }
        
        console.log('🚀 Initializing ROS 2 Material Handling Platform...');
        this.logMessage('🚀 Platform initialization started');
        
        try {
            // Initialize modules in order
            await this.initializeModules();
            
            // Setup event listeners
            this.setupEventListeners();
            
            // Initialize UI
            this.initializeUI();
            
            // Start update loops
            this.startUpdateLoops();
            
            // Load initial configuration
            this.loadInitialState();
            
            this.isInitialized = true;
            this.logMessage('✅ Platform initialization completed');
            console.log('✅ Platform initialization completed');
            
            // Emit initialization complete event
            this.emitEvent('platformInitialized');
            
        } catch (error) {
            console.error('❌ Platform initialization failed:', error);
            this.logMessage(`❌ Platform initialization failed: ${error.message}`);
            this.systemState = 'ERROR';
            this.updateSystemStateDisplay();
        }
    }
    
    /**
     * Initialize all platform modules
     */
    async initializeModules() {
        console.log('Initializing platform modules...');
        
        // Store module references
        this.modules.config = window.ConfigManager;
        this.modules.rosBridge = window.ROSBridge;
        this.modules.algorithms = window.AlgorithmComparison;
        this.modules.analytics = window.AnalyticsEngine;
        
        // Initialize robot simulator if available
        if (window.RobotSimulator) {
            this.modules.robotSimulator = window.RobotSimulator;
        }
        
        // Initialize obstacle avoidance if available
        if (window.ObstacleAvoidance) {
            this.modules.obstacleAvoidance = window.ObstacleAvoidance;
        }
        
        // Verify critical modules
        if (!this.modules.config || !this.modules.rosBridge || !this.modules.algorithms) {
            throw new Error('Critical modules not available');
        }
        
        console.log('All modules initialized successfully');
    }
    
    /**
     * Setup event listeners for inter-module communication
     */
    setupEventListeners() {
        // Robot state updates
        window.addEventListener('robotStateUpdate', (event) => {
            this.handleRobotStateUpdate(event.detail);
        });
        
        // ROS connection events
        this.modules.rosBridge.onConnection((event, data) => {
            this.handleConnectionEvent(event, data);
        });
        
        // Job events
        window.addEventListener('jobCompleted', (event) => {
            this.handleJobCompletion(event.detail);
        });
        
        // Algorithm changes
        window.addEventListener('algorithmChanged', (event) => {
            this.handleAlgorithmChange(event.detail);
        });
        
        // Configuration changes
        this.modules.config.onChange('controlParams', () => {
            this.updateAlgorithmParameters();
        });
        
        // Error handling
        window.addEventListener('error', (event) => {
            this.handleGlobalError(event);
        });
    }
    
    /**
     * Initialize UI components and populate initial data
     */
    initializeUI() {
        console.log('Initializing UI components...');
        
        // Populate location grid
        this.populateLocationGrid();
        
        // Populate location selectors
        this.populateLocationSelectors();
        
        // Update material counts
        this.updateMaterialCounts();
        
        // Update job queue display
        this.updateJobQueueDisplay();
        
        // Update algorithm parameters
        this.updateAlgorithmParameters();
        
        // Set initial system state
        this.updateSystemStateDisplay();
        
        console.log('UI components initialized');
    }
    
    /**
     * Start update loops for UI and performance monitoring
     */
    startUpdateLoops() {
        // UI update loop (every 100ms)
        this.uiUpdateInterval = setInterval(() => {
            this.updateUI();
        }, 100);
        
        // Performance update loop (every 500ms)
        this.performanceUpdateInterval = setInterval(() => {
            this.updatePerformanceMetrics();
        }, 500);
        
        console.log('Update loops started');
    }
    
    /**
     * Load initial system state and configuration
     */
    loadInitialState() {
        // Load saved material counts
        const savedCounts = this.modules.config.get('materialCounts');
        if (savedCounts) {
            Object.assign(this.modules.config.configs.materialCounts, savedCounts);
            this.updateMaterialCounts();
        }
        
        // Set default control algorithm
        const defaultAlgorithm = this.modules.config.get('defaultControlAlgorithm', 'proportional');
        this.setControlMode(defaultAlgorithm);
        
        console.log('Initial state loaded');
    }
    
    /**
     * Handle robot state updates from ROS
     * @param {object} robotState - Updated robot state
     */
    handleRobotStateUpdate(robotState) {
        // Update internal state
        this.robotPosition = robotState.position;
        this.robotOrientation = robotState.orientation;
        this.robotVelocity = robotState.velocity;
        
        // Update UI
        this.updateRobotPositionDisplay();
        this.updateRobotLocationIndicator();
        
        // Update performance metrics
        this.updateCount++;
        
        // Handle navigation if active
        if (this.currentNavigationTarget && this.systemState === 'RUNNING') {
            this.updateNavigation();
        }
        
        // Emit robot state event for other modules
        this.emitEvent('robotStateChanged', robotState);
    }
    
    /**
     * Handle ROS connection events
     * @param {string} event - Connection event type
     * @param {*} data - Event data
     */
    handleConnectionEvent(event, data) {
        switch (event) {
            case 'connected':
                this.logMessage('✅ Connected to ROS Bridge');
                break;
            case 'disconnected':
                this.logMessage('🔌 Disconnected from ROS Bridge');
                if (this.systemState === 'RUNNING') {
                    this.pauseSystem();
                    this.logMessage('⚠️ System auto-paused due to connection loss');
                }
                break;
            case 'error':
                this.logMessage(`❌ ROS Connection Error: ${data}`);
                if (this.systemState === 'RUNNING') {
                    this.pauseSystem();
                }
                break;
        }
    }
    
    /**
     * Set control algorithm mode
     * @param {string} mode - Algorithm mode
     */
    setControlMode(mode) {
        if (!this.modules.algorithms) {
            console.error('Algorithm module not available');
            return;
        }
        
        this.modules.algorithms.setAlgorithm(mode);
        
        // Update UI
        const modeElements = document.querySelectorAll('.mode-button');
        modeElements.forEach(btn => {
            btn.classList.remove('active');
            if (btn.textContent.toLowerCase().includes(mode.replace('_', ' '))) {
                btn.classList.add('active');
            }
        });
        
        // Update control mode display
        const controlModeEl = document.getElementById('control-mode');
        if (controlModeEl) {
            controlModeEl.textContent = mode.toUpperCase();
        }
        
        // Update algorithm parameters display
        this.updateAlgorithmParameters();
        
        this.logMessage(`🔧 Control mode changed to: ${mode.toUpperCase()}`);
    }
    
    /**
     * Add transport job to queue
     */
    addTransportJob() {
        const pickupSelect = document.getElementById('pickup-location');
        const dropoffSelect = document.getElementById('dropoff-location');
        const quantityInput = document.getElementById('item-quantity');
        
        if (!pickupSelect || !dropoffSelect || !quantityInput) {
            console.error('Job form elements not found');
            return;
        }
        
        const pickup = pickupSelect.value;
        const dropoff = dropoffSelect.value;
        const quantity = parseInt(quantityInput.value) || 1;
        
        // Validation
        if (!pickup || !dropoff) {
            alert('Please select both pickup and dropoff locations');
            return;
        }
        
        if (pickup === dropoff) {
            alert('Pickup and dropoff locations cannot be the same');
            return;
        }
        
        const availableItems = this.modules.config.getMaterialCount(pickup);
        if (availableItems < quantity) {
            alert(`Not enough items at ${this.modules.config.getLocation(pickup).name}. Available: ${availableItems}`);
            return;
        }
        
        // Create job
        const job = {
            id: Date.now(),
            pickup: pickup,
            dropoff: dropoff,
            quantity: quantity,
            status: 'QUEUED',
            created: new Date().toLocaleTimeString(),
            stage: 'PICKUP'
        };
        
        this.jobQueue.push(job);
        this.updateJobQueueDisplay();
        
        const pickupLocation = this.modules.config.getLocation(pickup);
        const dropoffLocation = this.modules.config.getLocation(dropoff);
        this.logMessage(`➕ Job added: Move ${quantity} items from ${pickupLocation.name} to ${dropoffLocation.name}`);
        
        // Reset form
        pickupSelect.value = '';
        dropoffSelect.value = '';
        quantityInput.value = '1';
        
        // Emit job added event
        this.emitEvent('jobAdded', job);
    }
    
    /**
     * Start the system
     */
    startSystem() {
        if (!this.modules.rosBridge.isConnected) {
            alert('Cannot start system - ROS Bridge not connected');
            this.logMessage('❌ Start failed: ROS Bridge not connected');
            return;
        }
        
        if (this.jobQueue.length === 0) {
            alert('No jobs in queue. Add some transport jobs first.');
            return;
        }
        
        this.systemState = 'RUNNING';
        this.updateSystemStateDisplay();
        this.logMessage('▶️ System started');
        
        if (!this.currentJob) {
            this.startNextJob();
        }
        
        // Emit system started event
        this.emitEvent('systemStarted');
    }
    
    /**
     * Pause the system
     */
    pauseSystem() {
        this.systemState = 'PAUSED';
        this.updateSystemStateDisplay();
        this.stopNavigation();
        this.stopRobot();
        this.logMessage('⏸️ System paused');
        
        // Emit system paused event
        this.emitEvent('systemPaused');
    }
    
    /**
     * Stop the system
     */
    stopSystem() {
        this.systemState = 'IDLE';
        this.updateSystemStateDisplay();
        this.stopNavigation();
        this.stopRobot();
        
        // Return current job to queue if exists
        if (this.currentJob) {
            this.currentJob.status = 'CANCELLED';
            this.jobQueue.unshift(this.currentJob);
            this.currentJob = null;
        }
        
        this.updateCurrentJobDisplay();
        this.updateJobQueueDisplay();
        this.logMessage('⏹️ System stopped');
        
        // Emit system stopped event
        this.emitEvent('systemStopped');
    }
    
    /**
     * Emergency stop
     */
    emergencyStop() {
        this.stopSystem();
        this.modules.rosBridge.stopRobot();
        this.logMessage('🛑 EMERGENCY STOP ACTIVATED');
        
        // Emit emergency stop event
        this.emitEvent('emergencyStop');
    }
    
    /**
     * Reset robot state
     */
    resetRobot() {
        this.stopNavigation();
        this.modules.rosBridge.stopRobot();
        
        if (this.modules.algorithms) {
            this.modules.algorithms.resetAlgorithmState();
        }
        
        this.logMessage('🔄 Robot reset completed');
        
        // Emit robot reset event
        this.emitEvent('robotReset');
    }
    
    /**
     * Clear job queue
     */
    clearJobs() {
        if (this.systemState === 'RUNNING') {
            if (!confirm('System is running. Stop and clear all jobs?')) {
                return;
            }
            this.stopSystem();
        }
        
        this.jobQueue = [];
        this.currentJob = null;
        this.updateCurrentJobDisplay();
        this.updateJobQueueDisplay();
        this.logMessage('🗑️ Job queue cleared');
        
        // Emit jobs cleared event
        this.emitEvent('jobsCleared');
    }
    
    /**
     * Return robot to home position
     */
    returnHome() {
        if (this.systemState === 'RUNNING') {
            this.pauseSystem();
        }
        
        this.stopNavigation();
        this.logMessage('🏠 Returning to home base...');
        this.startNavigation('home');
        
        // Emit return home event
        this.emitEvent('returningHome');
    }
    
    /**
     * Manual robot movement
     * @param {string} direction - Movement direction
     */
    manualMove(direction) {
        if (this.systemState === 'RUNNING') {
            this.logMessage('⚠️ Cannot use manual control while system is running');
            return;
        }
        
        if (!this.modules.rosBridge.isConnected) {
            this.logMessage('❌ Cannot move - not connected to ROS');
            return;
        }
        
        let linear = 0, angular = 0;
        
        switch (direction) {
            case 'forward':
                linear = 0.3;
                break;
            case 'backward':
                linear = -0.3;
                break;
            case 'left':
                angular = 0.5;
                break;
            case 'right':
                angular = -0.5;
                break;
            case 'stop':
                break;
        }
        
        this.modules.rosBridge.publishVelocity(linear, angular);
        this.logMessage(`🎮 Manual control: ${direction}`);
    }
    
    /**
     * Start next job in queue
     */
    startNextJob() {
        if (this.jobQueue.length === 0 || this.systemState !== 'RUNNING') {
            return;
        }
        
        this.currentJob = this.jobQueue.shift();
        this.currentJob.status = 'ACTIVE';
        this.currentJob.stage = 'PICKUP';
        this.currentJob.startTime = Date.now();
        
        this.updateCurrentJobDisplay();
        this.updateJobQueueDisplay();
        
        const pickupLocation = this.modules.config.getLocation(this.currentJob.pickup);
        const dropoffLocation = this.modules.config.getLocation(this.currentJob.dropoff);
        this.logMessage(`🚀 Starting job: Move ${this.currentJob.quantity} items from ${pickupLocation.name} to ${dropoffLocation.name}`);
        
        // Start navigation to pickup location
        this.startNavigation(this.currentJob.pickup);
        
        // Emit job started event
        this.emitEvent('jobStarted', this.currentJob);
    }
    
    /**
     * Start navigation to location
     * @param {string} locationId - Target location ID
     */
    startNavigation(locationId) {
        if (this.navigationInterval) {
            this.stopNavigation();
        }
        
        this.currentNavigationTarget = locationId;
        this.navigationStartTime = Date.now();
        
        const location = this.modules.config.getLocation(locationId);
        this.logMessage(`🎯 Navigating to ${location.name} using ${this.modules.algorithms.currentAlgorithm.toUpperCase()} control`);
        
        // Start navigation update loop
        this.navigationInterval = setInterval(() => {
            if (this.systemState !== 'RUNNING' || !this.currentNavigationTarget) {
                this.stopNavigation();
                return;
            }
            this.updateNavigation();
        }, 100); // 10 Hz update rate
    }
    
    /**
     * Update navigation control
     */
    updateNavigation() {
        if (!this.currentNavigationTarget || !this.modules.algorithms) return;
        
        const targetLocation = this.modules.config.getLocation(this.currentNavigationTarget);
        const currentPose = {
            x: this.robotPosition.x,
            y: this.robotPosition.y,
            orientation: this.robotOrientation
        };
        
        const targetPose = {
            x: targetLocation.x,
            y: targetLocation.y,
            orientation: 0 // Default orientation
        };
        
        // Calculate distance to target
        const dx = targetPose.x - currentPose.x;
        const dy = targetPose.y - currentPose.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // Update performance metrics
        this.performanceMetrics.distanceError = distance;
        
        // Check if arrived
        if (distance < 0.15) {
            this.handleArrival();
            return;
        }
        
        // Calculate control output
        const velocities = this.modules.algorithms.calculateControl(currentPose, targetPose);
        
        // Send velocity commands
        this.modules.rosBridge.publishVelocity(velocities.linear, velocities.angular);
    }
    
    /**
     * Handle arrival at navigation target
     */
    handleArrival() {
        const location = this.modules.config.getLocation(this.currentNavigationTarget);
        this.logMessage(`✅ Arrived at ${location.name}`);
        
        this.stopNavigation();
        
        if (this.currentJob) {
            this.handleJobProgress();
        }
        
        // Emit arrival event
        this.emitEvent('arrivedAtLocation', {
            locationId: this.currentNavigationTarget,
            location: location,
            job: this.currentJob
        });
    }
    
    /**
     * Handle job progress when arriving at locations
     */
    handleJobProgress() {
        if (!this.currentJob) return;
        
        if (this.currentJob.stage === 'PICKUP') {
            this.logMessage(`🔄 Arriving at pickup location...`);
            
            setTimeout(() => {
                this.performPickup();
            }, 1000);
            
        } else if (this.currentJob.stage === 'TRANSPORT') {
            this.logMessage(`🔄 Arriving at dropoff location...`);
            
            setTimeout(() => {
                this.performDropoff();
            }, 1000);
        }
    }
    
    /**
     * Perform pickup operation
     */
    performPickup() {
        if (!this.currentJob) return;
        
        const pickupLocation = this.modules.config.getLocation(this.currentJob.pickup);
        this.logMessage(`📦 Picking up ${this.currentJob.quantity} items from ${pickupLocation.name}`);
        
        // Update material count
        const currentCount = this.modules.config.getMaterialCount(this.currentJob.pickup);
        this.modules.config.setMaterialCount(this.currentJob.pickup, currentCount - this.currentJob.quantity);
        this.updateMaterialCounts();
        
        // Update job stage
        this.currentJob.stage = 'TRANSPORT';
        this.updateCurrentJobDisplay();
        
        setTimeout(() => {
            this.logMessage('🚚 Pickup complete, moving to dropoff location');
            this.startNavigation(this.currentJob.dropoff);
        }, 1500);
    }
    
    /**
     * Perform dropoff operation
     */
    performDropoff() {
        if (!this.currentJob) return;
        
        const dropoffLocation = this.modules.config.getLocation(this.currentJob.dropoff);
        this.logMessage(`📦 Dropping off ${this.currentJob.quantity} items at ${dropoffLocation.name}`);
        
        // Update material count
        const currentCount = this.modules.config.getMaterialCount(this.currentJob.dropoff);
        this.modules.config.setMaterialCount(this.currentJob.dropoff, currentCount + this.currentJob.quantity);
        this.updateMaterialCounts();
        
        // Complete job
        this.completeCurrentJob();
    }
    
    /**
     * Complete current job
     */
    completeCurrentJob() {
        if (!this.currentJob) return;
        
        this.currentJob.status = 'COMPLETED';
        this.currentJob.completionTime = Date.now();
        this.currentJob.duration = this.currentJob.completionTime - this.currentJob.startTime;
        
        this.jobsCompleted++;
        this.updateJobsCompletedDisplay();
        
        const pickupLocation = this.modules.config.getLocation(this.currentJob.pickup);
        const dropoffLocation = this.modules.config.getLocation(this.currentJob.dropoff);
        this.logMessage(`✅ Job completed: ${this.currentJob.quantity} items moved from ${pickupLocation.name} to ${dropoffLocation.name}`);
        
        // Emit job completed event
        this.emitEvent('jobCompleted', this.currentJob);
        
        this.currentJob = null;
        this.updateCurrentJobDisplay();
        
        // Start next job after delay
        setTimeout(() => {
            this.startNextJob();
        }, 2000);
    }
    
    /**
     * Stop navigation
     */
    stopNavigation() {
        if (this.navigationInterval) {
            clearInterval(this.navigationInterval);
            this.navigationInterval = null;
        }
        this.currentNavigationTarget = null;
        this.navigationStartTime = null;
        this.stopRobot();
    }
    
    /**
     * Stop robot movement
     */
    stopRobot() {
        if (this.modules.rosBridge) {
            this.modules.rosBridge.stopRobot();
        }
    }
    
    /**
     * Update UI components
     */
    updateUI() {
        // Update update rate display
        const now = Date.now();
        if (now - this.lastUpdateTime > 1000) {
            const updateRateEl = document.getElementById('update-rate');
            if (updateRateEl) {
                updateRateEl.textContent = `${this.updateCount}Hz`;
            }
            this.updateCount = 0;
            this.lastUpdateTime = now;
        }
    }
    
    /**
     * Update performance metrics display
     */
    updatePerformanceMetrics() {
        // Update distance error
        const distanceErrorEl = document.getElementById('distance-error');
        if (distanceErrorEl) {
            distanceErrorEl.textContent = `${this.performanceMetrics.distanceError.toFixed(2)}m`;
        }
        
        // Update heading error
        const headingErrorEl = document.getElementById('heading-error');
        if (headingErrorEl && this.modules.algorithms) {
            const headingError = this.modules.algorithms.performanceMetrics.headingError || 0;
            headingErrorEl.textContent = `${headingError.toFixed(1)}°`;
        }
        
        // Update velocity displays
        const linearVelEl = document.getElementById('linear-velocity');
        const angularVelEl = document.getElementById('angular-velocity');
        
        if (linearVelEl) {
            const linearSpeed = Math.sqrt(
                Math.pow(this.robotVelocity.linear.x, 2) + 
                Math.pow(this.robotVelocity.linear.y, 2)
            );
            linearVelEl.textContent = `${linearSpeed.toFixed(2)}m/s`;
        }
        
        if (angularVelEl) {
            const angularSpeed = Math.abs(this.robotVelocity.angular.z) * 180 / Math.PI;
            angularVelEl.textContent = `${angularSpeed.toFixed(1)}°/s`;
        }
    }
    
    /**
     * Update robot position display
     */
    updateRobotPositionDisplay() {
        const positionEl = document.getElementById('robot-position');
        if (positionEl) {
            positionEl.textContent = `(${this.robotPosition.x.toFixed(2)}, ${this.robotPosition.y.toFixed(2)})`;
        }
    }
    
    /**
     * Update robot location indicator
     */
    updateRobotLocationIndicator() {
        // Remove existing indicators
        document.querySelectorAll('.location-card').forEach(card => {
            card.classList.remove('robot-here');
            const statusSpan = card.querySelector('.robot-status');
            if (statusSpan) {
                statusSpan.remove();
            }
        });
        
        // Find closest location
        let closestLocation = null;
        let minDistance = Infinity;
        
        const locations = this.modules.config.getAllLocations();
        Object.keys(locations).forEach(locId => {
            const loc = locations[locId];
            const distance = Math.sqrt(
                Math.pow(this.robotPosition.x - loc.x, 2) + 
                Math.pow(this.robotPosition.y - loc.y, 2)
            );
            
            if (distance < minDistance && distance < 0.3) {
                minDistance = distance;
                closestLocation = locId;
            }
        });
        
        // Add indicator to closest location
        if (closestLocation) {
            const card = document.querySelector(`[data-location="${closestLocation}"]`);
            if (card) {
                card.classList.add('robot-here');
                
                const statusSpan = document.createElement('span');
                statusSpan.className = 'robot-status';
                statusSpan.innerHTML = '🤖 Robot Here';
                card.appendChild(statusSpan);
            }
        }
    }
    
    /**
     * Update system state display
     */
    updateSystemStateDisplay() {
        const stateEl = document.getElementById('system-state');
        if (stateEl) {
            stateEl.textContent = this.systemState;
            
            // Update color based on state
            switch (this.systemState) {
                case 'RUNNING':
                    stateEl.style.color = '#00ff88';
                    break;
                case 'PAUSED':
                    stateEl.style.color = '#ffaa00';
                    break;
                case 'ERROR':
                    stateEl.style.color = '#ff4444';
                    break;
                default:
                    stateEl.style.color = '#aaaaaa';
            }
        }
    }
    
    /**
     * Update current job display
     */
    updateCurrentJobDisplay() {
        const jobEl = document.getElementById('current-job');
        if (!jobEl) return;
        
        if (this.currentJob) {
            const pickupLocation = this.modules.config.getLocation(this.currentJob.pickup);
            const dropoffLocation = this.modules.config.getLocation(this.currentJob.dropoff);
            
            jobEl.innerHTML = `
                <strong>Moving ${this.currentJob.quantity} items</strong><br>
                ${pickupLocation.name} → ${dropoffLocation.name}<br>
                <span style="color: #00ff88;">Stage: ${this.currentJob.stage}</span>
            `;
        } else {
            jobEl.textContent = 'None';
        }
    }
    
    /**
     * Update job queue display
     */
    updateJobQueueDisplay() {
        const queueEl = document.getElementById('job-queue');
        if (!queueEl) return;
        
        queueEl.innerHTML = '';
        
        // Show current job
        if (this.currentJob) {
            const jobDiv = document.createElement('div');
            jobDiv.className = 'job-item active';
            const pickupLocation = this.modules.config.getLocation(this.currentJob.pickup);
            const dropoffLocation = this.modules.config.getLocation(this.currentJob.dropoff);
            
            jobDiv.innerHTML = `
                <strong>🔥 ACTIVE:</strong> Move ${this.currentJob.quantity} items<br>
                ${pickupLocation.name} → ${dropoffLocation.name}<br>
                Stage: ${this.currentJob.stage || 'PICKUP'}
            `;
            queueEl.appendChild(jobDiv);
        }
        
        // Show queued jobs
        this.jobQueue.forEach((job, index) => {
            const jobDiv = document.createElement('div');
            jobDiv.className = 'job-item';
            const pickupLocation = this.modules.config.getLocation(job.pickup);
            const dropoffLocation = this.modules.config.getLocation(job.dropoff);
            
            jobDiv.innerHTML = `
                <strong>Job ${index + 1}:</strong> Move ${job.quantity} items<br>
                ${pickupLocation.name} → ${dropoffLocation.name}<br>
                <small>Created: ${job.created}</small>
            `;
            queueEl.appendChild(jobDiv);
        });
        
        // Show empty message if no jobs
        if (!this.currentJob && this.jobQueue.length === 0) {
            queueEl.innerHTML = '<p style="text-align: center; color: #666; font-style: italic;">No jobs in queue</p>';
        }
    }
    
    /**
     * Update jobs completed display
     */
    updateJobsCompletedDisplay() {
        const completedEl = document.getElementById('jobs-completed');
        if (completedEl) {
            completedEl.textContent = this.jobsCompleted;
        }
    }
    
    /**
     * Update material counts display
     */
    updateMaterialCounts() {
        const locations = this.modules.config.getAllLocations();
        Object.keys(locations).forEach(locId => {
            const card = document.querySelector(`[data-location="${locId}"]`);
            if (card) {
                const countSpan = card.querySelector('.material-count');
                if (countSpan) {
                    const count = this.modules.config.getMaterialCount(locId);
                    countSpan.textContent = `Items: ${count}`;
                }
            }
        });
    }
    
    /**
     * Populate location grid
     */
    populateLocationGrid() {
        const gridEl = document.getElementById('location-grid');
        if (!gridEl) return;
        
        gridEl.innerHTML = '';
        const locations = this.modules.config.getAllLocations();
        
        Object.entries(locations).forEach(([locId, location]) => {
            const card = document.createElement('div');
            card.className = 'location-card';
            card.setAttribute('data-location', locId);
            card.onclick = () => this.selectLocation(locId);
            
            const count = this.modules.config.getMaterialCount(locId);
            
            card.innerHTML = `
                <strong>${this.getLocationIcon(locId)} ${location.name}</strong><br>
                (${location.x.toFixed(1)}, ${location.y.toFixed(1)})<br>
                <span class="material-count">Items: ${count}</span>
            `;
            
            gridEl.appendChild(card);
        });
    }
    
    /**
     * Populate location selectors
     */
    populateLocationSelectors() {
        const pickupSelect = document.getElementById('pickup-location');
        const dropoffSelect = document.getElementById('dropoff-location');
        
        if (!pickupSelect || !dropoffSelect) return;
        
        const locations = this.modules.config.getAllLocations();
        
        // Clear existing options (except the first one)
        pickupSelect.innerHTML = '<option value="">📤 Select pickup</option>';
        dropoffSelect.innerHTML = '<option value="">📥 Select dropoff</option>';
        
        // Add location options
        Object.entries(locations).forEach(([locId, location]) => {
            if (locId !== 'home') { // Don't allow pickup from home
                const pickupOption = document.createElement('option');
                pickupOption.value = locId;
                pickupOption.textContent = `${this.getLocationIcon(locId)} ${location.name}`;
                pickupSelect.appendChild(pickupOption);
            }
            
            const dropoffOption = document.createElement('option');
            dropoffOption.value = locId;
            dropoffOption.textContent = `${this.getLocationIcon(locId)} ${location.name}`;
            dropoffSelect.appendChild(dropoffOption);
        });
    }
    
    /**
     * Get icon for location
     * @param {string} locationId - Location ID
     * @returns {string} Location icon
     */
    getLocationIcon(locationId) {
        const icons = {
            'home': '🏠',
            'storage_a': '🏪',
            'storage_b': '🏪',
            'workstation': '🔧',
            'loading': '📦',
            'inspection': '🔍'
        };
        return icons[locationId] || '📍';
    }
    
    /**
     * Select location
     * @param {string} locationId - Location ID
     */
    selectLocation(locationId) {
        document.querySelectorAll('.location-card').forEach(card => {
            card.classList.remove('selected');
        });
        
        const card = document.querySelector(`[data-location="${locationId}"]`);
        if (card) {
            card.classList.add('selected');
        }
        
        const location = this.modules.config.getLocation(locationId);
        this.logMessage(`📍 Selected location: ${location.name}`);
    }
    
    /**
     * Update algorithm parameters display
     */
    updateAlgorithmParameters() {
        if (!this.modules.algorithms) return;
        
        const paramsEl = document.getElementById('algorithm-params');
        if (paramsEl) {
            const template = this.modules.algorithms.getParameterTemplate(this.modules.algorithms.currentAlgorithm);
            paramsEl.innerHTML = template;
        }
    }
    
    /**
     * Log message to system log
     * @param {string} message - Message to log
     */
    logMessage(message) {
        const timestamp = new Date().toLocaleTimeString();
        const logEl = document.getElementById('system-log');
        
        if (logEl) {
            logEl.textContent += `[${timestamp}] ${message}\n`;
            logEl.scrollTop = logEl.scrollHeight;
            
            // Keep log size manageable
            const lines = logEl.textContent.split('\n');
            if (lines.length > 100) {
                logEl.textContent = lines.slice(-80).join('\n');
            }
        }
        
        console.log(`[${timestamp}] ${message}`);
    }
    
    /**
     * Emit custom event
     * @param {string} eventName - Event name
     * @param {*} data - Event data
     */
    emitEvent(eventName, data = null) {
        window.dispatchEvent(new CustomEvent(eventName, {
            detail: data
        }));
    }
    
    /**
     * Handle global errors
     * @param {object} event - Error event
     */
    handleGlobalError(event) {
        console.error('Global error:', event.error);
        this.logMessage(`❌ Error: ${event.error.message}`);
        
        // Set error state if critical error
        if (this.systemState === 'RUNNING') {
            this.pauseSystem();
        }
    }
    
    /**
     * Get current system state
     * @returns {object} System state information
     */
    getSystemState() {
        return {
            systemState: this.systemState,
            currentJob: this.currentJob,
            jobQueue: [...this.jobQueue],
            jobsCompleted: this.jobsCompleted,
            robotPosition: { ...this.robotPosition },
            robotOrientation: this.robotOrientation,
            isNavigating: this.currentNavigationTarget !== null,
            currentTarget: this.currentNavigationTarget,
            performanceMetrics: { ...this.performanceMetrics }
        };
    }
    
    /**
     * Get current job
     * @returns {object|null} Current job
     */
    getCurrentJob() {
        return this.currentJob;
    }
    
    /**
     * Cleanup and shutdown
     */
    shutdown() {
        this.logMessage('🛑 Platform shutdown initiated');
        
        // Stop all operations
        this.stopSystem();
        
        // Clear intervals
        if (this.uiUpdateInterval) {
            clearInterval(this.uiUpdateInterval);
        }
        if (this.performanceUpdateInterval) {
            clearInterval(this.performanceUpdateInterval);
        }
        
        // Disconnect from ROS
        if (this.modules.rosBridge) {
            this.modules.rosBridge.disconnect();
        }
        
        // Save final state
        this.modules.config.saveToStorage();
        
        this.isInitialized = false;
        this.logMessage('✅ Platform shutdown completed');
    }
}

// Create global instance
window.PlatformCore = new PlatformCore();

// Auto-initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
        window.PlatformCore.initialize();
    });
} else {
    // DOM is already ready
    window.PlatformCore.initialize();
}

// Export for module systems
if (typeof module !== 'undefined' && module.exports) {
    module.exports = PlatformCore;
}