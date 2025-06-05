/**
 * Advanced Material Handling System
 * Main controller for the material handling interface
 */

class MaterialHandlingSystem {
    constructor() {
        this.isInitialized = false;
        this.systemState = 'IDLE';
        this.currentControlMode = 'proportional';
        this.robotPosition = { x: 0, y: 0 };
        this.robotOrientation = 0;
        this.robotVelocity = { linear: 0, angular: 0 };
        this.currentJob = null;
        this.jobQueue = [];
        this.jobsCompleted = 0;
        this.navigationInterval = null;
        this.currentNavigationTarget = null;
        this.distanceError = 0;
        this.headingError = 0;
        
        // Material counts at each location
        this.materialCounts = {
            'storage_a': 8,
            'storage_b': 5,
            'loading': 15,
            'inspection': 3,
            'workstation': 0,
            'home': 0
        };
        
        // Location coordinates
        this.locations = {
            'home': { x: 0.0, y: 0.0, name: 'Home Base' },
            'storage_a': { x: 0.0, y: 2.0, name: 'Storage A' },
            'storage_b': { x: 2.0, y: 2.0, name: 'Storage B' },
            'workstation': { x: 2.0, y: 0.0, name: 'Workstation' },
            'loading': { x: 1.0, y: 0.0, name: 'Loading Dock' },
            'inspection': { x: 1.0, y: 2.0, name: 'Inspection' }
        };
        
        this.init();
    }
    
    async init() {
        try {
            this.logMessage('🚀 Initializing Advanced Material Handling System...');
            
            // Initialize components
            this.initializeControlAlgorithms();
            this.initializeJobManager();
            this.initializeExcelIntegration();
            this.initializeEventHandlers();
            this.initializeROS();
            
            // Update initial display
            this.updateMaterialCounts();
            this.updateJobQueueDisplay();
            this.updateParameterDisplay();
            this.updateSystemStatus();
            
            this.isInitialized = true;
            this.logMessage('✅ Material Handling System initialized successfully');
            
        } catch (error) {
            console.error('Failed to initialize Material Handling System:', error);
            this.logMessage('❌ Failed to initialize system: ' + error.message);
        }
    }
    
    initializeControlAlgorithms() {
        if (window.ControlAlgorithms) {
            this.controlAlgorithms = new window.ControlAlgorithms();
            this.logMessage('🧠 Control algorithms initialized');
        } else {
            console.warn('ControlAlgorithms not available');
        }
    }
    
    initializeJobManager() {
        if (window.JobManager) {
            this.jobManager = new window.JobManager(this);
            this.logMessage('📋 Job manager initialized');
        } else {
            console.warn('JobManager not available');
        }
    }
    
    initializeExcelIntegration() {
        if (window.ExcelIntegration) {
            this.excelIntegration = new window.ExcelIntegration(this);
            this.logMessage('📊 Excel integration initialized');
        } else {
            console.warn('ExcelIntegration not available');
        }
    }
    
    initializeROS() {
        if (window.ROSBridge) {
            this.rosBridge = new window.ROSBridge();
            this.rosBridge.on('connected', () => {
                this.updateConnectionStatus('CONNECTED', true);
                this.logMessage('✅ Connected to ROS Bridge');
            });
            
            this.rosBridge.on('disconnected', () => {
                this.updateConnectionStatus('DISCONNECTED', false);
                this.logMessage('🔌 Disconnected from ROS Bridge');
                if (this.systemState === 'RUNNING') {
                    this.pauseSystem();
                }
            });
            
            this.rosBridge.on('error', (error) => {
                this.updateConnectionStatus('ERROR', false);
                this.logMessage('❌ ROS Error: ' + error);
            });
            
            this.rosBridge.on('odometry', (data) => {
                this.handleOdometryUpdate(data);
            });
            
            this.rosBridge.connect();
        } else {
            console.warn('ROSBridge not available');
            this.logMessage('⚠️ ROS Bridge not available - running in simulation mode');
        }
    }
    
    initializeEventHandlers() {
        // Control mode buttons
        document.querySelectorAll('.mode-button').forEach(button => {
            button.addEventListener('click', (e) => {
                const mode = e.currentTarget.dataset.mode;
                this.setControlMode(mode);
            });
        });
        
        // Location cards
        document.querySelectorAll('.location-card').forEach(card => {
            card.addEventListener('click', (e) => {
                const location = e.currentTarget.dataset.location;
                this.selectLocation(location);
            });
        });
        
        // System control buttons
        document.getElementById('start-btn')?.addEventListener('click', () => this.startSystem());
        document.getElementById('pause-btn')?.addEventListener('click', () => this.pauseSystem());
        document.getElementById('stop-btn')?.addEventListener('click', () => this.stopSystem());
        
        // Job management
        document.getElementById('add-job-btn')?.addEventListener('click', () => this.addTransportJob());
        
        // Emergency controls
        document.getElementById('emergency-stop-btn')?.addEventListener('click', () => this.emergencyStop());
        document.getElementById('reset-robot-btn')?.addEventListener('click', () => this.resetRobot());
        document.getElementById('clear-jobs-btn')?.addEventListener('click', () => this.clearJobs());
        document.getElementById('return-home-btn')?.addEventListener('click', () => this.returnHome());
        
        // Manual controls
        document.querySelectorAll('.dpad-btn').forEach(btn => {
            btn.addEventListener('mousedown', (e) => {
                const direction = e.currentTarget.dataset.direction;
                this.manualMove(direction);
            });
            btn.addEventListener('mouseup', () => this.stopManualMove());
            btn.addEventListener('mouseleave', () => this.stopManualMove());
        });
        
        // Excel controls
        document.getElementById('start-recording-btn')?.addEventListener('click', () => {
            if (this.excelIntegration) this.excelIntegration.startDataCollection();
        });
        document.getElementById('stop-recording-btn')?.addEventListener('click', () => {
            if (this.excelIntegration) this.excelIntegration.stopDataCollection();
        });
        document.getElementById('export-excel-btn')?.addEventListener('click', () => {
            if (this.excelIntegration) this.excelIntegration.exportToExcel();
        });
        document.getElementById('clear-data-btn')?.addEventListener('click', () => {
            if (this.excelIntegration) this.excelIntegration.clearDataCollection();
        });
        
        // Modal handlers
        this.initializeModalHandlers();
        
        // Performance update interval
        setInterval(() => {
            this.updatePerformanceDisplay();
        }, 500);
    }
    
    initializeModalHandlers() {
        const settingsBtn = document.getElementById('settings-btn');
        const helpBtn = document.getElementById('help-btn');
        const settingsModal = document.getElementById('settings-modal');
        const helpModal = document.getElementById('help-modal');
        const closeButtons = document.querySelectorAll('.close');
        
        if (settingsBtn && settingsModal) {
            settingsBtn.addEventListener('click', () => {
                settingsModal.style.display = 'block';
            });
        }
        
        if (helpBtn && helpModal) {
            helpBtn.addEventListener('click', () => {
                helpModal.style.display = 'block';
            });
        }
        
        closeButtons.forEach(button => {
            button.addEventListener('click', (e) => {
                e.target.closest('.modal').style.display = 'none';
            });
        });
        
        window.addEventListener('click', (e) => {
            if (e.target.classList.contains('modal')) {
                e.target.style.display = 'none';
            }
        });
        
        // Fullscreen handler
        const fullscreenBtn = document.getElementById('fullscreen-btn');
        if (fullscreenBtn) {
            fullscreenBtn.addEventListener('click', () => {
                if (!document.fullscreenElement) {
                    document.documentElement.requestFullscreen();
                } else {
                    document.exitFullscreen();
                }
            });
        }
        
        // Reconnect handler
        const reconnectBtn = document.getElementById('reconnect-btn');
        if (reconnectBtn) {
            reconnectBtn.addEventListener('click', () => {
                if (this.rosBridge) {
                    this.rosBridge.reconnect();
                    this.logMessage('🔄 Reconnecting to ROS Bridge...');
                }
            });
        }
    }
    
    // System Control Methods
    setControlMode(mode) {
        this.currentControlMode = mode;
        document.getElementById('control-mode').textContent = mode.toUpperCase();
        
        // Update mode buttons
        document.querySelectorAll('.mode-button').forEach(btn => {
            btn.classList.remove('active');
        });
        document.querySelector(`[data-mode="${mode}"]`).classList.add('active');
        
        // Reset navigation with new mode
        if (this.navigationInterval) {
            this.stopNavigation();
            if (this.currentNavigationTarget) {
                setTimeout(() => {
                    this.startNavigation(this.currentNavigationTarget);
                }, 100);
            }
        }
        
        this.updateParameterDisplay();
        this.logMessage(`🔧 Control mode changed to: ${mode.toUpperCase()}`);
    }
    
    selectLocation(locationId) {
        document.querySelectorAll('.location-card').forEach(card => {
            card.classList.remove('selected');
        });
        document.querySelector(`[data-location="${locationId}"]`).classList.add('selected');
        this.logMessage(`📍 Selected location: ${this.locations[locationId].name}`);
    }
    
    startSystem() {
        if (!this.rosBridge?.isConnected && this.rosBridge) {
            alert('Cannot start system - ROS Bridge not connected');
            this.logMessage('❌ Start failed: ROS Bridge not connected');
            return;
        }
        
        if (this.jobQueue.length === 0) {
            alert('No jobs in queue. Add some transport jobs first.');
            return;
        }
        
        this.systemState = 'RUNNING';
        this.updateSystemStatus();
        this.logMessage('▶️ System started');
        
        if (!this.currentJob && this.jobManager) {
            this.jobManager.startNextJob();
        }
    }
    
    pauseSystem() {
        this.systemState = 'PAUSED';
        this.updateSystemStatus();
        this.stopNavigation();
        this.stopRobot();
        this.logMessage('⏸️ System paused');
    }
    
    stopSystem() {
        this.systemState = 'IDLE';
        this.updateSystemStatus();
        this.stopNavigation();
        this.stopRobot();
        
        if (this.currentJob) {
            this.currentJob.status = 'CANCELLED';
            this.jobQueue.unshift(this.currentJob);
            this.currentJob = null;
        }
        
        document.getElementById('current-job').textContent = 'None';
        this.updateJobQueueDisplay();
        this.logMessage('⏹️ System stopped');
    }
    
    emergencyStop() {
        this.stopSystem();
        this.stopRobot();
        this.logMessage('🛑 EMERGENCY STOP ACTIVATED');
    }
    
    resetRobot() {
        this.stopNavigation();
        this.stopRobot();
        
        if (this.controlAlgorithms) {
            this.controlAlgorithms.resetPIDState();
        }
        
        this.logMessage('🔄 Robot reset completed');
    }
    
    clearJobs() {
        if (this.systemState === 'RUNNING') {
            if (!confirm('System is running. Stop and clear all jobs?')) {
                return;
            }
            this.stopSystem();
        }
        
        this.jobQueue = [];
        this.currentJob = null;
        document.getElementById('current-job').textContent = 'None';
        this.updateJobQueueDisplay();
        this.logMessage('🗑️ Job queue cleared');
    }
    
    returnHome() {
        if (this.systemState === 'RUNNING') {
            this.pauseSystem();
        }
        
        this.stopNavigation();
        this.logMessage('🏠 Returning to home base...');
        this.startNavigation('home');
    }
    
    addTransportJob() {
        const pickup = document.getElementById('pickup-location').value;
        const dropoff = document.getElementById('dropoff-location').value;
        const quantity = parseInt(document.getElementById('item-quantity').value) || 1;
        
        if (!pickup || !dropoff) {
            alert('Please select both pickup and dropoff locations');
            return;
        }
        
        if (pickup === dropoff) {
            alert('Pickup and dropoff locations cannot be the same');
            return;
        }
        
        if (this.materialCounts[pickup] < quantity) {
            alert(`Not enough items at ${this.locations[pickup].name}. Available: ${this.materialCounts[pickup]}`);
            return;
        }
        
        if (this.jobManager) {
            this.jobManager.addJob(pickup, dropoff, quantity);
        }
        
        // Reset form
        document.getElementById('pickup-location').value = '';
        document.getElementById('dropoff-location').value = '';
        document.getElementById('item-quantity').value = '1';
    }
    
    // Navigation Methods
    startNavigation(locationId) {
        if (this.controlAlgorithms) {
            this.controlAlgorithms.startNavigation(locationId, this.locations[locationId]);
        }
        this.currentNavigationTarget = locationId;
    }
    
    stopNavigation() {
        if (this.controlAlgorithms) {
            this.controlAlgorithms.stopNavigation();
        }
        this.currentNavigationTarget = null;
        this.stopRobot();
    }
    
    stopRobot() {
        if (this.rosBridge) {
            this.rosBridge.publishVelocity(0, 0);
        }
        this.robotVelocity.linear = 0;
        this.robotVelocity.angular = 0;
    }
    
    manualMove(direction) {
        if (this.systemState === 'RUNNING') {
            this.logMessage('⚠️ Cannot use manual control while system is running');
            return;
        }
        
        if (!this.rosBridge?.isConnected) {
            this.logMessage('❌ Cannot move - not connected to ROS');
            return;
        }
        
        let linear = 0, angular = 0;
        
        switch(direction) {
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
        
        this.rosBridge.publishVelocity(linear, angular);
        this.logMessage(`🎮 Manual control: ${direction}`);
    }
    
    stopManualMove() {
        if (this.systemState !== 'RUNNING') {
            this.stopRobot();
        }
    }
    
    // Data Handling Methods
    handleOdometryUpdate(data) {
        this.robotPosition.x = data.pose.pose.position.x;
        this.robotPosition.y = data.pose.pose.position.y;
        
        // Extract robot orientation from quaternion
        const orientation = data.pose.pose.orientation;
        this.robotOrientation = Math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        );
        
        // Update display
        document.getElementById('robot-position').textContent = 
            `(${this.robotPosition.x.toFixed(2)}, ${this.robotPosition.y.toFixed(2)})`;
            
        this.updateRobotLocationIndicator();
        
        // Update control algorithms
        if (this.controlAlgorithms) {
            this.controlAlgorithms.updateRobotPose(this.robotPosition, this.robotOrientation);
        }
        
        // Update excel data collection
        if (this.excelIntegration) {
            this.excelIntegration.recordDataPoint(this.robotPosition, this.robotOrientation, data);
        }
        
        // Check job progress
        if (this.jobManager && this.currentJob && this.systemState === 'RUNNING') {
            this.jobManager.checkJobProgress();
        }
    }
    
    // UI Update Methods
    updateSystemStatus() {
        const stateElement = document.getElementById('system-state');
        if (stateElement) {
            stateElement.textContent = this.systemState;
            switch(this.systemState) {
                case 'RUNNING':
                    stateElement.style.color = '#00ff88';
                    break;
                case 'PAUSED':
                    stateElement.style.color = '#ffaa00';
                    break;
                default:
                    stateElement.style.color = '#aaaaaa';
            }
        }
    }
    
    updateConnectionStatus(status, isConnected) {
        const statusElement = document.getElementById('connection-status');
        const dotElement = document.getElementById('status-dot');
        
        if (statusElement) statusElement.textContent = status;
        if (dotElement) {
            dotElement.className = `status-dot ${isConnected ? 'connected' : 'disconnected'}`;
        }
        
        const rosStatusElement = document.getElementById('ros-status');
        if (rosStatusElement) {
            rosStatusElement.textContent = status;
            rosStatusElement.style.color = isConnected ? '#00ff88' : '#ff4444';
        }
    }
    
    updateMaterialCounts() {
        Object.keys(this.materialCounts).forEach(locId => {
            const card = document.querySelector(`[data-location="${locId}"]`);
            const countSpan = card?.querySelector('.material-count');
            if (countSpan) {
                countSpan.textContent = `Items: ${this.materialCounts[locId]}`;
            }
        });
    }
    
    updateRobotLocationIndicator() {
        document.querySelectorAll('.location-card').forEach(card => {
            card.classList.remove('robot-here');
            const robotStatus = card.querySelector('.robot-status');
            if (robotStatus) robotStatus.remove();
        });
        
        let closestLocation = null;
        let minDistance = Infinity;
        
        Object.keys(this.locations).forEach(locId => {
            const loc = this.locations[locId];
            const distance = Math.sqrt(
                Math.pow(this.robotPosition.x - loc.x, 2) + 
                Math.pow(this.robotPosition.y - loc.y, 2)
            );
            
            if (distance < minDistance && distance < 0.3) {
                minDistance = distance;
                closestLocation = locId;
            }
        });
        
        if (closestLocation) {
            const card = document.querySelector(`[data-location="${closestLocation}"]`);
            if (card) {
                card.classList.add('robot-here');
                
                const statusSpan = document.createElement('div');
                statusSpan.className = 'robot-status';
                statusSpan.innerHTML = '🤖 Robot Here';
                card.appendChild(statusSpan);
            }
        }
    }
    
    updateJobQueueDisplay() {
        if (this.jobManager) {
            this.jobManager.updateJobQueueDisplay();
        }
    }
    
    updateParameterDisplay() {
        if (this.controlAlgorithms) {
            this.controlAlgorithms.updateParameterDisplay(this.currentControlMode);
        }
    }
    
    updatePerformanceDisplay() {
        document.getElementById('distance-error').textContent = `${this.distanceError.toFixed(2)}m`;
        document.getElementById('heading-error').textContent = `${(this.headingError || 0).toFixed(1)}°`;
        document.getElementById('linear-velocity').textContent = `${this.robotVelocity.linear.toFixed(2)}m/s`;
        document.getElementById('angular-velocity').textContent = `${(this.robotVelocity.angular * 180 / Math.PI).toFixed(1)}°/s`;
    }
    
    // Utility Methods
    logMessage(message) {
        const timestamp = new Date().toLocaleTimeString();
        const logDiv = document.getElementById('system-log');
        if (logDiv) {
            logDiv.textContent += `[${timestamp}] ${message}\n`;
            logDiv.scrollTop = logDiv.scrollHeight;
            
            // Keep log size manageable
            const lines = logDiv.textContent.split('\n');
            if (lines.length > 100) {
                logDiv.textContent = lines.slice(-80).join('\n');
            }
        }
        
        console.log(`[MaterialHandling] ${message}`);
    }
    
    // Public API Methods
    getSystemState() {
        return this.systemState;
    }
    
    getRobotPosition() {
        return { ...this.robotPosition };
    }
    
    getCurrentJob() {
        return this.currentJob;
    }
    
    getJobQueue() {
        return [...this.jobQueue];
    }
    
    getMaterialCounts() {
        return { ...this.materialCounts };
    }
    
    getLocations() {
        return { ...this.locations };
    }
}

// Make MaterialHandlingSystem available globally
window.MaterialHandlingSystem = MaterialHandlingSystem;