/**
 * Analytics Engine - Data collection, analysis, and Excel export functionality
 * Handles real-time data logging, statistical analysis, and comprehensive reporting
 */

class AnalyticsEngine {
    constructor() {
        this.isCollecting = false;
        this.robotData = [];
        this.collectionStartTime = null;
        this.collectionTimer = null;
        this.lastJobEvent = null;
        
        // Data collection settings
        this.collectionRate = ConfigManager.get('analytics.collectionRate', 10); // Hz
        this.maxDataPoints = ConfigManager.get('analytics.maxDataPoints', 10000);
        this.exportBatchSize = ConfigManager.get('analytics.exportBatchSize', 1000);
        
        // Performance tracking
        this.performanceMetrics = {
            totalJobs: 0,
            successfulJobs: 0,
            failedJobs: 0,
            totalDistance: 0,
            totalTime: 0,
            avgVelocity: 0,
            maxVelocity: 0
        };
        
        // Event listeners
        this.setupEventListeners();
    }
    
    /**
     * Setup event listeners for data collection
     */
    setupEventListeners() {
        // Listen for robot state updates
        window.addEventListener('robotStateUpdate', (event) => {
            if (this.isCollecting) {
                this.collectDataPoint(event.detail);
            }
        });
        
        // Listen for job events
        window.addEventListener('jobCompleted', (event) => {
            this.recordJobEvent('completed', event.detail);
        });
        
        window.addEventListener('jobStarted', (event) => {
            this.recordJobEvent('started', event.detail);
        });
        
        window.addEventListener('algorithmChanged', (event) => {
            this.recordJobEvent('algorithm_change', event.detail);
        });
    }
    
    /**
     * Start data collection
     */
    startDataCollection() {
        if (this.isCollecting) {
            console.warn('Data collection already running');
            return;
        }
        
        this.isCollecting = true;
        this.robotData = [];
        this.collectionStartTime = new Date();
        
        // Update UI
        this.updateCollectionStatus('RECORDING');
        
        // Start collection timer for UI updates
        this.collectionTimer = setInterval(() => {
            this.updateDataCollectionDisplay();
        }, 1000);
        
        console.log('📊 Data collection started');
        this.logMessage('📊 Excel data collection started');
    }
    
    /**
     * Stop data collection
     */
    stopDataCollection() {
        if (!this.isCollecting) {
            console.warn('Data collection not running');
            return;
        }
        
        this.isCollecting = false;
        
        // Update UI
        this.updateCollectionStatus('STOPPED');
        
        // Clear timer
        if (this.collectionTimer) {
            clearInterval(this.collectionTimer);
            this.collectionTimer = null;
        }
        
        console.log(`📊 Data collection stopped - ${this.robotData.length} records collected`);
        this.logMessage(`📊 Excel data collection stopped - ${this.robotData.length} records collected`);
    }
    
    /**
     * Clear collected data
     */
    clearDataCollection() {
        if (this.isCollecting) {
            this.stopDataCollection();
        }
        
        this.robotData = [];
        this.performanceMetrics = {
            totalJobs: 0,
            successfulJobs: 0,
            failedJobs: 0,
            totalDistance: 0,
            totalTime: 0,
            avgVelocity: 0,
            maxVelocity: 0
        };
        
        // Update UI
        this.updateDataCollectionDisplay();
        this.updateDataPreview();
        
        console.log('📊 Data collection cleared');
        this.logMessage('📊 Excel data collection cleared');
    }
    
    /**
     * Collect a data point
     * @param {object} robotState - Current robot state
     */
    collectDataPoint(robotState) {
        if (!this.isCollecting || !this.collectionStartTime) return;
        
        const currentTime = new Date();
        const speed = Math.sqrt(
            Math.pow(robotState.velocity.linear.x, 2) + 
            Math.pow(robotState.velocity.linear.y, 2)
        );
        
        // Get current system state from PlatformCore if available
        const systemState = window.PlatformCore ? window.PlatformCore.getSystemState() : 'UNKNOWN';
        const currentJob = window.PlatformCore ? window.PlatformCore.getCurrentJob() : null;
        
        const dataPoint = {
            timestamp: currentTime.toISOString(),
            time_seconds: (currentTime - this.collectionStartTime) / 1000,
            excel_timestamp: currentTime.getTime() / 86400000 + 25569, // Excel date format
            x_position: robotState.position.x,
            y_position: robotState.position.y,
            z_position: robotState.position.z || 0,
            velocity: speed,
            heading_deg: robotState.orientation * 180 / Math.PI,
            linear_velocity_x: robotState.velocity.linear.x,
            linear_velocity_y: robotState.velocity.linear.y,
            angular_velocity_z: robotState.velocity.angular.z,
            control_mode: window.AlgorithmComparison ? window.AlgorithmComparison.currentAlgorithm : 'unknown',
            system_state: systemState,
            current_job: currentJob ? `${currentJob.pickup} → ${currentJob.dropoff}` : 'None',
            job_stage: currentJob ? currentJob.stage : 'IDLE',
            distance_error: window.AlgorithmComparison ? window.AlgorithmComparison.performanceMetrics.distanceError : 0,
            heading_error: window.AlgorithmComparison ? window.AlgorithmComparison.performanceMetrics.headingError : 0,
            battery_level: this.getBatteryLevel(), // Mock data for now
            cpu_usage: this.getCPUUsage(), // Mock data for now
            memory_usage: this.getMemoryUsage() // Mock data for now
        };
        
        // Add data point
        this.robotData.push(dataPoint);
        
        // Limit data size
        if (this.robotData.length > this.maxDataPoints) {
            this.robotData.shift();
        }
        
        // Update performance metrics
        this.updatePerformanceMetrics(dataPoint);
        
        // Update UI periodically (every 10th point to reduce overhead)
        if (this.robotData.length % 10 === 0) {
            this.updateDataPreview();
        }
    }
    
    /**
     * Record job events
     * @param {string} eventType - Type of event
     * @param {object} eventData - Event data
     */
    recordJobEvent(eventType, eventData) {
        this.lastJobEvent = {
            timestamp: new Date().toISOString(),
            event_type: eventType,
            event_data: eventData,
            time_seconds: this.collectionStartTime ? (new Date() - this.collectionStartTime) / 1000 : 0
        };
        
        // Update performance metrics for job events
        if (eventType === 'completed') {
            this.performanceMetrics.successfulJobs++;
            this.performanceMetrics.totalJobs++;
        } else if (eventType === 'failed') {
            this.performanceMetrics.failedJobs++;
            this.performanceMetrics.totalJobs++;
        }
    }
    
    /**
     * Update performance metrics
     * @param {object} dataPoint - Latest data point
     */
    updatePerformanceMetrics(dataPoint) {
        // Calculate total distance traveled
        if (this.robotData.length > 1) {
            const prevPoint = this.robotData[this.robotData.length - 2];
            const dx = dataPoint.x_position - prevPoint.x_position;
            const dy = dataPoint.y_position - prevPoint.y_position;
            const distance = Math.sqrt(dx * dx + dy * dy);
            this.performanceMetrics.totalDistance += distance;
        }
        
        // Update velocity metrics
        this.performanceMetrics.maxVelocity = Math.max(this.performanceMetrics.maxVelocity, dataPoint.velocity);
        
        // Calculate average velocity
        const velocities = this.robotData.map(d => d.velocity);
        this.performanceMetrics.avgVelocity = velocities.reduce((a, b) => a + b, 0) / velocities.length;
        
        // Update total time
        this.performanceMetrics.totalTime = dataPoint.time_seconds;
    }
    
    /**
     * Export data to Excel
     */
    exportToExcel() {
        if (this.robotData.length === 0) {
            alert('No data to export. Start data collection first.');
            return;
        }
        
        try {
            console.log('Starting Excel export...');
            
            // Create workbook
            const workbook = XLSX.utils.book_new();
            
            // Main data sheet
            this.createMainDataSheet(workbook);
            
            // Statistics sheet
            this.createStatisticsSheet(workbook);
            
            // Control analysis sheet
            this.createControlAnalysisSheet(workbook);
            
            // Job timeline sheet
            this.createJobTimelineSheet(workbook);
            
            // Performance metrics sheet
            this.createPerformanceSheet(workbook);
            
            // Analysis templates sheet
            this.createAnalysisTemplateSheet(workbook);
            
            // Save file
            const filename = `robot_analytics_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.xlsx`;
            XLSX.writeFile(workbook, filename);
            
            this.logMessage(`📊 Excel file exported: ${filename} (${this.robotData.length} records)`);
            alert(`Excel file exported successfully!\n\nFile: ${filename}\nRecords: ${this.robotData.length}\nSheets: 6 comprehensive analysis sheets`);
            
        } catch (error) {
            console.error('Excel export failed:', error);
            alert('Failed to export Excel file. Please try again.');
        }
    }
    
    /**
     * Create main data sheet
     * @param {object} workbook - Excel workbook
     */
    createMainDataSheet(workbook) {
        const worksheet = XLSX.utils.json_to_sheet(this.robotData);
        
        // Set column widths
        worksheet['!cols'] = [
            { width: 22 }, // Timestamp
            { width: 12 }, // Time Seconds
            { width: 15 }, // Excel Date
            { width: 12 }, // X Position
            { width: 12 }, // Y Position
            { width: 12 }, // Z Position
            { width: 12 }, // Velocity
            { width: 12 }, // Heading
            { width: 12 }, // Linear Vel X
            { width: 12 }, // Linear Vel Y
            { width: 12 }, // Angular Vel Z
            { width: 15 }, // Control Mode
            { width: 15 }, // System State
            { width: 25 }, // Current Job
            { width: 12 }, // Job Stage
            { width: 15 }, // Distance Error
            { width: 15 }, // Heading Error
            { width: 12 }, // Battery Level
            { width: 12 }, // CPU Usage
            { width: 12 }  // Memory Usage
        ];
        
        XLSX.utils.book_append_sheet(workbook, worksheet, 'Robot_Data');
    }
    
    /**
     * Create statistics sheet
     * @param {object} workbook - Excel workbook
     */
    createStatisticsSheet(workbook) {
        const stats = this.calculateStatistics();
        const statsSheet = XLSX.utils.json_to_sheet([stats]);
        XLSX.utils.book_append_sheet(workbook, statsSheet, 'Statistics');
    }
    
    /**
     * Create control analysis sheet
     * @param {object} workbook - Excel workbook
     */
    createControlAnalysisSheet(workbook) {
        const controlAnalysis = this.analyzeControlPerformance();
        if (controlAnalysis.length > 0) {
            const controlSheet = XLSX.utils.json_to_sheet(controlAnalysis);
            XLSX.utils.book_append_sheet(workbook, controlSheet, 'Control_Analysis');
        }
    }
    
    /**
     * Create job timeline sheet
     * @param {object} workbook - Excel workbook
     */
    createJobTimelineSheet(workbook) {
        const jobEvents = this.extractJobEvents();
        if (jobEvents.length > 0) {
            const jobSheet = XLSX.utils.json_to_sheet(jobEvents);
            XLSX.utils.book_append_sheet(workbook, jobSheet, 'Job_Timeline');
        }
    }
    
    /**
     * Create performance metrics sheet
     * @param {object} workbook - Excel workbook
     */
    createPerformanceSheet(workbook) {
        const performance = [
            { Metric: 'Total Jobs', Value: this.performanceMetrics.totalJobs },
            { Metric: 'Successful Jobs', Value: this.performanceMetrics.successfulJobs },
            { Metric: 'Failed Jobs', Value: this.performanceMetrics.failedJobs },
            { Metric: 'Success Rate (%)', Value: this.performanceMetrics.totalJobs > 0 ? (this.performanceMetrics.successfulJobs / this.performanceMetrics.totalJobs * 100).toFixed(2) : 0 },
            { Metric: 'Total Distance (m)', Value: this.performanceMetrics.totalDistance.toFixed(3) },
            { Metric: 'Total Time (s)', Value: this.performanceMetrics.totalTime.toFixed(1) },
            { Metric: 'Average Velocity (m/s)', Value: this.performanceMetrics.avgVelocity.toFixed(3) },
            { Metric: 'Maximum Velocity (m/s)', Value: this.performanceMetrics.maxVelocity.toFixed(3) },
            { Metric: 'Data Points Collected', Value: this.robotData.length },
            { Metric: 'Collection Duration (min)', Value: (this.performanceMetrics.totalTime / 60).toFixed(2) }
        ];
        
        const performanceSheet = XLSX.utils.json_to_sheet(performance);
        XLSX.utils.book_append_sheet(workbook, performanceSheet, 'Performance');
    }
    
    /**
     * Create analysis template sheet
     * @param {object} workbook - Excel workbook
     */
    createAnalysisTemplateSheet(workbook) {
        const templates = [
            { Formula: '=AVERAGE(Robot_Data.G:G)', Description: 'Average Velocity (m/s)', Category: 'Velocity Analysis' },
            { Formula: '=MAX(Robot_Data.G:G)', Description: 'Maximum Velocity (m/s)', Category: 'Velocity Analysis' },
            { Formula: '=STDEV(Robot_Data.G:G)', Description: 'Velocity Std Dev (m/s)', Category: 'Velocity Analysis' },
            { Formula: '=AVERAGE(Robot_Data.P:P)', Description: 'Average Distance Error (m)', Category: 'Control Performance' },
            { Formula: '=MAX(Robot_Data.P:P)', Description: 'Maximum Distance Error (m)', Category: 'Control Performance' },
            { Formula: '=AVERAGE(ABS(Robot_Data.Q:Q))', Description: 'Average Heading Error (deg)', Category: 'Control Performance' },
            { Formula: '=COUNT(Robot_Data.A:A)', Description: 'Total Data Points', Category: 'General Statistics' },
            { Formula: '=MAX(Robot_Data.B:B)', Description: 'Total Collection Time (s)', Category: 'General Statistics' },
            { Formula: '=COUNTIF(Robot_Data.M:M,"RUNNING")', Description: 'Time in Running State', Category: 'System Analysis' },
            { Formula: '=COUNTIF(Robot_Data.L:L,"proportional")', Description: 'Proportional Control Usage', Category: 'Algorithm Usage' },
            { Formula: '=COUNTIF(Robot_Data.L:L,"pid")', Description: 'PID Control Usage', Category: 'Algorithm Usage' },
            { Formula: '=COUNTIF(Robot_Data.L:L,"pure_pursuit")', Description: 'Pure Pursuit Usage', Category: 'Algorithm Usage' }
        ];
        
        const templateSheet = XLSX.utils.json_to_sheet(templates);
        XLSX.utils.book_append_sheet(workbook, templateSheet, 'Excel_Formulas');
    }
    
    /**
     * Calculate comprehensive statistics
     * @returns {object} Statistics object
     */
    calculateStatistics() {
        if (this.robotData.length === 0) return {};
        
        const velocities = this.robotData.map(d => d.velocity);
        const xPositions = this.robotData.map(d => d.x_position);
        const yPositions = this.robotData.map(d => d.y_position);
        const distanceErrors = this.robotData.map(d => d.distance_error).filter(d => d > 0);
        const headingErrors = this.robotData.map(d => Math.abs(d.heading_error)).filter(d => !isNaN(d));
        
        return {
            'Collection_Start': this.robotData[0].timestamp,
            'Collection_End': this.robotData[this.robotData.length-1].timestamp,
            'Total_Data_Points': this.robotData.length,
            'Collection_Duration_s': this.robotData[this.robotData.length-1].time_seconds,
            'Total_Distance_m': this.performanceMetrics.totalDistance.toFixed(3),
            'Average_Velocity_ms': this.calculateAverage(velocities).toFixed(3),
            'Max_Velocity_ms': Math.max(...velocities).toFixed(3),
            'Min_Velocity_ms': Math.min(...velocities).toFixed(3),
            'Velocity_StdDev_ms': this.calculateStandardDeviation(velocities).toFixed(3),
            'Average_Distance_Error_m': distanceErrors.length > 0 ? this.calculateAverage(distanceErrors).toFixed(3) : 'N/A',
            'Max_Distance_Error_m': distanceErrors.length > 0 ? Math.max(...distanceErrors).toFixed(3) : 'N/A',
            'Average_Heading_Error_deg': headingErrors.length > 0 ? this.calculateAverage(headingErrors).toFixed(2) : 'N/A',
            'Workspace_X_Range_m': (Math.max(...xPositions) - Math.min(...xPositions)).toFixed(3),
            'Workspace_Y_Range_m': (Math.max(...yPositions) - Math.min(...yPositions)).toFixed(3),
            'Jobs_Completed': this.performanceMetrics.successfulJobs,
            'Jobs_Failed': this.performanceMetrics.failedJobs,
            'Success_Rate_Percent': this.performanceMetrics.totalJobs > 0 ? ((this.performanceMetrics.successfulJobs / this.performanceMetrics.totalJobs) * 100).toFixed(2) : 'N/A'
        };
    }
    
    /**
     * Analyze control performance by algorithm
     * @returns {array} Control analysis data
     */
    analyzeControlPerformance() {
        const controlModes = [...new Set(this.robotData.map(d => d.control_mode))];
        const analysis = [];
        
        controlModes.forEach(mode => {
            const modeData = this.robotData.filter(d => d.control_mode === mode);
            if (modeData.length < 10) return;
            
            const velocities = modeData.map(d => d.velocity);
            const distanceErrors = modeData.map(d => d.distance_error).filter(d => d > 0);
            const headingErrors = modeData.map(d => Math.abs(d.heading_error)).filter(d => !isNaN(d));
            
            analysis.push({
                'Control_Mode': mode,
                'Sample_Count': modeData.length,
                'Duration_Seconds': (modeData[modeData.length-1].time_seconds - modeData[0].time_seconds).toFixed(1),
                'Average_Velocity': this.calculateAverage(velocities).toFixed(3),
                'Average_Distance_Error': distanceErrors.length > 0 ? this.calculateAverage(distanceErrors).toFixed(3) : 'N/A',
                'Average_Heading_Error': headingErrors.length > 0 ? this.calculateAverage(headingErrors).toFixed(2) : 'N/A',
                'Velocity_StdDev': this.calculateStandardDeviation(velocities).toFixed(3),
                'Control_Efficiency': this.calculateControlEfficiency(modeData).toFixed(3)
            });
        });
        
        return analysis;
    }
    
    /**
     * Extract job events from data
     * @returns {array} Job events
     */
    extractJobEvents() {
        const events = [];
        let lastJobStage = '';
        let lastJob = '';
        
        this.robotData.forEach(record => {
            if (record.job_stage !== lastJobStage || record.current_job !== lastJob) {
                events.push({
                    'Time_Seconds': record.time_seconds,
                    'Timestamp': record.timestamp,
                    'Event_Type': 'Job State Change',
                    'Job': record.current_job,
                    'Stage': record.job_stage,
                    'Robot_Position': `(${record.x_position.toFixed(2)}, ${record.y_position.toFixed(2)})`,
                    'Control_Mode': record.control_mode,
                    'Velocity': record.velocity.toFixed(2)
                });
                lastJobStage = record.job_stage;
                lastJob = record.current_job;
            }
        });
        
        return events;
    }
    
    /**
     * Calculate average of array
     * @param {array} values - Array of numbers
     * @returns {number} Average
     */
    calculateAverage(values) {
        return values.length > 0 ? values.reduce((a, b) => a + b, 0) / values.length : 0;
    }
    
    /**
     * Calculate standard deviation
     * @param {array} values - Array of numbers
     * @returns {number} Standard deviation
     */
    calculateStandardDeviation(values) {
        if (values.length === 0) return 0;
        const avg = this.calculateAverage(values);
        const squareDiffs = values.map(value => Math.pow(value - avg, 2));
        const avgSquareDiff = squareDiffs.reduce((a, b) => a + b, 0) / squareDiffs.length;
        return Math.sqrt(avgSquareDiff);
    }
    
    /**
     * Calculate control efficiency
     * @param {array} data - Control mode data
     * @returns {number} Efficiency metric
     */
    calculateControlEfficiency(data) {
        if (data.length < 2) return 0;
        
        let totalProgress = 0;
        let totalEnergyUsed = 0;
        
        for (let i = 1; i < data.length; i++) {
            const progress = Math.max(0, data[i-1].distance_error - data[i].distance_error);
            const energyUsed = Math.abs(data[i].velocity) + Math.abs(data[i].angular_velocity_z);
            
            totalProgress += progress;
            totalEnergyUsed += energyUsed;
        }
        
        return totalEnergyUsed > 0 ? totalProgress / totalEnergyUsed : 0;
    }
    
    /**
     * Update data collection display
     */
    updateDataCollectionDisplay() {
        if (!this.collectionStartTime) return;
        
        // Update record count
        const recordCountEl = document.getElementById('record-count');
        if (recordCountEl) {
            recordCountEl.textContent = this.robotData.length;
        }
        
        // Update duration
        const elapsed = new Date() - this.collectionStartTime;
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        const durationEl = document.getElementById('collection-duration');
        if (durationEl) {
            durationEl.textContent = minutes.toString().padStart(2, '0') + ':' + seconds.toString().padStart(2, '0');
        }
        
        // Estimate data size
        const estimatedSize = this.robotData.length * 300; // Rough estimate
        const sizeKB = (estimatedSize / 1024).toFixed(1);
        const sizeEl = document.getElementById('data-size');
        if (sizeEl) {
            sizeEl.textContent = sizeKB + ' KB';
        }
    }
    
    /**
     * Update data preview table
     */
    updateDataPreview() {
        const tbody = document.getElementById('preview-body');
        if (!tbody) return;
        
        if (this.robotData.length === 0) {
            tbody.innerHTML = '<tr><td colspan="4" class="no-data">No data collected</td></tr>';
            return;
        }
        
        // Show last 8 records
        const recentData = this.robotData.slice(-8);
        
        tbody.innerHTML = '';
        recentData.forEach(record => {
            const row = tbody.insertRow();
            row.insertCell(0).textContent = record.time_seconds.toFixed(1) + 's';
            row.insertCell(1).textContent = record.x_position.toFixed(2);
            row.insertCell(2).textContent = record.y_position.toFixed(2);
            row.insertCell(3).textContent = record.velocity.toFixed(2);
        });
    }
    
    /**
     * Update collection status in UI
     * @param {string} status - Collection status
     */
    updateCollectionStatus(status) {
        const statusEl = document.getElementById('collection-status');
        if (statusEl) {
            statusEl.textContent = status;
            statusEl.style.color = status === 'RECORDING' ? '#ff4444' : '#00ff88';
        }
    }
    
    /**
     * Get mock battery level (for demonstration)
     * @returns {number} Battery level percentage
     */
    getBatteryLevel() {
        return Math.max(20, 100 - (this.robotData.length * 0.01));
    }
    
    /**
     * Get mock CPU usage (for demonstration)
     * @returns {number} CPU usage percentage
     */
    getCPUUsage() {
        return 20 + Math.sin(Date.now() / 10000) * 15 + Math.random() * 10;
    }
    
    /**
     * Get mock memory usage (for demonstration)
     * @returns {number} Memory usage percentage
     */
    getMemoryUsage() {
        return 30 + Math.cos(Date.now() / 15000) * 10 + Math.random() * 5;
    }
    
    /**
     * Log message to system log
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
     * Get analytics summary
     * @returns {object} Analytics summary
     */
    getAnalyticsSummary() {
        return {
            isCollecting: this.isCollecting,
            dataPoints: this.robotData.length,
            collectionTime: this.collectionStartTime ? (Date.now() - this.collectionStartTime) / 1000 : 0,
            performanceMetrics: { ...this.performanceMetrics },
            lastDataPoint: this.robotData.length > 0 ? this.robotData[this.robotData.length - 1] : null
        };
    }
}

// Create global instance
window.AnalyticsEngine = new AnalyticsEngine();

// Export for module systems
if (typeof module !== 'undefined' && module.exports) {
    module.exports = AnalyticsEngine;
}