/**
 * Excel Integration Module
 * Handles data collection, analysis, and Excel export functionality
 */

class ExcelIntegration {
    constructor(materialHandlingSystem) {
        this.mhs = materialHandlingSystem;
        this.isCollecting = false;
        this.robotData = [];
        this.jobEvents = [];
        this.systemEvents = [];
        this.collectionStartTime = null;
        this.collectionTimer = null;
        this.lastDataPoint = null;
        
        // Collection settings
        this.collectionRate = 5; // Hz
        this.maxDataPoints = 10000;
        this.autoExportOnJobComplete = true;
        this.includePerformanceData = true;
        
        this.init();
    }
    
    init() {
        console.log('Excel Integration initialized');
        this.setupCollectionInterval();
    }
    
    setupCollectionInterval() {
        // Clear existing interval
        if (this.collectionTimer) {
            clearInterval(this.collectionTimer);
        }
        
        // Set up new collection interval based on rate
        const intervalMs = 1000 / this.collectionRate;
        this.collectionTimer = setInterval(() => {
            if (this.isCollecting) {
                this.collectDataPoint();
            }
        }, intervalMs);
    }
    
    // Data Collection Methods
    startDataCollection() {
        if (this.isCollecting) return;
        
        this.isCollecting = true;
        this.robotData = [];
        this.jobEvents = [];
        this.systemEvents = [];
        this.collectionStartTime = new Date();
        
        this.updateCollectionStatus();
        this.mhs.logMessage('📊 Excel data collection started');
        
        // Record initial system state
        this.recordSystemEvent('DATA_COLLECTION_STARTED', {
            control_mode: this.mhs.currentControlMode,
            system_state: this.mhs.systemState,
            active_jobs: this.mhs.jobQueue.length
        });
    }
    
    stopDataCollection() {
        if (!this.isCollecting) return;
        
        this.isCollecting = false;
        this.updateCollectionStatus();
        
        const recordCount = this.robotData.length;
        const duration = this.collectionStartTime ? 
            ((new Date() - this.collectionStartTime) / 1000).toFixed(1) : 0;
        
        this.mhs.logMessage(`📊 Excel data collection stopped - ${recordCount} records collected in ${duration}s`);
        
        // Record final system state
        this.recordSystemEvent('DATA_COLLECTION_STOPPED', {
            total_records: recordCount,
            duration_seconds: parseFloat(duration),
            final_position: { ...this.mhs.robotPosition }
        });
    }
    
    clearDataCollection() {
        if (this.isCollecting) {
            this.stopDataCollection();
        }
        
        this.robotData = [];
        this.jobEvents = [];
        this.systemEvents = [];
        this.collectionStartTime = null;
        this.lastDataPoint = null;
        
        this.updateCollectionDisplay();
        this.mhs.logMessage('📊 Excel data collection cleared');
    }
    
    collectDataPoint() {
        if (!this.isCollecting || !this.collectionStartTime) return;
        
        const currentTime = new Date();
        const timeSeconds = (currentTime - this.collectionStartTime) / 1000;
        
        // Calculate velocity
        const velocity = Math.sqrt(
            Math.pow(this.mhs.robotVelocity.linear, 2) + 
            Math.pow(this.mhs.robotVelocity.angular, 2)
        );
        
        // Calculate distance traveled since last point
        let distanceTraveled = 0;
        if (this.lastDataPoint) {
            const dx = this.mhs.robotPosition.x - this.lastDataPoint.x_position;
            const dy = this.mhs.robotPosition.y - this.lastDataPoint.y_position;
            distanceTraveled = Math.sqrt(dx * dx + dy * dy);
        }
        
        const dataPoint = {
            timestamp: currentTime.toISOString(),
            time_seconds: timeSeconds,
            excel_timestamp: this.toExcelDate(currentTime),
            x_position: this.mhs.robotPosition.x,
            y_position: this.mhs.robotPosition.y,
            orientation_rad: this.mhs.robotOrientation,
            orientation_deg: this.mhs.robotOrientation * 180 / Math.PI,
            linear_velocity: this.mhs.robotVelocity.linear,
            angular_velocity: this.mhs.robotVelocity.angular,
            velocity_magnitude: velocity,
            distance_traveled: distanceTraveled,
            control_mode: this.mhs.currentControlMode,
            system_state: this.mhs.systemState,
            current_job_id: this.mhs.currentJob ? this.mhs.currentJob.id : null,
            job_stage: this.mhs.currentJob ? this.mhs.currentJob.stage : 'IDLE',
            distance_error: this.mhs.distanceError || 0,
            heading_error: this.mhs.headingError || 0,
            target_location: this.mhs.currentNavigationTarget || null,
            jobs_in_queue: this.mhs.jobQueue.length,
            jobs_completed: this.mhs.jobsCompleted
        };
        
        // Add performance metrics if enabled
        if (this.includePerformanceData && this.mhs.controlAlgorithms) {
            const controlParams = this.mhs.controlAlgorithms.getControlParameters(this.mhs.currentControlMode);
            dataPoint.control_parameters = JSON.stringify(controlParams);
        }
        
        this.robotData.push(dataPoint);
        this.lastDataPoint = dataPoint;
        
        // Limit data points to prevent memory issues
        if (this.robotData.length > this.maxDataPoints) {
            this.robotData = this.robotData.slice(-this.maxDataPoints + 1000);
            this.mhs.logMessage('⚠️ Data collection limit reached, oldest records removed');
        }
        
        this.updateCollectionDisplay();
        this.updateDataPreview();
    }
    
    recordDataPoint(position, orientation, odomData) {
        // This method can be called directly from odometry updates
        // for more precise timing
        if (!this.isCollecting) return;
        
        // The main collection happens in collectDataPoint()
        // This method is kept for compatibility
    }
    
    recordJobEvent(eventType, jobData, additionalData = {}) {
        if (!this.isCollecting) return;
        
        const currentTime = new Date();
        const timeSeconds = this.collectionStartTime ? 
            (currentTime - this.collectionStartTime) / 1000 : 0;
        
        const jobEvent = {
            timestamp: currentTime.toISOString(),
            time_seconds: timeSeconds,
            excel_timestamp: this.toExcelDate(currentTime),
            event_type: eventType,
            job_id: jobData.id,
            pickup_location: jobData.pickup,
            dropoff_location: jobData.dropoff,
            quantity: jobData.quantity,
            job_stage: jobData.stage,
            robot_position_x: this.mhs.robotPosition.x,
            robot_position_y: this.mhs.robotPosition.y,
            robot_orientation: this.mhs.robotOrientation,
            control_mode: this.mhs.currentControlMode,
            ...additionalData
        };
        
        this.jobEvents.push(jobEvent);
        this.mhs.logMessage(`📝 Job event recorded: ${eventType} for Job ${jobData.id}`);
    }
    
    recordJobCompletion(jobData) {
        this.recordJobEvent('JOB_COMPLETED', jobData, {
            completion_time: jobData.completionTime,
            total_duration: jobData.totalTime,
            pickup_duration: jobData.pickupTime - jobData.startTime,
            transport_duration: jobData.completionTime - jobData.pickupTime
        });
        
        // Auto-export if enabled and this was the last job
        if (this.autoExportOnJobComplete && this.mhs.jobQueue.length === 0) {
            setTimeout(() => {
                if (this.mhs.jobQueue.length === 0) { // Double-check no new jobs added
                    this.exportToExcel();
                }
            }, 2000);
        }
    }
    
    recordSystemEvent(eventType, eventData = {}) {
        if (!this.isCollecting) return;
        
        const currentTime = new Date();
        const timeSeconds = this.collectionStartTime ? 
            (currentTime - this.collectionStartTime) / 1000 : 0;
        
        const systemEvent = {
            timestamp: currentTime.toISOString(),
            time_seconds: timeSeconds,
            excel_timestamp: this.toExcelDate(currentTime),
            event_type: eventType,
            robot_position_x: this.mhs.robotPosition.x,
            robot_position_y: this.mhs.robotPosition.y,
            robot_orientation: this.mhs.robotOrientation,
            system_state: this.mhs.systemState,
            control_mode: this.mhs.currentControlMode,
            ...eventData
        };
        
        this.systemEvents.push(systemEvent);
    }
    
    // Excel Export Methods
    exportToExcel() {
        if (this.robotData.length === 0) {
            alert('No data to export. Start data collection first.');
            return;
        }
        
        try {
            this.mhs.logMessage('📊 Generating Excel export...');
            
            // Create workbook
            const workbook = XLSX.utils.book_new();
            
            // Main robot data sheet
            this.addRobotDataSheet(workbook);
            
            // Job events sheet
            if (this.jobEvents.length > 0) {
                this.addJobEventsSheet(workbook);
            }
            
            // System events sheet
            if (this.systemEvents.length > 0) {
                this.addSystemEventsSheet(workbook);
            }
            
            // Statistics and analysis sheets
            this.addStatisticsSheet(workbook);
            this.addControlAnalysisSheet(workbook);
            this.addPerformanceMetricsSheet(workbook);
            
            // Excel formulas and templates
            this.addFormulasSheet(workbook);
            
            // Save file
            const filename = this.generateFilename();
            XLSX.writeFile(workbook, filename);
            
            this.mhs.logMessage(`📊 Excel file exported: ${filename} (${this.robotData.length} records)`);
            
            // Show success message with details
            this.showExportSuccess(filename);
            
        } catch (error) {
            console.error('Excel export error:', error);
            this.mhs.logMessage(`❌ Excel export failed: ${error.message}`);
            alert(`Excel export failed: ${error.message}`);
        }
    }
    
    addRobotDataSheet(workbook) {
        const data = this.robotData.map(record => ({
            'Timestamp': record.timestamp,
            'Time_Seconds': record.time_seconds,
            'Excel_Date': record.excel_timestamp,
            'X_Position_m': record.x_position,
            'Y_Position_m': record.y_position,
            'Orientation_rad': record.orientation_rad,
            'Orientation_deg': record.orientation_deg,
            'Linear_Velocity_ms': record.linear_velocity,
            'Angular_Velocity_rads': record.angular_velocity,
            'Velocity_Magnitude_ms': record.velocity_magnitude,
            'Distance_Traveled_m': record.distance_traveled,
            'Control_Mode': record.control_mode,
            'System_State': record.system_state,
            'Current_Job_ID': record.current_job_id,
            'Job_Stage': record.job_stage,
            'Distance_Error_m': record.distance_error,
            'Heading_Error_deg': record.heading_error,
            'Target_Location': record.target_location,
            'Jobs_In_Queue': record.jobs_in_queue,
            'Jobs_Completed': record.jobs_completed
        }));
        
        const worksheet = XLSX.utils.json_to_sheet(data);
        
        // Set column widths
        worksheet['!cols'] = [
            { width: 22 }, // Timestamp
            { width: 12 }, // Time Seconds
            { width: 15 }, // Excel Date
            { width: 12 }, // X Position
            { width: 12 }, // Y Position
            { width: 15 }, // Orientation rad
            { width: 15 }, // Orientation deg
            { width: 18 }, // Linear Velocity
            { width: 20 }, // Angular Velocity
            { width: 20 }, // Velocity Magnitude
            { width: 18 }, // Distance Traveled
            { width: 15 }, // Control Mode
            { width: 15 }, // System State
            { width: 15 }, // Current Job ID
            { width: 12 }, // Job Stage
            { width: 15 }, // Distance Error
            { width: 15 }, // Heading Error
            { width: 15 }, // Target Location
            { width: 15 }, // Jobs In Queue
            { width: 15 }  // Jobs Completed
        ];
        
        XLSX.utils.book_append_sheet(workbook, worksheet, 'Robot_Data');
    }
    
    addJobEventsSheet(workbook) {
        const data = this.jobEvents.map(event => ({
            'Timestamp': event.timestamp,
            'Time_Seconds': event.time_seconds,
            'Excel_Date': event.excel_timestamp,
            'Event_Type': event.event_type,
            'Job_ID': event.job_id,
            'Pickup_Location': event.pickup_location,
            'Dropoff_Location': event.dropoff_location,
            'Quantity': event.quantity,
            'Job_Stage': event.job_stage,
            'Robot_X': event.robot_position_x,
            'Robot_Y': event.robot_position_y,
            'Robot_Orientation': event.robot_orientation,
            'Control_Mode': event.control_mode,
            'Duration_ms': event.total_duration || '',
            'Pickup_Duration_ms': event.pickup_duration || '',
            'Transport_Duration_ms': event.transport_duration || ''
        }));
        
        const worksheet = XLSX.utils.json_to_sheet(data);
        XLSX.utils.book_append_sheet(workbook, worksheet, 'Job_Events');
    }
    
    addSystemEventsSheet(workbook) {
        const data = this.systemEvents.map(event => ({
            'Timestamp': event.timestamp,
            'Time_Seconds': event.time_seconds,
            'Excel_Date': event.excel_timestamp,
            'Event_Type': event.event_type,
            'Robot_X': event.robot_position_x,
            'Robot_Y': event.robot_position_y,
            'Robot_Orientation': event.robot_orientation,
            'System_State': event.system_state,
            'Control_Mode': event.control_mode,
            'Additional_Data': JSON.stringify(event.additional_data || {})
        }));
        
        const worksheet = XLSX.utils.json_to_sheet(data);
        XLSX.utils.book_append_sheet(workbook, worksheet, 'System_Events');
    }
    
    addStatisticsSheet(workbook) {
        const stats = this.calculateStatistics();
        const statsArray = Object.entries(stats).map(([key, value]) => ({
            'Metric': key.replace(/_/g, ' ').toUpperCase(),
            'Value': value,
            'Unit': this.getMetricUnit(key)
        }));
        
        const worksheet = XLSX.utils.json_to_sheet(statsArray);
        XLSX.utils.book_append_sheet(workbook, worksheet, 'Statistics');
    }
    
    addControlAnalysisSheet(workbook) {
        const analysis = this.analyzeControlPerformance();
        if (analysis.length > 0) {
            const worksheet = XLSX.utils.json_to_sheet(analysis);
            XLSX.utils.book_append_sheet(workbook, worksheet, 'Control_Analysis');
        }
    }
    
    addPerformanceMetricsSheet(workbook) {
        const metrics = this.calculatePerformanceMetrics();
        const worksheet = XLSX.utils.json_to_sheet(metrics);
        XLSX.utils.book_append_sheet(workbook, worksheet, 'Performance_Metrics');
    }
    
    addFormulasSheet(workbook) {
        const formulas = [
            { Category: 'Basic Statistics', Formula: '=AVERAGE(Robot_Data.D:D)', Description: 'Average X Position', Target_Column: 'D (X_Position_m)' },
            { Category: 'Basic Statistics', Formula: '=AVERAGE(Robot_Data.E:E)', Description: 'Average Y Position', Target_Column: 'E (Y_Position_m)' },
            { Category: 'Velocity Analysis', Formula: '=AVERAGE(Robot_Data.H:H)', Description: 'Average Linear Velocity', Target_Column: 'H (Linear_Velocity_ms)' },
            { Category: 'Velocity Analysis', Formula: '=MAX(Robot_Data.H:H)', Description: 'Maximum Linear Velocity', Target_Column: 'H (Linear_Velocity_ms)' },
            { Category: 'Velocity Analysis', Formula: '=STDEV(Robot_Data.H:H)', Description: 'Linear Velocity Std Dev', Target_Column: 'H (Linear_Velocity_ms)' },
            { Category: 'Control Performance', Formula: '=AVERAGE(Robot_Data.P:P)', Description: 'Average Distance Error', Target_Column: 'P (Distance_Error_m)' },
            { Category: 'Control Performance', Formula: '=AVERAGE(ABS(Robot_Data.Q:Q))', Description: 'Average Heading Error (Abs)', Target_Column: 'Q (Heading_Error_deg)' },
            { Category: 'Distance Analysis', Formula: '=SUM(Robot_Data.K:K)', Description: 'Total Distance Traveled', Target_Column: 'K (Distance_Traveled_m)' },
            { Category: 'Time Analysis', Formula: '=MAX(Robot_Data.B:B)', Description: 'Total Collection Time', Target_Column: 'B (Time_Seconds)' },
            { Category: 'Job Analysis', Formula: '=MAX(Robot_Data.S:S)', Description: 'Total Jobs Completed', Target_Column: 'S (Jobs_Completed)' }
        ];
        
        const worksheet = XLSX.utils.json_to_sheet(formulas);
        XLSX.utils.book_append_sheet(workbook, worksheet, 'Excel_Formulas');
    }
    
    // Analysis Methods
    calculateStatistics() {
        if (this.robotData.length === 0) return {};
        
        const positions = this.robotData.map(d => ({ x: d.x_position, y: d.y_position }));
        const velocities = this.robotData.map(d => d.velocity_magnitude);
        const linearVels = this.robotData.map(d => d.linear_velocity);
        const angularVels = this.robotData.map(d => d.angular_velocity);
        const distanceErrors = this.robotData.map(d => d.distance_error).filter(d => d > 0);
        const headingErrors = this.robotData.map(d => Math.abs(d.heading_error)).filter(d => !isNaN(d));
        
        const totalDistance = this.robotData.reduce((sum, d) => sum + d.distance_traveled, 0);
        const duration = this.robotData.length > 0 ? 
            this.robotData[this.robotData.length - 1].time_seconds : 0;
        
        return {
            collection_start: this.robotData[0].timestamp,
            collection_end: this.robotData[this.robotData.length - 1].timestamp,
            total_data_points: this.robotData.length,
            collection_duration_s: duration,
            total_distance_m: totalDistance,
            average_velocity_ms: this.calculateMean(velocities),
            max_velocity_ms: Math.max(...velocities),
            min_velocity_ms: Math.min(...velocities),
            velocity_std_dev: this.calculateStandardDeviation(velocities),
            average_linear_velocity_ms: this.calculateMean(linearVels),
            average_angular_velocity_rads: this.calculateMean(angularVels),
            average_distance_error_m: distanceErrors.length > 0 ? this.calculateMean(distanceErrors) : 0,
            max_distance_error_m: distanceErrors.length > 0 ? Math.max(...distanceErrors) : 0,
            average_heading_error_deg: headingErrors.length > 0 ? this.calculateMean(headingErrors) : 0,
            max_heading_error_deg: headingErrors.length > 0 ? Math.max(...headingErrors) : 0,
            workspace_x_range: Math.max(...positions.map(p => p.x)) - Math.min(...positions.map(p => p.x)),
            workspace_y_range: Math.max(...positions.map(p => p.y)) - Math.min(...positions.map(p => p.y)),
            jobs_completed: Math.max(...this.robotData.map(d => d.jobs_completed)),
            data_collection_rate_hz: this.collectionRate
        };
    }
    
    analyzeControlPerformance() {
        const controlModes = [...new Set(this.robotData.map(d => d.control_mode))];
        const analysis = [];
        
        controlModes.forEach(mode => {
            const modeData = this.robotData.filter(d => d.control_mode === mode);
            if (modeData.length < 10) return;
            
            const velocities = modeData.map(d => d.velocity_magnitude);
            const distanceErrors = modeData.map(d => d.distance_error).filter(d => d > 0);
            const headingErrors = modeData.map(d => Math.abs(d.heading_error)).filter(d => !isNaN(d));
            
            analysis.push({
                Control_Mode: mode,
                Sample_Count: modeData.length,
                Duration_Seconds: modeData[modeData.length - 1].time_seconds - modeData[0].time_seconds,
                Average_Velocity_ms: this.calculateMean(velocities),
                Velocity_Std_Dev: this.calculateStandardDeviation(velocities),
                Average_Distance_Error_m: distanceErrors.length > 0 ? this.calculateMean(distanceErrors) : 0,
                Max_Distance_Error_m: distanceErrors.length > 0 ? Math.max(...distanceErrors) : 0,
                Average_Heading_Error_deg: headingErrors.length > 0 ? this.calculateMean(headingErrors) : 0,
                Max_Heading_Error_deg: headingErrors.length > 0 ? Math.max(...headingErrors) : 0,
                Distance_Traveled_m: modeData.reduce((sum, d) => sum + d.distance_traveled, 0)
            });
        });
        
        return analysis;
    }
    
    calculatePerformanceMetrics() {
        const jobStages = [...new Set(this.robotData.map(d => d.job_stage))];
        const metrics = [];
        
        jobStages.forEach(stage => {
            const stageData = this.robotData.filter(d => d.job_stage === stage);
            if (stageData.length < 5) return;
            
            const velocities = stageData.map(d => d.velocity_magnitude);
            const distanceErrors = stageData.map(d => d.distance_error).filter(d => d > 0);
            
            metrics.push({
                Job_Stage: stage,
                Sample_Count: stageData.length,
                Average_Velocity_ms: this.calculateMean(velocities),
                Average_Distance_Error_m: distanceErrors.length > 0 ? this.calculateMean(distanceErrors) : 0,
                Distance_Traveled_m: stageData.reduce((sum, d) => sum + d.distance_traveled, 0),
                Duration_Seconds: stageData.length > 0 ? 
                    stageData[stageData.length - 1].time_seconds - stageData[0].time_seconds : 0
            });
        });
        
        return metrics;
    }
    
    // UI Update Methods
    updateCollectionStatus() {
        const statusElement = document.getElementById('collection-status');
        if (statusElement) {
            statusElement.textContent = this.isCollecting ? 'RECORDING' : 'STOPPED';
            statusElement.style.color = this.isCollecting ? '#ff4444' : '#00ff88';
        }
    }
    
    updateCollectionDisplay() {
        // Update record count
        const recordElement = document.getElementById('record-count');
        if (recordElement) {
            recordElement.textContent = this.robotData.length;
        }
        
        // Update duration
        const durationElement = document.getElementById('collection-duration');
        if (durationElement && this.collectionStartTime) {
            const elapsed = new Date() - this.collectionStartTime;
            const minutes = Math.floor(elapsed / 60000);
            const seconds = Math.floor((elapsed % 60000) / 1000);
            durationElement.textContent = 
                minutes.toString().padStart(2, '0') + ':' + seconds.toString().padStart(2, '0');
        }
        
        // Update data size estimate
        const sizeElement = document.getElementById('data-size');
        if (sizeElement) {
            const estimatedSize = this.robotData.length * 300; // Rough estimate
            const sizeKB = (estimatedSize / 1024).toFixed(1);
            sizeElement.textContent = sizeKB + ' KB';
        }
    }
    
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
            row.insertCell(3).textContent = record.velocity_magnitude.toFixed(2);
        });
    }
    
    // Utility Methods
    toExcelDate(date) {
        // Convert JavaScript date to Excel serial date
        return date.getTime() / 86400000 + 25569;
    }
    
    calculateMean(values) {
        return values.length > 0 ? values.reduce((a, b) => a + b, 0) / values.length : 0;
    }
    
    calculateStandardDeviation(values) {
        if (values.length === 0) return 0;
        const mean = this.calculateMean(values);
        const squareDiffs = values.map(value => Math.pow(value - mean, 2));
        const avgSquareDiff = this.calculateMean(squareDiffs);
        return Math.sqrt(avgSquareDiff);
    }
    
    getMetricUnit(metric) {
        const units = {
            'total_distance_m': 'm',
            'average_velocity_ms': 'm/s',
            'max_velocity_ms': 'm/s',
            'min_velocity_ms': 'm/s',
            'velocity_std_dev': 'm/s',
            'average_linear_velocity_ms': 'm/s',
            'average_angular_velocity_rads': 'rad/s',
            'average_distance_error_m': 'm',
            'max_distance_error_m': 'm',
            'average_heading_error_deg': 'deg',
            'max_heading_error_deg': 'deg',
            'workspace_x_range': 'm',
            'workspace_y_range': 'm',
            'collection_duration_s': 's',
            'data_collection_rate_hz': 'Hz'
        };
        return units[metric] || '';
    }
    
    generateFilename() {
        const now = new Date();
        const dateStr = now.toISOString().slice(0, 19).replace(/:/g, '-');
        const controlMode = this.mhs.currentControlMode;
        const recordCount = this.robotData.length;
        
        return `material_handling_${controlMode}_${dateStr}_${recordCount}pts.xlsx`;
    }
    
    showExportSuccess(filename) {
        const stats = this.calculateStatistics();
        const sheetCount = 6 + (this.jobEvents.length > 0 ? 1 : 0) + (this.systemEvents.length > 0 ? 1 : 0);
        
        alert(`Excel export completed successfully!

Filename: ${filename}
Records: ${this.robotData.length}
Duration: ${stats.collection_duration_s.toFixed(1)}s
Distance: ${stats.total_distance_m.toFixed(2)}m
Sheets: ${sheetCount}

Sheets included:
• Robot Data (main dataset)
• Statistics (summary metrics)
• Control Analysis (algorithm comparison)
• Performance Metrics (stage analysis)
• Excel Formulas (ready-to-use formulas)
${this.jobEvents.length > 0 ? '• Job Events (pickup/dropoff timeline)\n' : ''}${this.systemEvents.length > 0 ? '• System Events (state changes)\n' : ''}`);
    }
    
    // Settings Management
    updateCollectionRate(newRate) {
        this.collectionRate = Math.max(1, Math.min(50, newRate));
        this.setupCollectionInterval();
        this.mhs.logMessage(`📊 Data collection rate updated to ${this.collectionRate} Hz`);
    }
    
    setAutoExport(enabled) {
        this.autoExportOnJobComplete = enabled;
        this.mhs.logMessage(`📊 Auto-export ${enabled ? 'enabled' : 'disabled'}`);
    }
    
    setIncludePerformanceData(enabled) {
        this.includePerformanceData = enabled;
        this.mhs.logMessage(`📊 Performance data inclusion ${enabled ? 'enabled' : 'disabled'}`);
    }
    
    // Public API Methods
    getDataStats() {
        return {
            isCollecting: this.isCollecting,
            recordCount: this.robotData.length,
            jobEventCount: this.jobEvents.length,
            systemEventCount: this.systemEvents.length,
            collectionDuration: this.collectionStartTime ? 
                (new Date() - this.collectionStartTime) / 1000 : 0,
            collectionRate: this.collectionRate
        };
    }
    
    exportData() {
        return {
            robotData: [...this.robotData],
            jobEvents: [...this.jobEvents],
            systemEvents: [...this.systemEvents],
            metadata: {
                collectionStartTime: this.collectionStartTime,
                collectionRate: this.collectionRate,
                version: '1.0'
            }
        };
    }
    
    importData(data) {
        if (this.isCollecting) {
            this.stopDataCollection();
        }
        
        this.robotData = data.robotData || [];
        this.jobEvents = data.jobEvents || [];
        this.systemEvents = data.systemEvents || [];
        this.collectionStartTime = data.metadata?.collectionStartTime || null;
        
        this.updateCollectionDisplay();
        this.updateDataPreview();
        this.mhs.logMessage(`📊 Data imported: ${this.robotData.length} records`);
    }
}

// Make ExcelIntegration available globally
window.ExcelIntegration = ExcelIntegration;