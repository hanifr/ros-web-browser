/**
 * Job Manager Module
 * Handles job creation, execution, and queue management for the material handling system
 */

class JobManager {
    constructor(materialHandlingSystem) {
        this.mhs = materialHandlingSystem;
        this.jobIdCounter = 1;
        this.init();
    }
    
    init() {
        console.log('Job Manager initialized');
    }
    
    // Job Creation and Management
    addJob(pickup, dropoff, quantity) {
        const job = {
            id: this.jobIdCounter++,
            pickup: pickup,
            dropoff: dropoff,
            quantity: quantity,
            status: 'QUEUED',
            stage: 'PICKUP',
            created: new Date().toLocaleTimeString(),
            createdTimestamp: Date.now()
        };
        
        this.mhs.jobQueue.push(job);
        this.updateJobQueueDisplay();
        
        this.mhs.logMessage(`➕ Job added: Move ${quantity} items from ${this.mhs.locations[pickup].name} to ${this.mhs.locations[dropoff].name}`);
        
        return job;
    }
    
    startNextJob() {
        if (this.mhs.jobQueue.length === 0 || this.mhs.systemState !== 'RUNNING') {
            return false;
        }
        
        this.mhs.currentJob = this.mhs.jobQueue.shift();
        this.mhs.currentJob.status = 'ACTIVE';
        this.mhs.currentJob.stage = 'PICKUP';
        this.mhs.currentJob.startTime = Date.now();
        
        this.updateCurrentJobDisplay();
        this.updateJobQueueDisplay();
        
        this.mhs.logMessage(`🚀 Starting job ${this.mhs.currentJob.id}: Move ${this.mhs.currentJob.quantity} items from ${this.mhs.locations[this.mhs.currentJob.pickup].name} to ${this.mhs.locations[this.mhs.currentJob.dropoff].name}`);
        
        // Start navigation to pickup location
        this.mhs.startNavigation(this.mhs.currentJob.pickup);
        
        return true;
    }
    
    checkJobProgress() {
        if (!this.mhs.currentJob || this.mhs.currentControlMode === 'state_machine') {
            return;
        }
        
        const targetLocation = (this.mhs.currentJob.stage === 'PICKUP') ? 
                              this.mhs.currentJob.pickup : this.mhs.currentJob.dropoff;
        const target = this.mhs.locations[targetLocation];
        
        const distance = Math.sqrt(
            Math.pow(this.mhs.robotPosition.x - target.x, 2) + 
            Math.pow(this.mhs.robotPosition.y - target.y, 2)
        );
        
        if (distance < 0.25 && (!this.mhs.currentJob.arrivedAt || this.mhs.currentJob.arrivedAt !== targetLocation)) {
            this.mhs.currentJob.arrivedAt = targetLocation;
            this.mhs.stopNavigation();
            
            if (this.mhs.currentJob.stage === 'PICKUP') {
                this.handlePickupArrival();
            } else if (this.mhs.currentJob.stage === 'TRANSPORT') {
                this.handleDropoffArrival();
            }
        }
    }
    
    handlePickupArrival() {
        const job = this.mhs.currentJob;
        this.mhs.logMessage(`🔄 Arriving at ${this.mhs.locations[job.pickup].name} for pickup...`);
        
        setTimeout(() => {
            this.mhs.logMessage(`📦 Picking up ${job.quantity} items from ${this.mhs.locations[job.pickup].name}`);
            this.mhs.materialCounts[job.pickup] -= job.quantity;
            this.mhs.updateMaterialCounts();
            
            job.stage = 'TRANSPORT';
            job.pickupTime = Date.now();
            delete job.arrivedAt;
            
            this.updateCurrentJobDisplay();
            
            setTimeout(() => {
                this.mhs.logMessage('🚚 Pickup complete, moving to dropoff location');
                this.mhs.startNavigation(job.dropoff);
            }, 1500);
        }, 1000);
    }
    
    handleDropoffArrival() {
        const job = this.mhs.currentJob;
        this.mhs.logMessage(`🔄 Arriving at ${this.mhs.locations[job.dropoff].name} for dropoff...`);
        
        setTimeout(() => {
            this.mhs.logMessage(`📦 Dropping off ${job.quantity} items at ${this.mhs.locations[job.dropoff].name}`);
            this.mhs.materialCounts[job.dropoff] += job.quantity;
            this.mhs.updateMaterialCounts();
            
            job.status = 'COMPLETED';
            job.completionTime = Date.now();
            job.totalTime = job.completionTime - job.startTime;
            
            this.completeCurrentJob();
            
        }, 1000);
    }
    
    completeCurrentJob() {
        if (!this.mhs.currentJob) return;
        
        const job = this.mhs.currentJob;
        this.mhs.jobsCompleted++;
        
        // Update completed jobs counter
        const completedElement = document.getElementById('jobs-completed');
        if (completedElement) {
            completedElement.textContent = this.mhs.jobsCompleted;
        }
        
        // Log completion with timing information
        const totalTimeSeconds = (job.totalTime / 1000).toFixed(1);
        this.mhs.logMessage(`✅ Job ${job.id} completed in ${totalTimeSeconds}s: ${job.quantity} items moved from ${this.mhs.locations[job.pickup].name} to ${this.mhs.locations[job.dropoff].name}`);
        
        // Record job completion for Excel export
        if (this.mhs.excelIntegration) {
            this.mhs.excelIntegration.recordJobCompletion(job);
        }
        
        // Clear current job
        this.mhs.currentJob = null;
        this.updateCurrentJobDisplay();
        
        // Start next job after delay
        setTimeout(() => {
            this.startNextJob();
        }, 2000);
    }
    
    cancelCurrentJob() {
        if (!this.mhs.currentJob) return;
        
        this.mhs.currentJob.status = 'CANCELLED';
        this.mhs.currentJob.cancellationTime = Date.now();
        
        // Return materials if pickup was completed
        if (this.mhs.currentJob.stage === 'TRANSPORT') {
            this.mhs.materialCounts[this.mhs.currentJob.pickup] += this.mhs.currentJob.quantity;
            this.mhs.updateMaterialCounts();
        }
        
        this.mhs.logMessage(`❌ Job ${this.mhs.currentJob.id} cancelled`);
        
        // Add back to front of queue if needed
        this.mhs.jobQueue.unshift(this.mhs.currentJob);
        this.mhs.currentJob = null;
        
        this.updateCurrentJobDisplay();
        this.updateJobQueueDisplay();
    }
    
    removeJobFromQueue(jobId) {
        const index = this.mhs.jobQueue.findIndex(job => job.id === jobId);
        if (index !== -1) {
            const removedJob = this.mhs.jobQueue.splice(index, 1)[0];
            this.mhs.logMessage(`🗑️ Job ${jobId} removed from queue`);
            this.updateJobQueueDisplay();
            return removedJob;
        }
        return null;
    }
    
    clearAllJobs() {
        this.mhs.jobQueue = [];
        if (this.mhs.currentJob) {
            this.cancelCurrentJob();
        }
        this.updateJobQueueDisplay();
        this.mhs.logMessage('🗑️ All jobs cleared');
    }
    
    reorderJob(jobId, newPosition) {
        const jobIndex = this.mhs.jobQueue.findIndex(job => job.id === jobId);
        if (jobIndex === -1 || newPosition < 0 || newPosition >= this.mhs.jobQueue.length) {
            return false;
        }
        
        const job = this.mhs.jobQueue.splice(jobIndex, 1)[0];
        this.mhs.jobQueue.splice(newPosition, 0, job);
        
        this.updateJobQueueDisplay();
        this.mhs.logMessage(`📝 Job ${jobId} moved to position ${newPosition + 1}`);
        return true;
    }
    
    prioritizeJob(jobId) {
        const jobIndex = this.mhs.jobQueue.findIndex(job => job.id === jobId);
        if (jobIndex === -1) return false;
        
        const job = this.mhs.jobQueue.splice(jobIndex, 1)[0];
        this.mhs.jobQueue.unshift(job);
        
        this.updateJobQueueDisplay();
        this.mhs.logMessage(`⚡ Job ${jobId} prioritized`);
        return true;
    }
    
    // Display Update Methods
    updateCurrentJobDisplay() {
        const currentJobElement = document.getElementById('current-job');
        if (!currentJobElement) return;
        
        if (!this.mhs.currentJob) {
            currentJobElement.textContent = 'None';
            currentJobElement.className = 'current-job-display';
            return;
        }
        
        const job = this.mhs.currentJob;
        const pickupName = this.mhs.locations[job.pickup].name;
        const dropoffName = this.mhs.locations[job.dropoff].name;
        
        // Calculate elapsed time if job has started
        let timeInfo = '';
        if (job.startTime) {
            const elapsed = (Date.now() - job.startTime) / 1000;
            timeInfo = ` (${elapsed.toFixed(0)}s)`;
        }
        
        currentJobElement.innerHTML = `
            <div class="job-header">
                <strong>Job ${job.id}: Moving ${job.quantity} items</strong>
                <span class="job-time">${timeInfo}</span>
            </div>
            <div class="job-route">${pickupName} → ${dropoffName}</div>
            <div class="job-stage">Stage: <span class="stage-${job.stage.toLowerCase()}">${job.stage}</span></div>
        `;
        
        currentJobElement.className = `current-job-display active-job-${job.stage.toLowerCase()}`;
    }
    
    updateJobQueueDisplay() {
        const queueDiv = document.getElementById('job-queue');
        if (!queueDiv) return;
        
        queueDiv.innerHTML = '';
        
        // Show current job if exists
        if (this.mhs.currentJob) {
            const jobDiv = this.createJobQueueItem(this.mhs.currentJob, true);
            queueDiv.appendChild(jobDiv);
        }
        
        // Show queued jobs
        this.mhs.jobQueue.forEach((job, index) => {
            const jobDiv = this.createJobQueueItem(job, false, index + 1);
            queueDiv.appendChild(jobDiv);
        });
        
        // Show empty state if no jobs
        if (!this.mhs.currentJob && this.mhs.jobQueue.length === 0) {
            const emptyDiv = document.createElement('div');
            emptyDiv.className = 'no-jobs-message';
            emptyDiv.innerHTML = '<p style="text-align: center; color: #666; font-style: italic; padding: 20px;">No jobs in queue</p>';
            queueDiv.appendChild(emptyDiv);
        }
    }
    
    createJobQueueItem(job, isActive, queuePosition = null) {
        const jobDiv = document.createElement('div');
        jobDiv.className = `job-item ${isActive ? 'active' : ''}${job.status === 'completed' ? ' completed' : ''}`;
        jobDiv.dataset.jobId = job.id;
        
        const pickupName = this.mhs.locations[job.pickup].name;
        const dropoffName = this.mhs.locations[job.dropoff].name;
        
        let statusText = '';
        if (isActive) {
            statusText = `🔥 ACTIVE (${job.stage})`;
        } else if (queuePosition) {
            statusText = `Job ${queuePosition}`;
        }
        
        // Calculate estimated time or actual time
        let timeInfo = '';
        if (job.completionTime) {
            const totalTime = (job.completionTime - job.startTime) / 1000;
            timeInfo = `<small>Completed in ${totalTime.toFixed(1)}s</small>`;
        } else if (job.startTime) {
            const elapsed = (Date.now() - job.startTime) / 1000;
            timeInfo = `<small>Running for ${elapsed.toFixed(0)}s</small>`;
        } else {
            timeInfo = `<small>Created: ${job.created}</small>`;
        }
        
        jobDiv.innerHTML = `
            <div class="job-item-header">
                <strong>${statusText}</strong>
                ${!isActive ? `<button class="job-remove-btn" onclick="window.materialHandling.jobManager.removeJobFromQueue(${job.id})" title="Remove job">✕</button>` : ''}
            </div>
            <div class="job-item-details">
                Move ${job.quantity} items<br>
                ${pickupName} → ${dropoffName}<br>
                ${timeInfo}
            </div>
        `;
        
        // Add drag and drop for queue reordering (only for queued jobs)
        if (!isActive && queuePosition) {
            jobDiv.draggable = true;
            jobDiv.addEventListener('dragstart', (e) => {
                e.dataTransfer.setData('text/plain', job.id);
                jobDiv.classList.add('dragging');
            });
            
            jobDiv.addEventListener('dragend', () => {
                jobDiv.classList.remove('dragging');
            });
            
            jobDiv.addEventListener('dragover', (e) => {
                e.preventDefault();
                jobDiv.classList.add('drag-over');
            });
            
            jobDiv.addEventListener('dragleave', () => {
                jobDiv.classList.remove('drag-over');
            });
            
            jobDiv.addEventListener('drop', (e) => {
                e.preventDefault();
                jobDiv.classList.remove('drag-over');
                
                const draggedJobId = parseInt(e.dataTransfer.getData('text/plain'));
                const targetJobIndex = this.mhs.jobQueue.findIndex(j => j.id === job.id);
                
                if (draggedJobId !== job.id) {
                    this.reorderJob(draggedJobId, targetJobIndex);
                }
            });
            
            // Add priority button
            const priorityBtn = document.createElement('button');
            priorityBtn.className = 'job-priority-btn';
            priorityBtn.innerHTML = '⚡';
            priorityBtn.title = 'Prioritize this job';
            priorityBtn.onclick = () => this.prioritizeJob(job.id);
            jobDiv.querySelector('.job-item-header').appendChild(priorityBtn);
        }
        
        return jobDiv;
    }
    
    // Job Statistics and Analytics
    getJobStatistics() {
        const completedJobs = this.getCompletedJobs();
        
        if (completedJobs.length === 0) {
            return {
                totalJobs: 0,
                averageTime: 0,
                totalItems: 0,
                averageItemsPerJob: 0,
                successRate: 0
            };
        }
        
        const totalTime = completedJobs.reduce((sum, job) => sum + job.totalTime, 0);
        const totalItems = completedJobs.reduce((sum, job) => sum + job.quantity, 0);
        
        return {
            totalJobs: completedJobs.length,
            averageTime: totalTime / completedJobs.length / 1000, // in seconds
            totalItems: totalItems,
            averageItemsPerJob: totalItems / completedJobs.length,
            successRate: (completedJobs.length / (completedJobs.length + this.getCancelledJobs().length)) * 100
        };
    }
    
    getCompletedJobs() {
        // This would typically be stored in a database or persistent storage
        // For now, we'll return an empty array as jobs are cleared on page reload
        return [];
    }
    
    getCancelledJobs() {
        // This would typically be stored in a database or persistent storage
        return [];
    }
    
    estimateJobTime(pickup, dropoff, quantity) {
        // Simple estimation based on distance and quantity
        const pickupLoc = this.mhs.locations[pickup];
        const dropoffLoc = this.mhs.locations[dropoff];
        
        const distance = Math.sqrt(
            Math.pow(dropoffLoc.x - pickupLoc.x, 2) + 
            Math.pow(dropoffLoc.y - pickupLoc.y, 2)
        );
        
        // Estimate: base time + travel time + handling time
        const baseTime = 5; // seconds
        const travelTime = distance * 8; // seconds per meter (conservative)
        const handlingTime = quantity * 2; // seconds per item
        
        return baseTime + travelTime + handlingTime;
    }
    
    // Public API Methods
    getJobQueue() {
        return [...this.mhs.jobQueue];
    }
    
    getCurrentJob() {
        return this.mhs.currentJob ? { ...this.mhs.currentJob } : null;
    }
    
    getJobById(jobId) {
        const queueJob = this.mhs.jobQueue.find(job => job.id === jobId);
        if (queueJob) return { ...queueJob };
        
        if (this.mhs.currentJob && this.mhs.currentJob.id === jobId) {
            return { ...this.mhs.currentJob };
        }
        
        return null;
    }
    
    validateJob(pickup, dropoff, quantity) {
        const errors = [];
        
        if (!pickup || !this.mhs.locations[pickup]) {
            errors.push('Invalid pickup location');
        }
        
        if (!dropoff || !this.mhs.locations[dropoff]) {
            errors.push('Invalid dropoff location');
        }
        
        if (pickup === dropoff) {
            errors.push('Pickup and dropoff locations cannot be the same');
        }
        
        if (!quantity || quantity < 1 || quantity > 10) {
            errors.push('Quantity must be between 1 and 10');
        }
        
        if (pickup && this.mhs.materialCounts[pickup] < quantity) {
            errors.push(`Not enough items at ${this.mhs.locations[pickup].name}. Available: ${this.mhs.materialCounts[pickup]}`);
        }
        
        return {
            isValid: errors.length === 0,
            errors: errors
        };
    }
}

// Make JobManager available globally
window.JobManager = JobManager;