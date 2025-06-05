/**
 * Control Algorithms Module
 * Implements various robot control algorithms for navigation
 */

class ControlAlgorithms {
    constructor() {
        this.navigationInterval = null;
        this.currentTarget = null;
        this.robotPosition = { x: 0, y: 0 };
        this.robotOrientation = 0;
        
        // Control Algorithm Parameters
        this.controlParams = {
            proportional: {
                kp_linear: 0.8,
                kp_angular: 2.5,
                max_linear: 0.6,
                max_angular: 1.2
            },
            pid: {
                kp_linear: 1.0, ki_linear: 0.15, kd_linear: 0.08,
                kp_angular: 3.5, ki_angular: 0.25, kd_angular: 0.12,
                max_linear: 0.7, max_angular: 1.5,
                integral_limit: 1.0
            },
            pure_pursuit: {
                lookahead_distance: 0.6,
                max_linear: 0.65,
                max_angular: 1.3,
                kp_linear: 0.9
            },
            state_machine: {
                arrival_tolerance: 0.18,
                heading_tolerance: 0.2,
                max_linear: 0.5,
                max_angular: 1.0
            }
        };
        
        // PID Controller State
        this.pidState = {
            prevLinearError: 0,
            prevAngularError: 0,
            linearErrorIntegral: 0,
            angularErrorIntegral: 0,
            dt: 0.1
        };
        
        // Velocity Smoother
        this.velocitySmoother = {
            currentLinear: 0,
            currentAngular: 0,
            maxLinearAccel: 0.8,
            maxAngularAccel: 1.5,
            dt: 0.1,
            
            smooth: (targetLinear, targetAngular) => {
                const maxLinearChange = this.velocitySmoother.maxLinearAccel * this.velocitySmoother.dt;
                const maxAngularChange = this.velocitySmoother.maxAngularAccel * this.velocitySmoother.dt;
                
                let linearChange = targetLinear - this.velocitySmoother.currentLinear;
                let angularChange = targetAngular - this.velocitySmoother.currentAngular;
                
                linearChange = Math.max(Math.min(linearChange, maxLinearChange), -maxLinearChange);
                angularChange = Math.max(Math.min(angularChange, maxAngularChange), -maxAngularChange);
                
                this.velocitySmoother.currentLinear += linearChange;
                this.velocitySmoother.currentAngular += angularChange;
                
                return {
                    linear: this.velocitySmoother.currentLinear,
                    angular: this.velocitySmoother.currentAngular
                };
            }
        };
        
        // Robot State Machine
        this.robotStateMachine = {
            state: 'IDLE', // IDLE, NAVIGATING, PICKING, DROPPING, ERROR
            targetLocation: null,
            stateStartTime: 0,
            
            setState: (newState) => {
                this.robotStateMachine.state = newState;
                this.robotStateMachine.stateStartTime = Date.now();
                console.log(`State changed to: ${newState}`);
            },
            
            hasArrived: () => {
                if (!this.currentTarget) return false;
                const distance = Math.sqrt(
                    Math.pow(this.robotPosition.x - this.currentTarget.x, 2) + 
                    Math.pow(this.robotPosition.y - this.currentTarget.y, 2)
                );
                return distance < this.controlParams.state_machine.arrival_tolerance;
            }
        };
        
        this.init();
    }
    
    init() {
        console.log('Control Algorithms initialized');
    }
    
    // Navigation Control Methods
    startNavigation(locationId, target) {
        if (this.navigationInterval) {
            clearInterval(this.navigationInterval);
        }
        
        this.currentTarget = target;
        console.log(`Starting navigation to ${target.name} using control algorithm`);
        
        this.resetPIDState();
        
        this.navigationInterval = setInterval(() => {
            this.updateNavigationControl();
        }, 100); // 10 Hz update rate
    }
    
    stopNavigation() {
        if (this.navigationInterval) {
            clearInterval(this.navigationInterval);
            this.navigationInterval = null;
        }
        this.currentTarget = null;
    }
    
    updateNavigationControl() {
        if (!this.currentTarget) return;
        
        const dx = this.currentTarget.x - this.robotPosition.x;
        const dy = this.currentTarget.y - this.robotPosition.y;
        const distance = Math.sqrt(dx*dx + dy*dy);
        
        // Update distance error for display
        if (window.materialHandling) {
            window.materialHandling.distanceError = distance;
        }
        
        if (distance < 0.15) {
            this.stopNavigation();
            console.log(`Arrived at ${this.currentTarget.name}`);
            return;
        }
        
        const mode = window.materialHandling?.currentControlMode || 'proportional';
        let velocities;
        
        switch(mode) {
            case 'proportional':
                velocities = this.calculateProportionalControl(dx, dy, distance);
                break;
            case 'pid':
                velocities = this.calculatePIDControl(dx, dy, distance);
                break;
            case 'pure_pursuit':
                velocities = this.calculatePurePursuitControl(dx, dy, distance);
                break;
            case 'state_machine':
                velocities = this.calculateStateMachineControl(dx, dy, distance);
                break;
            default:
                velocities = { linear: 0, angular: 0 };
        }
        
        // Apply velocity smoothing
        const smoothedVel = this.velocitySmoother.smooth(velocities.linear, velocities.angular);
        
        // Publish velocities
        this.publishVelocity(smoothedVel.linear, smoothedVel.angular);
    }
    
    // Control Algorithm Implementations
    calculateProportionalControl(dx, dy, distance) {
        const params = this.controlParams.proportional;
        
        const targetHeading = Math.atan2(dy, dx);
        const headingError = this.normalizeAngle(targetHeading - this.robotOrientation);
        
        const linearVel = Math.min(params.kp_linear * distance, params.max_linear);
        const angularVel = Math.max(Math.min(params.kp_angular * headingError, params.max_angular), -params.max_angular);
        
        // Store heading error for display
        if (window.materialHandling) {
            window.materialHandling.headingError = headingError * 180 / Math.PI;
        }
        
        return { linear: linearVel, angular: angularVel };
    }
    
    calculatePIDControl(dx, dy, distance) {
        const params = this.controlParams.pid;
        
        const targetHeading = Math.atan2(dy, dx);
        const headingError = this.normalizeAngle(targetHeading - this.robotOrientation);
        
        // PID for linear velocity
        this.pidState.linearErrorIntegral += distance * this.pidState.dt;
        this.pidState.linearErrorIntegral = Math.max(Math.min(this.pidState.linearErrorIntegral, params.integral_limit), -params.integral_limit);
        
        const linearErrorDerivative = (distance - this.pidState.prevLinearError) / this.pidState.dt;
        let linearVel = params.kp_linear * distance + 
                       params.ki_linear * this.pidState.linearErrorIntegral + 
                       params.kd_linear * linearErrorDerivative;
        
        // PID for angular velocity
        this.pidState.angularErrorIntegral += headingError * this.pidState.dt;
        this.pidState.angularErrorIntegral = Math.max(Math.min(this.pidState.angularErrorIntegral, params.integral_limit), -params.integral_limit);
        
        const angularErrorDerivative = (headingError - this.pidState.prevAngularError) / this.pidState.dt;
        let angularVel = params.kp_angular * headingError + 
                        params.ki_angular * this.pidState.angularErrorIntegral + 
                        params.kd_angular * angularErrorDerivative;
        
        // Store previous errors
        this.pidState.prevLinearError = distance;
        this.pidState.prevAngularError = headingError;
        
        // Apply limits
        linearVel = Math.max(Math.min(linearVel, params.max_linear), 0);
        angularVel = Math.max(Math.min(angularVel, params.max_angular), -params.max_angular);
        
        // Store heading error for display
        if (window.materialHandling) {
            window.materialHandling.headingError = headingError * 180 / Math.PI;
        }
        
        return { linear: linearVel, angular: angularVel };
    }
    
    calculatePurePursuitControl(dx, dy, distance) {
        const params = this.controlParams.pure_pursuit;
        
        // Transform target to robot local frame
        const cos_theta = Math.cos(this.robotOrientation);
        const sin_theta = Math.sin(this.robotOrientation);
        
        const local_x = cos_theta * dx + sin_theta * dy;
        const local_y = -sin_theta * dx + cos_theta * dy;
        
        // Calculate curvature using Pure Pursuit formula
        const L = Math.min(params.lookahead_distance, distance);
        const curvature = (L > 0.01) ? 2 * local_y / (L * L) : 0;
        
        // Calculate velocities
        let linearVel = Math.min(params.kp_linear * distance, params.max_linear);
        let angularVel = curvature * linearVel;
        
        // Limit angular velocity
        angularVel = Math.max(Math.min(angularVel, params.max_angular), -params.max_angular);
        
        // Store heading error for display
        if (window.materialHandling) {
            window.materialHandling.headingError = Math.atan2(local_y, local_x) * 180 / Math.PI;
        }
        
        return { linear: linearVel, angular: angularVel };
    }
    
    calculateStateMachineControl(dx, dy, distance) {
        const params = this.controlParams.state_machine;
        
        const targetHeading = Math.atan2(dy, dx);
        const headingError = this.normalizeAngle(targetHeading - this.robotOrientation);
        
        // State-based control
        let linearVel = 0;
        let angularVel = 0;
        
        if (Math.abs(headingError) > params.heading_tolerance) {
            // First align heading
            angularVel = Math.sign(headingError) * Math.min(Math.abs(headingError) * 2.0, params.max_angular);
        } else {
            // Then move forward
            linearVel = Math.min(distance * 1.5, params.max_linear);
            angularVel = headingError * 1.0; // Small correction
        }
        
        // Store heading error for display
        if (window.materialHandling) {
            window.materialHandling.headingError = headingError * 180 / Math.PI;
        }
        
        return { linear: linearVel, angular: angularVel };
    }
    
    // Utility Methods
    normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    resetPIDState() {
        this.pidState.prevLinearError = 0;
        this.pidState.prevAngularError = 0;
        this.pidState.linearErrorIntegral = 0;
        this.pidState.angularErrorIntegral = 0;
    }
    
    publishVelocity(linear, angular) {
        if (window.materialHandling?.rosBridge) {
            window.materialHandling.rosBridge.publishVelocity(linear, angular);
        }
        
        // Store current velocities for monitoring
        if (window.materialHandling) {
            window.materialHandling.robotVelocity.linear = linear;
            window.materialHandling.robotVelocity.angular = angular;
        }
    }
    
    updateRobotPose(position, orientation) {
        this.robotPosition = { ...position };
        this.robotOrientation = orientation;
    }
    
    // Parameter Management
    updateParameterDisplay(currentMode) {
        const paramsDiv = document.getElementById('algorithm-params');
        if (!paramsDiv) return;
        
        const params = this.controlParams[currentMode];
        let html = '';
        
        switch(currentMode) {
            case 'proportional':
                html = `
                    <div class="param-row">
                        <label>Linear Gain (Kp):</label>
                        <input type="number" step="0.1" value="${params.kp_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.proportional.kp_linear = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Angular Gain (Kp):</label>
                        <input type="number" step="0.1" value="${params.kp_angular}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.proportional.kp_angular = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Max Linear Speed:</label>
                        <input type="number" step="0.1" value="${params.max_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.proportional.max_linear = parseFloat(this.value)" class="param-input">
                    </div>
                `;
                break;
            case 'pid':
                html = `
                    <div class="param-row">
                        <label>Linear Kp:</label>
                        <input type="number" step="0.1" value="${params.kp_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pid.kp_linear = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Linear Ki:</label>
                        <input type="number" step="0.01" value="${params.ki_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pid.ki_linear = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Linear Kd:</label>
                        <input type="number" step="0.01" value="${params.kd_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pid.kd_linear = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Angular Kp:</label>
                        <input type="number" step="0.1" value="${params.kp_angular}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pid.kp_angular = parseFloat(this.value)" class="param-input">
                    </div>
                `;
                break;
            case 'pure_pursuit':
                html = `
                    <div class="param-row">
                        <label>Lookahead Distance:</label>
                        <input type="number" step="0.1" value="${params.lookahead_distance}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pure_pursuit.lookahead_distance = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Linear Gain:</label>
                        <input type="number" step="0.1" value="${params.kp_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pure_pursuit.kp_linear = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Max Linear Speed:</label>
                        <input type="number" step="0.1" value="${params.max_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.pure_pursuit.max_linear = parseFloat(this.value)" class="param-input">
                    </div>
                `;
                break;
            case 'state_machine':
                html = `
                    <div class="param-row">
                        <label>Arrival Tolerance:</label>
                        <input type="number" step="0.05" value="${params.arrival_tolerance}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.state_machine.arrival_tolerance = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Heading Tolerance:</label>
                        <input type="number" step="0.05" value="${params.heading_tolerance}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.state_machine.heading_tolerance = parseFloat(this.value)" class="param-input">
                    </div>
                    <div class="param-row">
                        <label>Max Linear Speed:</label>
                        <input type="number" step="0.1" value="${params.max_linear}" 
                               onchange="window.materialHandling.controlAlgorithms.controlParams.state_machine.max_linear = parseFloat(this.value)" class="param-input">
                    </div>
                `;
                break;
        }
        
        paramsDiv.innerHTML = html;
    }
    
    // Public API
    getControlParameters(mode) {
        return { ...this.controlParams[mode] };
    }
    
    setControlParameter(mode, param, value) {
        if (this.controlParams[mode] && this.controlParams[mode].hasOwnProperty(param)) {
            this.controlParams[mode][param] = value;
            return true;
        }
        return false;
    }
    
    getCurrentTarget() {
        return this.currentTarget;
    }
    
    isNavigating() {
        return this.navigationInterval !== null;
    }
}

// Make ControlAlgorithms available globally
window.ControlAlgorithms = ControlAlgorithms;