/**
 * Platform Core Module
 * Provides shared utilities, configuration management, and core functionality
 * for the RoboTech Platform
 */

class PlatformCore {
    constructor() {
        this.version = '3.0.0';
        this.initialized = false;
        this.modules = new Map();
        this.eventListeners = new Map();
        this.config = this.getDefaultConfig();
        
        this.init();
    }
    
    init() {
        console.log(`🚀 Initializing RoboTech Platform Core v${this.version}`);
        
        // Load configuration from localStorage
        this.loadConfiguration();
        
        // Initialize error handling
        this.initializeErrorHandling();
        
        // Initialize performance monitoring
        this.initializePerformanceMonitoring();
        
        // Set up periodic tasks
        this.setupPeriodicTasks();
        
        this.initialized = true;
        console.log('✅ Platform Core initialized successfully');
    }
    
    getDefaultConfig() {
        return {
            ros: {
                websocket_url: 'ws://localhost:9090',
                connection_timeout: 5000,
                reconnect_interval: 2000,
                max_reconnect_attempts: 10
            },
            control: {
                update_rate: 10, // Hz
                velocity_smoothing: true,
                adaptive_control: true,
                emergency_stop_timeout: 1000
            },
            ui: {
                theme: 'dark',
                animations_enabled: true,
                performance_overlay: false,
                log_level: 'info'
            },
            data_collection: {
                auto_export: true,
                collection_rate: 5, // Hz
                include_performance: true,
                max_records: 10000
            },
            navigation: {
                default_tolerance: 0.15,
                max_linear_velocity: 0.8,
                max_angular_velocity: 1.5,
                path_smoothing: true
            },
            safety: {
                collision_avoidance: true,
                emergency_stop_enabled: true,
                workspace_limits: {
                    x_min: -2.0, x_max: 4.0,
                    y_min: -2.0, y_max: 4.0
                }
            }
        };
    }
    
    // Configuration Management
    loadConfiguration() {
        try {
            const savedConfig = localStorage.getItem('robotech_platform_config');
            if (savedConfig) {
                const parsed = JSON.parse(savedConfig);
                this.config = this.mergeConfigs(this.config, parsed);
                console.log('📋 Configuration loaded from localStorage');
            }
        } catch (error) {
            console.warn('⚠️ Failed to load configuration, using defaults:', error);
        }
    }
    
    saveConfiguration() {
        try {
            localStorage.setItem('robotech_platform_config', JSON.stringify(this.config));
            console.log('💾 Configuration saved to localStorage');
            return true;
        } catch (error) {
            console.error('❌ Failed to save configuration:', error);
            return false;
        }
    }
    
    mergeConfigs(defaultConfig, userConfig) {
        const merged = { ...defaultConfig };
        
        Object.keys(userConfig).forEach(key => {
            if (typeof userConfig[key] === 'object' && userConfig[key] !== null && !Array.isArray(userConfig[key])) {
                merged[key] = this.mergeConfigs(defaultConfig[key] || {}, userConfig[key]);
            } else {
                merged[key] = userConfig[key];
            }
        });
        
        return merged;
    }
    
    getConfig(path) {
        return this.getNestedValue(this.config, path);
    }
    
    setConfig(path, value) {
        this.setNestedValue(this.config, path, value);
        this.saveConfiguration();
        this.emit('config:changed', { path, value });
    }
    
    resetConfig() {
        this.config = this.getDefaultConfig();
        this.saveConfiguration();
        this.emit('config:reset');
        console.log('🔄 Configuration reset to defaults');
    }
    
    // Error Handling
    initializeErrorHandling() {
        window.addEventListener('error', (event) => {
            this.handleError('JavaScript Error', event.error, {
                filename: event.filename,
                lineno: event.lineno,
                colno: event.colno
            });
        });
        
        window.addEventListener('unhandledrejection', (event) => {
            this.handleError('Unhandled Promise Rejection', event.reason);
        });
    }
    
    handleError(type, error, context = {}) {
        const errorInfo = {
            type: type,
            message: error?.message || error,
            stack: error?.stack,
            timestamp: new Date().toISOString(),
            context: context,
            userAgent: navigator.userAgent,
            url: window.location.href
        };
        
        console.error(`❌ ${type}:`, errorInfo);
        
        // Store error for debugging
        this.storeError(errorInfo);
        
        // Emit error event
        this.emit('platform:error', errorInfo);
        
        // Show user notification for critical errors
        if (this.isCriticalError(error)) {
            this.showErrorNotification(type, error.message);
        }
    }
    
    storeError(errorInfo) {
        try {
            const errors = JSON.parse(localStorage.getItem('robotech_platform_errors') || '[]');
            errors.push(errorInfo);
            
            // Keep only last 50 errors
            if (errors.length > 50) {
                errors.splice(0, errors.length - 50);
            }
            
            localStorage.setItem('robotech_platform_errors', JSON.stringify(errors));
        } catch (e) {
            console.warn('Failed to store error information');
        }
    }
    
    isCriticalError(error) {
        const criticalPatterns = [
            /WebSocket/i,
            /ROS/i,
            /Network/i,
            /Failed to fetch/i,
            /Connection/i
        ];
        
        const message = error?.message || error;
        return criticalPatterns.some(pattern => pattern.test(message));
    }
    
    showErrorNotification(type, message) {
        // Create a simple error notification
        const notification = document.createElement('div');
        notification.className = 'platform-error-notification';
        notification.innerHTML = `
            <div class="error-content">
                <strong>${type}</strong>
                <p>${message}</p>
                <button onclick="this.parentElement.parentElement.remove()">Dismiss</button>
            </div>
        `;
        
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: rgba(220, 53, 69, 0.95);
            color: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            z-index: 10000;
            max-width: 400px;
            animation: slideIn 0.3s ease;
        `;
        
        document.body.appendChild(notification);
        
        // Auto-remove after 10 seconds
        setTimeout(() => {
            if (notification.parentElement) {
                notification.remove();
            }
        }, 10000);
    }
    
    // Performance Monitoring
    initializePerformanceMonitoring() {
        this.performanceMetrics = {
            frameRate: 0,
            memoryUsage: 0,
            cpuUsage: 0,
            networkLatency: 0,
            lastUpdate: Date.now()
        };
        
        // Monitor frame rate
        this.frameCounter = 0;
        this.lastFrameTime = Date.now();
        
        // Start performance monitoring loop
        this.startPerformanceLoop();
    }
    
    startPerformanceLoop() {
        const updatePerformance = () => {
            this.frameCounter++;
            const now = Date.now();
            
            // Update frame rate every second
            if (now - this.lastFrameTime >= 1000) {
                this.performanceMetrics.frameRate = this.frameCounter;
                this.frameCounter = 0;
                this.lastFrameTime = now;
                
                // Update memory usage if available
                if (performance.memory) {
                    this.performanceMetrics.memoryUsage = performance.memory.usedJSHeapSize / 1024 / 1024; // MB
                }
                
                this.performanceMetrics.lastUpdate = now;
                this.emit('performance:update', this.performanceMetrics);
            }
            
            requestAnimationFrame(updatePerformance);
        };
        
        requestAnimationFrame(updatePerformance);
    }
    
    measureNetworkLatency(url) {
        const start = performance.now();
        return fetch(url, { method: 'HEAD', mode: 'no-cors' })
            .then(() => {
                const latency = performance.now() - start;
                this.performanceMetrics.networkLatency = latency;
                return latency;
            })
            .catch(() => {
                this.performanceMetrics.networkLatency = -1;
                return -1;
            });
    }
    
    // Event System
    on(event, callback) {
        if (!this.eventListeners.has(event)) {
            this.eventListeners.set(event, []);
        }
        this.eventListeners.get(event).push(callback);
        return () => this.off(event, callback);
    }
    
    off(event, callback) {
        const listeners = this.eventListeners.get(event);
        if (listeners) {
            const index = listeners.indexOf(callback);
            if (index > -1) {
                listeners.splice(index, 1);
            }
        }
    }
    
    emit(event, data) {
        const listeners = this.eventListeners.get(event);
        if (listeners) {
            listeners.forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    console.error(`Error in event listener for ${event}:`, error);
                }
            });
        }
    }
    
    once(event, callback) {
        const onceCallback = (data) => {
            callback(data);
            this.off(event, onceCallback);
        };
        this.on(event, onceCallback);
    }
    
    // Module Management
    registerModule(name, instance) {
        this.modules.set(name, instance);
        console.log(`📦 Module registered: ${name}`);
        this.emit('module:registered', { name, instance });
    }
    
    getModule(name) {
        return this.modules.get(name);
    }
    
    hasModule(name) {
        return this.modules.has(name);
    }
    
    unregisterModule(name) {
        const instance = this.modules.get(name);
        if (instance) {
            this.modules.delete(name);
            console.log(`📦 Module unregistered: ${name}`);
            this.emit('module:unregistered', { name, instance });
        }
    }
    
    // Utility Functions
    getNestedValue(obj, path) {
        return path.split('.').reduce((current, key) => {
            return current && current[key] !== undefined ? current[key] : undefined;
        }, obj);
    }
    
    setNestedValue(obj, path, value) {
        const keys = path.split('.');
        const lastKey = keys.pop();
        const target = keys.reduce((current, key) => {
            if (!current[key] || typeof current[key] !== 'object') {
                current[key] = {};
            }
            return current[key];
        }, obj);
        target[lastKey] = value;
    }
    
    debounce(func, wait, immediate) {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                timeout = null;
                if (!immediate) func(...args);
            };
            const callNow = immediate && !timeout;
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
            if (callNow) func(...args);
        };
    }
    
    throttle(func, limit) {
        let inThrottle;
        return function(...args) {
            if (!inThrottle) {
                func.apply(this, args);
                inThrottle = true;
                setTimeout(() => inThrottle = false, limit);
            }
        };
    }
    
    formatBytes(bytes, decimals = 2) {
        if (bytes === 0) return '0 Bytes';
        const k = 1024;
        const dm = decimals < 0 ? 0 : decimals;
        const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        return parseFloat((bytes / Math.pow(k, i)).toFixed(dm)) + ' ' + sizes[i];
    }
    
    formatDuration(milliseconds) {
        const seconds = Math.floor(milliseconds / 1000);
        const minutes = Math.floor(seconds / 60);
        const hours = Math.floor(minutes / 60);
        
        if (hours > 0) {
            return `${hours}h ${minutes % 60}m ${seconds % 60}s`;
        } else if (minutes > 0) {
            return `${minutes}m ${seconds % 60}s`;
        } else {
            return `${seconds}s`;
        }
    }
    
    generateId(prefix = 'id') {
        return `${prefix}_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }
    
    clamp(value, min, max) {
        return Math.min(Math.max(value, min), max);
    }
    
    lerp(start, end, factor) {
        return start + (end - start) * factor;
    }
    
    normalizeAngle(angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    distance2D(p1, p2) {
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    angle2D(p1, p2) {
        return Math.atan2(p2.y - p1.y, p2.x - p1.x);
    }
    
    // Validation Utilities
    validateConfig(config, schema) {
        const errors = [];
        
        const validate = (obj, schemaObj, path = '') => {
            Object.keys(schemaObj).forEach(key => {
                const fullPath = path ? `${path}.${key}` : key;
                const value = obj[key];
                const schemaValue = schemaObj[key];
                
                if (schemaValue.required && (value === undefined || value === null)) {
                    errors.push(`${fullPath} is required`);
                    return;
                }
                
                if (value !== undefined && schemaValue.type && typeof value !== schemaValue.type) {
                    errors.push(`${fullPath} must be of type ${schemaValue.type}`);
                    return;
                }
                
                if (schemaValue.min !== undefined && value < schemaValue.min) {
                    errors.push(`${fullPath} must be at least ${schemaValue.min}`);
                }
                
                if (schemaValue.max !== undefined && value > schemaValue.max) {
                    errors.push(`${fullPath} must be at most ${schemaValue.max}`);
                }
                
                if (schemaValue.enum && !schemaValue.enum.includes(value)) {
                    errors.push(`${fullPath} must be one of: ${schemaValue.enum.join(', ')}`);
                }
                
                if (schemaValue.properties && typeof value === 'object') {
                    validate(value, schemaValue.properties, fullPath);
                }
            });
        };
        
        validate(config, schema);
        
        return {
            isValid: errors.length === 0,
            errors: errors
        };
    }
    
    // Periodic Tasks
    setupPeriodicTasks() {
        // Performance monitoring
        setInterval(() => {
            if (this.getConfig('ui.performance_overlay')) {
                this.updatePerformanceOverlay();
            }
        }, 1000);
        
        // Configuration backup
        setInterval(() => {
            this.saveConfiguration();
        }, 30000); // Every 30 seconds
        
        // Memory cleanup
        setInterval(() => {
            this.performMemoryCleanup();
        }, 60000); // Every minute
    }
    
    updatePerformanceOverlay() {
        let overlay = document.getElementById('platform-performance-overlay');
        if (!overlay) {
            overlay = this.createPerformanceOverlay();
        }
        
        const metrics = this.performanceMetrics;
        overlay.innerHTML = `
            <div class="perf-metric">FPS: ${metrics.frameRate}</div>
            <div class="perf-metric">Memory: ${metrics.memoryUsage.toFixed(1)} MB</div>
            <div class="perf-metric">Latency: ${metrics.networkLatency >= 0 ? metrics.networkLatency.toFixed(0) + 'ms' : 'N/A'}</div>
        `;
    }
    
    createPerformanceOverlay() {
        const overlay = document.createElement('div');
        overlay.id = 'platform-performance-overlay';
        overlay.style.cssText = `
            position: fixed;
            top: 80px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: #00ff88;
            padding: 10px;
            border-radius: 6px;
            font-family: monospace;
            font-size: 12px;
            z-index: 9999;
            border: 1px solid rgba(0, 255, 136, 0.3);
        `;
        
        document.body.appendChild(overlay);
        return overlay;
    }
    
    performMemoryCleanup() {
        // Clear old error logs
        try {
            const errors = JSON.parse(localStorage.getItem('robotech_platform_errors') || '[]');
            if (errors.length > 100) {
                const recentErrors = errors.slice(-50);
                localStorage.setItem('robotech_platform_errors', JSON.stringify(recentErrors));
            }
        } catch (e) {
            // Ignore cleanup errors
        }
        
        // Emit cleanup event for modules to handle their own cleanup
        this.emit('platform:cleanup');
    }
    
    // Diagnostic Methods
    getDiagnostics() {
        const now = Date.now();
        const memInfo = performance.memory ? {
            used: this.formatBytes(performance.memory.usedJSHeapSize),
            total: this.formatBytes(performance.memory.totalJSHeapSize),
            limit: this.formatBytes(performance.memory.jsHeapSizeLimit)
        } : null;
        
        return {
            platform: {
                version: this.version,
                initialized: this.initialized,
                uptime: this.formatDuration(now - this.initTime),
                modules: Array.from(this.modules.keys())
            },
            performance: {
                ...this.performanceMetrics,
                memory: memInfo
            },
            browser: {
                userAgent: navigator.userAgent,
                language: navigator.language,
                cookieEnabled: navigator.cookieEnabled,
                onLine: navigator.onLine
            },
            screen: {
                width: screen.width,
                height: screen.height,
                colorDepth: screen.colorDepth,
                pixelRatio: window.devicePixelRatio
            }
        };
    }
    
    exportDiagnostics() {
        const diagnostics = this.getDiagnostics();
        const errors = JSON.parse(localStorage.getItem('robotech_platform_errors') || '[]');
        
        const exportData = {
            diagnostics: diagnostics,
            errors: errors,
            config: this.config,
            timestamp: new Date().toISOString()
        };
        
        const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `robotech_diagnostics_${Date.now()}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        
        console.log('📊 Diagnostics exported');
    }
    
    // Public API
    getVersion() {
        return this.version;
    }
    
    isInitialized() {
        return this.initialized;
    }
    
    getPerformanceMetrics() {
        return { ...this.performanceMetrics };
    }
    
    destroy() {
        // Clean up event listeners
        this.eventListeners.clear();
        
        // Unregister all modules
        this.modules.clear();
        
        // Remove performance overlay
        const overlay = document.getElementById('platform-performance-overlay');
        if (overlay) {
            overlay.remove();
        }
        
        this.initialized = false;
        console.log('🔄 Platform Core destroyed');
    }
}

// Create global instance
window.platformCore = new PlatformCore();

// Make PlatformCore class available globally
window.PlatformCore = PlatformCore;

// Add CSS for notifications and overlays
const platformStyles = document.createElement('style');
platformStyles.textContent = `
    @keyframes slideIn {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    
    .platform-error-notification {
        animation: slideIn 0.3s ease;
    }
    
    .platform-error-notification .error-content {
        display: flex;
        flex-direction: column;
        gap: 8px;
    }
    
    .platform-error-notification button {
        align-self: flex-end;
        background: rgba(255, 255, 255, 0.2);
        border: 1px solid rgba(255, 255, 255, 0.3);
        color: white;
        padding: 4px 8px;
        border-radius: 4px;
        cursor: pointer;
        font-size: 12px;
    }
    
    .platform-error-notification button:hover {
        background: rgba(255, 255, 255, 0.3);
    }
    
    .perf-metric {
        margin: 2px 0;
        white-space: nowrap;
    }
`;

document.head.appendChild(platformStyles);