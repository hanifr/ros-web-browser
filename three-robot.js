// Three.js Robot Model
class RobotModel {
    constructor() {
        this.group = new THREE.Group();
        this.wheels = [];
        this.sensors = [];
        this.laserPoints = [];
        this.wireframeMode = false;
        this.sensorsVisible = true;
        
        // Animation properties
        this.wheelRotation = 0;
        this.sensorAnimationTime = 0;
        
        this.createRobot();
    }
    
    createRobot() {
        console.log('🤖 Creating 3D robot model...');
        
        // Robot chassis/body
        this.createChassis();
        
        // Wheels
        this.createWheels();
        
        // Sensors
        this.createSensors();
        
        // Robot indicators
        this.createIndicators();
        
        // Laser scan visualization
        this.createLaserVisualization();
        
        console.log('✅ Robot model created');
    }
    
    createChassis() {
        // Main body
        const bodyGeometry = new THREE.BoxGeometry(0.6, 0.15, 0.4);
        const bodyMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x2196f3,
            transparent: true,
            opacity: 0.9
        });
        this.body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        this.body.position.y = 0.075;
        this.body.castShadow = true;
        this.group.add(this.body);
        
        // Top platform
        const platformGeometry = new THREE.CylinderGeometry(0.25, 0.25, 0.02, 16);
        const platformMaterial = new THREE.MeshLambertMaterial({ color: 0x1976d2 });
        this.platform = new THREE.Mesh(platformGeometry, platformMaterial);
        this.platform.position.y = 0.16;
        this.platform.castShadow = true;
        this.group.add(this.platform);
        
        // Battery indicator (decorative)
        const batteryGeometry = new THREE.BoxGeometry(0.1, 0.03, 0.05);
        const batteryMaterial = new THREE.MeshLambertMaterial({ color: 0x4caf50 });
        this.battery = new THREE.Mesh(batteryGeometry, batteryMaterial);
        this.battery.position.set(-0.25, 0.1, 0);
        this.group.add(this.battery);
    }
    
    createWheels() {
        const wheelGeometry = new THREE.CylinderGeometry(0.08, 0.08, 0.04, 16);
        const wheelMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
        
        // Left wheel
        const leftWheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
        leftWheel.position.set(0, 0.08, 0.22);
        leftWheel.rotation.z = Math.PI / 2;
        leftWheel.castShadow = true;
        this.wheels.push(leftWheel);
        this.group.add(leftWheel);
        
        // Right wheel
        const rightWheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
        rightWheel.position.set(0, 0.08, -0.22);
        rightWheel.rotation.z = Math.PI / 2;
        rightWheel.castShadow = true;
        this.wheels.push(rightWheel);
        this.group.add(rightWheel);
        
        // Caster wheel (rear)
        const casterGeometry = new THREE.SphereGeometry(0.03, 12, 12);
        const casterMaterial = new THREE.MeshLambertMaterial({ color: 0x666666 });
        this.caster = new THREE.Mesh(casterGeometry, casterMaterial);
        this.caster.position.set(-0.25, 0.03, 0);
        this.caster.castShadow = true;
        this.group.add(this.caster);
    }
    
    createSensors() {
        // LIDAR sensor
        const lidarGeometry = new THREE.CylinderGeometry(0.06, 0.06, 0.08, 16);
        const lidarMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x444444,
            transparent: true,
            opacity: 0.8
        });
        this.lidar = new THREE.Mesh(lidarGeometry, lidarMaterial);
        this.lidar.position.set(0, 0.2, 0);
        this.lidar.castShadow = true;
        this.sensors.push(this.lidar);
        this.group.add(this.lidar);
        
        // Camera
        const cameraGeometry = new THREE.BoxGeometry(0.04, 0.03, 0.02);
        const cameraMaterial = new THREE.MeshLambertMaterial({ color: 0x222222 });
        this.camera = new THREE.Mesh(cameraGeometry, cameraMaterial);
        this.camera.position.set(0.28, 0.12, 0);
        this.sensors.push(this.camera);
        this.group.add(this.camera);
        
        // IMU (Inertial Measurement Unit)
        const imuGeometry = new THREE.BoxGeometry(0.02, 0.01, 0.03);
        const imuMaterial = new THREE.MeshLambertMaterial({ color: 0x795548 });
        this.imu = new THREE.Mesh(imuGeometry, imuMaterial);
        this.imu.position.set(0, 0.17, 0.1);
        this.sensors.push(this.imu);
        this.group.add(this.imu);
        
        // Ultrasonic sensors (range sensors)
        const ultrasonicGeometry = new THREE.CylinderGeometry(0.008, 0.008, 0.02, 8);
        const ultrasonicMaterial = new THREE.MeshLambertMaterial({ color: 0xff9800 });
        
        // Front ultrasonic sensors
        for (let i = 0; i < 3; i++) {
            const sensor = new THREE.Mesh(ultrasonicGeometry, ultrasonicMaterial);
            const angle = (i - 1) * 0.5; // -0.5, 0, 0.5 radians
            sensor.position.set(
                0.28 * Math.cos(angle),
                0.08,
                0.28 * Math.sin(angle)
            );
            sensor.rotation.y = angle;
            this.sensors.push(sensor);
            this.group.add(sensor);
        }
        
        // Status LEDs
        this.createStatusLEDs();
    }
    
    createStatusLEDs() {
        const ledGeometry = new THREE.SphereGeometry(0.005, 8, 8);
        
        // Power LED (green)
        const powerLEDMaterial = new THREE.MeshBasicMaterial({ 
            color: 0x4caf50,
            transparent: true,
            opacity: 0.8
        });
        this.powerLED = new THREE.Mesh(ledGeometry, powerLEDMaterial);
        this.powerLED.position.set(-0.2, 0.17, 0.15);
        this.group.add(this.powerLED);
        
        // Status LED (blue - will change based on robot state)
        const statusLEDMaterial = new THREE.MeshBasicMaterial({ 
            color: 0x2196f3,
            transparent: true,
            opacity: 0.8
        });
        this.statusLED = new THREE.Mesh(ledGeometry, statusLEDMaterial);
        this.statusLED.position.set(-0.2, 0.17, 0.1);
        this.group.add(this.statusLED);
        
        // Communication LED (orange)
        const commLEDMaterial = new THREE.MeshBasicMaterial({ 
            color: 0xff9800,
            transparent: true,
            opacity: 0.8
        });
        this.commLED = new THREE.Mesh(ledGeometry, commLEDMaterial);
        this.commLED.position.set(-0.2, 0.17, 0.05);
        this.group.add(this.commLED);
    }
    
    createIndicators() {
        // Front direction indicator (arrow)
        const arrowGeometry = new THREE.ConeGeometry(0.03, 0.08, 6);
        const arrowMaterial = new THREE.MeshLambertMaterial({ 
            color: 0xff4444,
            transparent: true,
            opacity: 0.9
        });
        this.directionArrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        this.directionArrow.position.set(0.32, 0.12, 0);
        this.directionArrow.rotation.z = -Math.PI / 2;
        this.group.add(this.directionArrow);
        
        // Robot name label (optional text)
        this.createNameLabel();
    }
    
    createNameLabel() {
        // Simple text representation using geometry
        const textGeometry = new THREE.PlaneGeometry(0.15, 0.03);
        const canvas = document.createElement('canvas');
        canvas.width = 128;
        canvas.height = 32;
        const context = canvas.getContext('2d');
        
        context.fillStyle = '#ffffff';
        context.font = '16px Arial';
        context.textAlign = 'center';
        context.fillText('ROBOT-01', 64, 20);
        
        const texture = new THREE.CanvasTexture(canvas);
        const textMaterial = new THREE.MeshBasicMaterial({ 
            map: texture,
            transparent: true,
            opacity: 0.8
        });
        
        this.nameLabel = new THREE.Mesh(textGeometry, textMaterial);
        this.nameLabel.position.set(0, 0.25, 0);
        this.nameLabel.rotation.x = -Math.PI / 2;
        this.group.add(this.nameLabel);
    }
    
    createLaserVisualization() {
        // Laser scan points
        this.laserGeometry = new THREE.BufferGeometry();
        this.laserMaterial = new THREE.PointsMaterial({ 
            color: 0xff0000,
            size: 0.02,
            transparent: true,
            opacity: 0.7
        });
        this.laserPoints = new THREE.Points(this.laserGeometry, this.laserMaterial);
        this.group.add(this.laserPoints);
        
        // Laser scan lines (for debugging)
        this.laserLinesGeometry = new THREE.BufferGeometry();
        this.laserLinesMaterial = new THREE.LineBasicMaterial({ 
            color: 0xff0000,
            transparent: true,
            opacity: 0.3
        });
        this.laserLines = new THREE.LineSegments(this.laserLinesGeometry, this.laserLinesMaterial);
        this.group.add(this.laserLines);
    }
    
    update() {
        this.sensorAnimationTime += 0.016; // ~60fps
        
        // Animate LIDAR rotation
        if (this.lidar) {
            this.lidar.rotation.y += 0.05;
        }
        
        // Animate wheel rotation (visual only)
        this.wheels.forEach(wheel => {
            wheel.rotation.x += 0.02; // Simulate rotation
        });
        
        // Animate LEDs (blinking effect)
        this.animateLEDs();
        
        // Animate sensors
        this.animateSensors();
    }
    
    animateLEDs() {
        const time = this.sensorAnimationTime;
        
        // Power LED - steady on
        if (this.powerLED) {
            this.powerLED.material.opacity = 0.8 + 0.2 * Math.sin(time * 2);
        }
        
        // Status LED - slow blink
        if (this.statusLED) {
            this.statusLED.material.opacity = 0.5 + 0.5 * Math.sin(time);
        }
        
        // Comm LED - fast blink when active
        if (this.commLED) {
            this.commLED.material.opacity = 0.3 + 0.7 * Math.sin(time * 5);
        }
    }
    
    animateSensors() {
        const time = this.sensorAnimationTime;
        
        // Ultrasonic sensors - scanning animation
        this.sensors.forEach((sensor, i) => {
            if (sensor.geometry.type === 'CylinderGeometry' && sensor.material.color.getHex() === 0xff9800) {
                const intensity = 0.5 + 0.5 * Math.sin(time * 3 + i * 0.5);
                sensor.material.opacity = intensity;
            }
        });
    }
    
    updateLaserScan(scan) {
        if (!scan || !scan.ranges) return;
        
        const positions = [];
        const linePositions = [];
        
        scan.ranges.forEach((range, i) => {
            if (range < scan.range_max && range > scan.range_min) {
                const angle = scan.angle_min + i * scan.angle_increment;
                const x = range * Math.cos(angle);
                const y = 0.2; // LIDAR height
                const z = range * Math.sin(angle);
                
                // Add point for point cloud
                positions.push(x, y, z);
                
                // Add line from LIDAR to point
                linePositions.push(0, 0.2, 0); // LIDAR position
                linePositions.push(x, y, z);   // Scan point
            }
        });
        
        // Update point cloud
        this.laserGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        
        // Update scan lines
        this.laserLinesGeometry.setAttribute('position', new THREE.Float32BufferAttribute(linePositions, 3));
    }
    
    toggleWireframe() {
        this.wireframeMode = !this.wireframeMode;
        
        this.group.traverse((child) => {
            if (child.material && child.material.wireframe !== undefined) {
                child.material.wireframe = this.wireframeMode;
            }
        });
        
        console.log(`🔲 Wireframe mode: ${this.wireframeMode ? 'ON' : 'OFF'}`);
    }
    
    toggleSensors() {
        this.sensorsVisible = !this.sensorsVisible;
        
        this.sensors.forEach(sensor => {
            sensor.visible = this.sensorsVisible;
        });
        
        // Toggle laser visualization
        if (this.laserPoints) {
            this.laserPoints.visible = this.sensorsVisible;
        }
        if (this.laserLines) {
            this.laserLines.visible = this.sensorsVisible;
        }
        
        console.log(`📡 Sensors visibility: ${this.sensorsVisible ? 'ON' : 'OFF'}`);
    }
    
    setRobotState(state) {
        // Change LED colors based on robot state
        if (this.statusLED) {
            switch (state) {
                case 'idle':
                    this.statusLED.material.color.setHex(0x2196f3); // Blue
                    break;
                case 'moving':
                    this.statusLED.material.color.setHex(0x4caf50); // Green
                    break;
                case 'error':
                    this.statusLED.material.color.setHex(0xf44336); // Red
                    break;
                case 'warning':
                    this.statusLED.material.color.setHex(0xff9800); // Orange
                    break;
                default:
                    this.statusLED.material.color.setHex(0x2196f3); // Blue
            }
        }
    }
    
    setBatteryLevel(level) {
        // Update battery indicator color based on level (0-1)
        if (this.battery) {
            if (level > 0.5) {
                this.battery.material.color.setHex(0x4caf50); // Green
            } else if (level > 0.2) {
                this.battery.material.color.setHex(0xff9800); // Orange
            } else {
                this.battery.material.color.setHex(0xf44336); // Red
            }
            
            // Scale battery indicator based on level
            this.battery.scale.x = Math.max(0.1, level);
        }
    }
    
    // Cleanup method
    dispose() {
        this.group.traverse((child) => {
            if (child.geometry) {
                child.geometry.dispose();
            }
            if (child.material) {
                if (child.material.map) {
                    child.material.map.dispose();
                }
                child.material.dispose();
            }
        });
    }
}