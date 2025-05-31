// Robot Simulator Main Controller
class RobotSimulator {
    constructor() {
        this.robot = null;
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.isMoving = false;
        this.currentLinear = 0;
        this.currentAngular = 0;
        
        // Statistics
        this.stats = {
            distanceTraveled: 0,
            startTime: Date.now(),
            commandsSent: 0,
            lastPosition: { x: 0, y: 0 }
        };
        
        // Settings
        this.settings = {
            showTrail: true,
            showSensors: true,
            autoFollow: true,
            graphicsQuality: 'medium',
            updateRate: 60
        };
        
        this.init();
    }
    
    init() {
        console.log('🚀 Initializing Robot Simulator...');
        
        // Initialize Three.js scene
        this.initThreeJS();
        
        // Initialize ROS connection
        this.initROS();
        
        // Setup robot
        this.createRobot();
        
        // Setup UI event handlers
        this.setupUI();
        
        // Start animation loop
        this.animate();
        
        // Start statistics update
        this.updateStats();
        
        console.log('✅ Robot Simulator initialized successfully!');
    }
    
    initThreeJS() {
        console.log('🎨 Setting up Three.js scene...');
        
        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);
        this.scene.fog = new THREE.Fog(0x1a1a2e, 10, 50);
        
        // Camera setup
        const container = document.getElementById('three-container');
        this.camera = new THREE.PerspectiveCamera(
            75, 
            container.clientWidth / container.clientHeight, 
            0.1, 
            1000
        );
        this.camera.position.set(5, 5, 5);
        
        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true 
        });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        
        container.appendChild(this.renderer.domElement);
        
        // Lighting setup
        this.setupLighting();
        
        // Environment setup
        this.setupEnvironment();
        
        // Controls setup
        this.setupControls();
        
        // Handle window resize
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        // Directional light (sun)
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        directionalLight.shadow.camera.left = -10;
        directionalLight.shadow.camera.right = 10;
        directionalLight.shadow.camera.top = 10;
        directionalLight.shadow.camera.bottom = -10;
        this.scene.add(directionalLight);
        
        // Hemisphere light for better ambient lighting
        const hemisphereLight = new THREE.HemisphereLight(0x87ceeb, 0x8B4513, 0.3);
        this.scene.add(hemisphereLight);
    }
    
    setupEnvironment() {
        // Ground plane
        const groundGeometry = new THREE.PlaneGeometry(50, 50);
        const groundMaterial = new THREE.MeshLambertMaterial({ 
            color: 0x2d2d44,
            transparent: true,
            opacity: 0.8
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        this.scene.add(ground);
        
        // Grid helper
        this.gridHelper = new THREE.GridHelper(50, 50, 0x444444, 0x444444);
        this.gridHelper.material.transparent = true;
        this.gridHelper.material.opacity = 0.3;
        this.scene.add(this.gridHelper);
        
        // Add some obstacles/environment features
        this.createObstacles();
        
        // Robot trail
        this.robotTrail = [];
        this.trailGeometry = new THREE.BufferGeometry();
        this.trailMaterial = new THREE.LineBasicMaterial({ 
            color: 0x00ff88,
            transparent: true,
            opacity: 0.6
        });
        this.trailLine = new THREE.Line(this.trailGeometry, this.trailMaterial);
        this.scene.add(this.trailLine);
    }
    
    createObstacles() {
        // Simple obstacles for testing
        const obstacles = [
            { x: 3, z: 2, width: 1, height: 0.5, depth: 1 },
            { x: -2, z: -1, width: 0.8, height: 0.8, depth: 0.8 },
            { x: 1, z: -3, width: 1.2, height: 0.3, depth: 1.2 },
        ];
        
        obstacles.forEach(obs => {
            const geometry = new THREE.BoxGeometry(obs.width, obs.height, obs.depth);
            const material = new THREE.MeshLambertMaterial({ 
                color: Math.random() * 0xffffff 
            });
            const obstacle = new THREE.Mesh(geometry, material);
            obstacle.position.set(obs.x, obs.height/2, obs.z);
            obstacle.castShadow = true;
            obstacle.receiveShadow = true;
            this.scene.add(obstacle);
        });
    }
    
    setupControls() {
        const container = this.renderer.domElement;
        
        this.controls = {
            mouseX: 0,
            mouseY: 0,
            isMouseDown: false,
            rotationX: 0,
            rotationY: 0
        };
        
        container.addEventListener('mousedown', (e) => {
            this.controls.isMouseDown = true;
            this.controls.mouseX = e.clientX;
            this.controls.mouseY = e.clientY;
        });
        
        container.addEventListener('mouseup', () => {
            this.controls.isMouseDown = false;
        });
        
        container.addEventListener('mousemove', (e) => {
            if (!this.controls.isMouseDown) return;
            
            const deltaX = e.clientX - this.controls.mouseX;
            const deltaY = e.clientY - this.controls.mouseY;
            
            this.controls.rotationY -= deltaX * 0.01;
            this.controls.rotationX -= deltaY * 0.01;
            this.controls.rotationX = Math.max(-Math.PI/2, Math.min(Math.PI/2, this.controls.rotationX));
            
            this.updateCameraPosition();
            
            this.controls.mouseX = e.clientX;
            this.controls.mouseY = e.clientY;
        });
        
        container.addEventListener('wheel', (e) => {
            this.cameraDistance = Math.max(2, Math.min(20, this.cameraDistance + e.deltaY * 0.01));
            this.updateCameraPosition();
        });
        
        this.cameraDistance = 8;
        this.updateCameraPosition();
    }
    
    updateCameraPosition() {
        const robotPos = this.robot ? this.robot.group.position : new THREE.Vector3(0, 0, 0);
        
        this.camera.position.x = robotPos.x + this.cameraDistance * Math.cos(this.controls.rotationX) * Math.cos(this.controls.rotationY);
        this.camera.position.y = robotPos.y + this.cameraDistance * Math.sin(this.controls.rotationX) + 3;
        this.camera.position.z = robotPos.z + this.cameraDistance * Math.cos(this.controls.rotationX) * Math.sin(this.controls.rotationY);
        
        this.camera.lookAt(robotPos);
    }
    
    initROS() {
        console.log('🔌 Connecting to ROS...');
        
        const wsUrl = document.getElementById('websocket-url').value || 'ws://localhost:9090';
        window.rosConnection = new ROSBridge(wsUrl);
        
        // Subscribe to robot pose updates
        window.rosConnection.subscribe('/robot_pose', 'geometry_msgs/PoseStamped', 
            (message) => {
                this.updateRobotPose(message.pose);
            });
            
        // Subscribe to odometry
        window.rosConnection.subscribe('/odom', 'nav_msgs/Odometry',
            (message) => {
                this.updateOdometry(message);
            });
            
        // Subscribe to velocity commands echo
        window.rosConnection.subscribe('/cmd_vel_echo', 'geometry_msgs/Twist',
            (message) => {
                this.updateVelocityDisplay(message);
            });
            
        // Subscribe to laser scan
        window.rosConnection.subscribe('/scan', 'sensor_msgs/LaserScan',
            (message) => {
                this.updateLaserScan(message);
            });
            
        // Subscribe to range sensor
        window.rosConnection.subscribe('/range_sensor', 'sensor_msgs/Range',
            (message) => {
                this.updateRangeSensor(message);
            });
    }
    
    createRobot() {
        console.log('🤖 Creating robot model...');
        this.robot = new RobotModel();
        this.scene.add(this.robot.group);
        
        // Update object count
        this.updateObjectCount();
    }
    
    setupUI() {
        // Speed slider event handlers
        const linearSlider = document.getElementById('linear-speed');
        const angularSlider = document.getElementById('angular-speed');
        
        linearSlider.addEventListener('input', (e) => {
            document.getElementById('linear-value').textContent = `${e.target.value} m/s`;
        });
        
        angularSlider.addEventListener('input', (e) => {
            document.getElementById('angular-value').textContent = `${e.target.value} rad/s`;
        });
        
        // Modal handlers
        this.setupModals();
        
        // Action button handlers
        document.getElementById('reset-pose-btn').addEventListener('click', () => {
            this.resetRobotPose();
        });
        
        document.getElementById('emergency-stop-btn').addEventListener('click', () => {
            this.emergencyStop();
        });
        
        // Settings handlers
        document.getElementById('reconnect-btn').addEventListener('click', () => {
            this.reconnectROS();
        });
    }
    
    setupModals() {
        const modals = document.querySelectorAll('.modal');
        const closeButtons = document.querySelectorAll('.close');
        
        // Settings modal
        document.getElementById('settings-btn').addEventListener('click', () => {
            document.getElementById('settings-modal').style.display = 'block';
        });
        
        // Help modal
        document.getElementById('help-btn').addEventListener('click', () => {
            document.getElementById('help-modal').style.display = 'block';
        });
        
        // Close modal handlers
        closeButtons.forEach(button => {
            button.addEventListener('click', (e) => {
                e.target.closest('.modal').style.display = 'none';
            });
        });
        
        // Click outside to close
        window.addEventListener('click', (e) => {
            modals.forEach(modal => {
                if (e.target === modal) {
                    modal.style.display = 'none';
                }
            });
        });
    }
    
    updateRobotPose(pose) {
        if (!this.robot) return;
        
        const newPos = new THREE.Vector3(
            pose.position.x,
            pose.position.z,
            -pose.position.y
        );
        
        this.robot.group.position.copy(newPos);
        
        // Convert quaternion
        this.robot.group.quaternion.set(
            pose.orientation.x,
            pose.orientation.z,
            -pose.orientation.y,
            pose.orientation.w
        );
        
        // Update trail
        if (this.settings.showTrail) {
            this.updateRobotTrail(newPos);
        }
        
        // Update camera if auto-follow is enabled
        if (this.settings.autoFollow) {
            this.updateCameraPosition();
        }
        
        // Update UI
        document.getElementById('position').textContent = 
            `x: ${pose.position.x.toFixed(2)}, y: ${pose.position.y.toFixed(2)}`;
            
        // Update statistics
        this.updateDistanceTraveled(pose.position);
    }
    
    updateOdometry(odom) {
        const twist = odom.twist.twist;
        const linearVel = Math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2);
        
        document.getElementById('linear-velocity').textContent = `${linearVel.toFixed(2)} m/s`;
        document.getElementById('angular-velocity').textContent = `${twist.angular.z.toFixed(2)} rad/s`;
    }
    
    updateVelocityDisplay(twist) {
        // Update orientation display
        if (this.robot) {
            const euler = new THREE.Euler().setFromQuaternion(this.robot.group.quaternion);
            const angle = euler.y * 180 / Math.PI;
            document.getElementById('orientation').textContent = `${angle.toFixed(1)}°`;
        }
    }
    
    updateLaserScan(scan) {
        if (!this.robot) return;
        
        // Update laser visualization
        this.robot.updateLaserScan(scan);
        
        // Update laser panel
        document.getElementById('laser-points').textContent = scan.ranges.length;
        document.getElementById('laser-range').textContent = `${scan.range_min.toFixed(1)}-${scan.range_max.toFixed(1)}m`;
        
        // Draw laser scan on canvas
        this.drawLaserScan(scan);
    }
    
    updateRangeSensor(range) {
        const distance = range.range < range.max_range ? range.range.toFixed(2) : '--';
        document.getElementById('range-sensor').textContent = `${distance} m`;
    }
    
    updateRobotTrail(position) {
        this.robotTrail.push(position.clone());
        
        // Limit trail length
        if (this.robotTrail.length > 100) {
            this.robotTrail.shift();
        }
        
        // Update trail geometry
        const positions = [];
        this.robotTrail.forEach(pos => {
            positions.push(pos.x, pos.y, pos.z);
        });
        
        this.trailGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    }
    
    drawLaserScan(scan) {
        const canvas = document.getElementById('laser-canvas');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(centerX, centerY) - 10;
        
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw background circle
        ctx.strokeStyle = '#333';
        ctx.beginPath();
        ctx.arc(centerX, centerY, maxRadius, 0, 2 * Math.PI);
        ctx.stroke();
        
        // Draw scan points
        ctx.fillStyle = '#ff4444';
        scan.ranges.forEach((range, i) => {
            if (range < scan.range_max && range > scan.range_min) {
                const angle = scan.angle_min + i * scan.angle_increment;
                const distance = (range / scan.range_max) * maxRadius;
                const x = centerX + distance * Math.cos(angle);
                const y = centerY + distance * Math.sin(angle);
                
                ctx.beginPath();
                ctx.arc(x, y, 2, 0, 2 * Math.PI);
                ctx.fill();
            }
        });
        
        // Draw robot in center
        ctx.fillStyle = '#00ff88';
        ctx.beginPath();
        ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI);
        ctx.fill();
    }
    
    updateStats() {
        setInterval(() => {
            // Update runtime
            const runtime = (Date.now() - this.stats.startTime) / 1000;
            const minutes = Math.floor(runtime / 60);
            const seconds = Math.floor(runtime % 60);
            document.getElementById('runtime').textContent = 
                `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
            
            // Update distance
            document.getElementById('distance-traveled').textContent = 
                `${this.stats.distanceTraveled.toFixed(2)} m`;
            
            // Update commands sent
            document.getElementById('commands-sent').textContent = this.stats.commandsSent;
        }, 1000);
    }
    
    updateDistanceTraveled(position) {
        const dx = position.x - this.stats.lastPosition.x;
        const dy = position.y - this.stats.lastPosition.y;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        this.stats.distanceTraveled += distance;
        this.stats.lastPosition = { x: position.x, y: position.y };
    }
    
    updateObjectCount() {
        let count = 0;
        this.scene.traverse(() => count++);
        document.getElementById('object-count').textContent = count;
    }
    
    onWindowResize() {
        const container = document.getElementById('three-container');
        this.camera.aspect = container.clientWidth / container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(container.clientWidth, container.clientHeight);
    }
    
    resetRobotPose() {
        if (window.rosConnection) {
            window.rosConnection.resetPose();
            this.robotTrail = [];
            this.stats.distanceTraveled = 0;
            this.stats.lastPosition = { x: 0, y: 0 };
        }
    }
    
    emergencyStop() {
        if (window.rosConnection) {
            window.rosConnection.emergencyStop();
        }
        this.currentLinear = 0;
        this.currentAngular = 0;
    }
    
    reconnectROS() {
        if (window.rosConnection) {
            window.rosConnection.reconnect();
        }
    }
    
    animate() {
        requestAnimationFrame(() => this.animate());
        
        // Update robot animation
        if (this.robot) {
            this.robot.update();
        }
        
        // Update FPS counter
        this.updateFPS();
        
        this.renderer.render(this.scene, this.camera);
    }
    
    updateFPS() {
        // Simple FPS counter
        this.frameCount = (this.frameCount || 0) + 1;
        this.lastTime = this.lastTime || Date.now();
        
        if (Date.now() - this.lastTime >= 1000) {
            document.getElementById('fps').textContent = this.frameCount;
            this.frameCount = 0;
            this.lastTime = Date.now();
        }
    }
}

// Global control functions
function startMove(linear, angular) {
    const linearSpeed = parseFloat(document.getElementById('linear-speed').value);
    const angularSpeed = parseFloat(document.getElementById('angular-speed').value);
    
    if (window.rosConnection) {
        window.rosConnection.publishVelocity(
            linear * linearSpeed, 
            angular * angularSpeed
        );
        
        // Update statistics
        if (window.simulator) {
            window.simulator.stats.commandsSent++;
        }
    }
}

function stopMove() {
    if (window.rosConnection) {
        window.rosConnection.publishVelocity(0, 0);
    }
}

function resetView() {
    if (window.simulator) {
        window.simulator.controls.rotationX = 0;
        window.simulator.controls.rotationY = 0;
        window.simulator.cameraDistance = 8;
        window.simulator.updateCameraPosition();
    }
}

function toggleWireframe() {
    if (window.simulator && window.simulator.robot) {
        window.simulator.robot.toggleWireframe();
    }
}

function toggleGrid() {
    if (window.simulator && window.simulator.gridHelper) {
        window.simulator.gridHelper.visible = !window.simulator.gridHelper.visible;
    }
}

function toggleSensors() {
    if (window.simulator && window.simulator.robot) {
        window.simulator.robot.toggleSensors();
    }
}

function togglePath() {
    if (window.simulator && window.simulator.trailLine) {
        window.simulator.trailLine.visible = !window.simulator.trailLine.visible;
    }
}