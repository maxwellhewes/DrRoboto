// Global variables for robot control
let socket;
let stepSize = 5; // degrees per key press
let currentJoints = [0.0, 0.0, 0.0]; // [joint0, joint1, joint2] in degrees
let currentPosition = { x: 0, y: 0, z: 1.8 }; // Current end-effector position
let pressedKeys = new Set(); // Track currently pressed keys
let keyRepeatInterval = null; // For continuous movement

// Joint limits (in degrees)
const JOINT_LIMITS = {
    0: { min: -90, max: 90 },   // Joint 0: -90° to +90°
    1: { min: -34, max: 86 },   // Joint 1: -34° to +86°
    2: { min: -115, max: 115 }  // Joint 2: -115° to +115°
};

// Initialize the control interface
document.addEventListener('DOMContentLoaded', function () {
    initializeControls();
    initializeSocket();
    initializeKeyboardControls();
});

function initializeControls() {
    // Initialize step size slider
    initializeStepSizeControl();

    // Initialize individual joint sliders (keep existing functionality)
    initializeSliders();

    // Update displays
    updateJointDisplays();
    updatePositionDisplay();
}

function initializeStepSizeControl() {
    const stepSizeValue = document.getElementById('stepSizeValue');
    const stepSizeUp = document.getElementById('stepSizeUp');
    const stepSizeDown = document.getElementById('stepSizeDown');

    if (stepSizeValue && stepSizeUp && stepSizeDown) {
        // Update display
        stepSizeValue.value = stepSize;

        // Up button
        stepSizeUp.addEventListener('click', function () {
            if (stepSize < 10) {
                stepSize++;
                stepSizeValue.value = stepSize;
            }
        });

        // Down button
        stepSizeDown.addEventListener('click', function () {
            if (stepSize > 1) {
                stepSize--;
                stepSizeValue.value = stepSize;
            }
        });
    }
}

function initializeKeyboardControls() {
    // Add event listeners for keydown and keyup
    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keyup', handleKeyUp);

    // Prevent default behavior for arrow keys and WASD to avoid page scrolling
    document.addEventListener('keydown', function (event) {
        const key = event.key.toLowerCase();
        if (['w', 's', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
            event.preventDefault();
        }
    });
}

function handleKeyDown(event) {
    const key = event.key.toLowerCase();

    // Only process if key isn't already pressed (prevent multiple triggers)
    if (pressedKeys.has(key)) {
        return;
    }

    pressedKeys.add(key);

    // Map keys to joint movements
    let joint = -1;
    let direction = 0;

    switch (key) {
        case 'w':
            joint = 1;
            direction = 1; // + degrees
            break;
        case 's':
            joint = 1;
            direction = -1; // - degrees
            break;
        case 'arrowup':
            joint = 2;
            direction = 1; // + degrees
            break;
        case 'arrowdown':
            joint = 2;
            direction = -1; // - degrees
            break;
        case 'arrowleft':
            joint = 0;
            direction = 1; // + degrees
            break;
        case 'arrowright':
            joint = 0;
            direction = -1; // - degrees
            break;
    }

    if (joint !== -1) {
        moveJoint(joint, direction);

        // Start continuous movement
        startKeyRepeat(joint, direction);
    }
}

function handleKeyUp(event) {
    const key = event.key.toLowerCase();
    pressedKeys.delete(key);

    // Stop continuous movement
    stopKeyRepeat();
}

function moveJoint(joint, direction) {
    // Calculate new angle
    const newAngle = currentJoints[joint] + (direction * stepSize);

    // Check joint limits
    const limits = JOINT_LIMITS[joint];
    if (newAngle < limits.min || newAngle > limits.max) {
        // Update status to show limit reached
        updateJointStatus(joint, 'limit');
        return;
    }

    // Update joint angle
    currentJoints[joint] = newAngle;

    // Send command to robot
    const radians = newAngle * Math.PI / 180;
    sendJointCommand(joint, radians);

    // Update displays
    updateJointDisplays();
    updatePositionDisplay();

    // Update status to show OK
    updateJointStatus(joint, 'ok');
}

function startKeyRepeat(joint, direction) {
    // Clear any existing interval
    stopKeyRepeat();

    // Start repeating movement every 100ms (10 times per second)
    keyRepeatInterval = setInterval(() => {
        moveJoint(joint, direction);
    }, 100);
}

function stopKeyRepeat() {
    if (keyRepeatInterval) {
        clearInterval(keyRepeatInterval);
        keyRepeatInterval = null;
    }
}

function updateJointDisplays() {
    // Update joint angle displays
    for (let i = 0; i < 3; i++) {
        const display = document.getElementById(`joint${i}Display`);
        if (display) {
            display.textContent = currentJoints[i].toFixed(1) + '°';
        }
    }

    // Update individual joint sliders
    for (let i = 0; i < 3; i++) {
        const slider = document.getElementById(`joint${i}`);
        const valueDisplay = document.getElementById(`joint${i}-value`);

        if (slider && valueDisplay) {
            slider.value = currentJoints[i];
            valueDisplay.textContent = currentJoints[i].toFixed(1) + '°';
        }
    }
}

function updateJointStatus(joint, status) {
    const statusElement = document.getElementById(`joint${joint}Status`);
    if (statusElement) {
        switch (status) {
            case 'ok':
                statusElement.textContent = 'OK';
                statusElement.className = 'badge bg-success';
                break;
            case 'limit':
                statusElement.textContent = 'LIMIT';
                statusElement.className = 'badge bg-warning';
                break;
            case 'error':
                statusElement.textContent = 'ERROR';
                statusElement.className = 'badge bg-danger';
                break;
        }
    }
}

function updatePositionDisplay() {
    // Calculate end-effector position using forward kinematics
    const position = calculateForwardKinematics();

    const currentX = document.getElementById('currentX');
    const currentY = document.getElementById('currentY');
    const currentZ = document.getElementById('currentZ');
    const reachabilityStatus = document.getElementById('reachabilityStatus');

    if (currentX) currentX.textContent = position.x.toFixed(3);
    if (currentY) currentY.textContent = position.y.toFixed(3);
    if (currentZ) currentZ.textContent = position.z.toFixed(3);

    // Update reachability status
    if (reachabilityStatus) {
        reachabilityStatus.textContent = 'Reachable';
        reachabilityStatus.className = 'h4 mb-1 text-success';
    }
}

function calculateForwardKinematics() {
    // Convert degrees to radians
    const joint0 = currentJoints[0] * Math.PI / 180;
    const joint1 = currentJoints[1] * Math.PI / 180;
    const joint2 = currentJoints[2] * Math.PI / 180;

    // Robot dimensions (matching backend)
    const LINK1_LENGTH = 1.99;
    const LINK2_LENGTH = 0.4;
    const BASE_HEIGHT = 1.8;

    // Calculate forward kinematics
    const x1 = LINK1_LENGTH * Math.cos(joint1);
    const y1 = LINK1_LENGTH * Math.sin(joint1);
    const z1 = 0;

    const x2 = x1 + LINK2_LENGTH * Math.cos(joint1 + joint2);
    const y2 = y1 + LINK2_LENGTH * Math.sin(joint1 + joint2);
    const z2 = z1;

    // Apply base rotation
    const x = x2 * Math.cos(joint0) - y2 * Math.sin(joint0);
    const y = x2 * Math.sin(joint0) + y2 * Math.cos(joint0);
    const z = z2 + BASE_HEIGHT;

    return { x, y, z };
}

function sendJointCommand(joint, radians) {
    if (socket) {
        console.log(`Sending joint ${joint} command: ${radians.toFixed(3)} rad (${(radians * 180 / Math.PI).toFixed(1)}°)`);
        socket.emit('set_joint', { joint: joint, value: radians });
    } else {
        console.error('Socket not connected!');
    }
}

// Initialize individual joint sliders (keep existing functionality)
function initializeSliders() {
    const sliders = ['joint0', 'joint1', 'joint2'];

    sliders.forEach(sliderId => {
        const slider = document.getElementById(sliderId);
        const valueDisplay = document.getElementById(sliderId + '-value');

        if (slider && valueDisplay) {
            slider.addEventListener('input', function () {
                const value = parseFloat(this.value);
                const jointIndex = parseInt(sliderId.replace('joint', ''));

                // Update current joint angle
                currentJoints[jointIndex] = value;

                // Update displays
                updateJointDisplays();
                updatePositionDisplay();

                // Send command to robot
                const radians = value * Math.PI / 180;
                sendJointCommand(jointIndex, radians);
            });
        }
    });
}

// Socket.IO connection
function initializeSocket() {
    socket = io();

    socket.on('connect', function () {
        console.log('Connected to server via Socket.IO');
    });

    socket.on('disconnect', function () {
        console.log('Disconnected from server');
    });

    socket.on('connection_status', function (data) {
        console.log('Connection status:', data);
    });

    socket.on('joint_states', function (data) {
        console.log('Received joint states:', data);
        // Update sliders when joint states change
        if (data.joints) {
            data.joints.forEach((joint, index) => {
                const degrees = joint * 180 / Math.PI;
                currentJoints[index] = degrees;

                const slider = document.getElementById(`joint${index}`);
                const valueDisplay = document.getElementById(`joint${index}-value`);

                if (slider && valueDisplay) {
                    slider.value = degrees;
                    valueDisplay.textContent = degrees.toFixed(1) + '°';
                }
            });

            // Update displays
            updateJointDisplays();
            updatePositionDisplay();
        }
    });

    socket.on('joint_updated', function (data) {
        console.log('Joint updated:', data);
    });

    socket.on('end_effector_updated', function (data) {
        console.log('End-effector updated:', data);
    });

    socket.on('error', function (data) {
        console.error('Socket.IO error:', data);
    });

    socket.on('joints_reset', function (data) {
        console.log('Joints reset:', data);
    });

    socket.on('moved_to_home', function (data) {
        console.log('Moved to home:', data);
    });
}

// Reset functions
function resetJoints() {
    if (socket) {
        socket.emit('reset_joints');
    }

    // Reset UI
    currentJoints = [0.0, 0.0, 0.0];
    updateJointDisplays();
    updatePositionDisplay();

    // Reset all joint statuses
    for (let i = 0; i < 3; i++) {
        updateJointStatus(i, 'ok');
    }
}

function moveToHome() {
    if (socket) {
        socket.emit('move_to_home');
    }

    // Reset UI to home position
    currentJoints = [0.0, 0.0, 0.0];
    updateJointDisplays();
    updatePositionDisplay();

    // Reset all joint statuses
    for (let i = 0; i < 3; i++) {
        updateJointStatus(i, 'ok');
    }
}

// Cleanup on page unload
window.addEventListener('beforeunload', function () {
    stopKeyRepeat();
});