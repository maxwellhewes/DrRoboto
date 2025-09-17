// DrRoboto Web Control Interface JavaScript

let commandsSent = 0;
let successfulCommands = 0;
let socket = null;
let websocketConnected = false;

/**
 * Initialize the interface when the DOM is fully loaded.
 * Sets up sliders, updates statistics, checks connection, and initializes WebSocket.
 */
document.addEventListener('DOMContentLoaded', function () {
    initializeSliders();
    updateStats();
    checkConnection();
    initializeWebSocket();
});

/**
 * Initialize joint sliders with event listeners for real-time control.
 * Sets up input event handlers for both joint sliders and implements debounced
 * position sending to prevent excessive API calls during slider dragging.
 */
function initializeSliders() {
    const joint1Slider = document.getElementById('joint1');
    const joint2Slider = document.getElementById('joint2');
    const joint1Value = document.getElementById('joint1-value');
    const joint2Value = document.getElementById('joint2-value');

    // Debounce timer for slider updates
    let sliderDebounceTimer = null;

    joint1Slider.addEventListener('input', function () {
        joint1Value.textContent = this.value + '°';
        sendSliderPosition();
    });

    joint2Slider.addEventListener('input', function () {
        joint2Value.textContent = this.value + '°';
        sendSliderPosition();
    });

    // Debounced function to send slider position
    function sendSliderPosition() {
        clearTimeout(sliderDebounceTimer);
        sliderDebounceTimer = setTimeout(() => {
            setCustomJoints(false); // Don't show loading modal for slider updates
        }, 150); // 150ms debounce delay
    }
}

/**
 * Move the robot to a predefined pose.
 * @param {string} poseName - The name of the pose to move to (e.g., 'home', 'wave')
 * @returns {Promise<void>} Promise that resolves when the pose command is complete
 */
async function moveToPose(poseName) {
    console.log('Starting moveToPose:', poseName);
    showLoading();

    try {
        console.log('Sending request to:', `/api/move/${poseName}`);
        const response = await fetch(`/api/move/${poseName}`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });

        console.log('Response received:', response.status);
        const data = await response.json();
        console.log('Response data:', data);

        if (data.success) {
            showStatus('success', `Moved to pose: ${poseName}`, data.message);
            updateLastCommand(`Pose: ${poseName}`);
        } else {
            showStatus('danger', 'Error', data.error);
        }
    } catch (error) {
        console.error('Error in moveToPose:', error);
        showStatus('danger', 'Error', 'Failed to send command: ' + error.message);
    } finally {
        console.log('Finally block - hiding loading');
        hideLoading();
        updateStats();
    }
}

/**
 * Set custom joint positions for the robot.
 * @param {boolean} [showLoadingModal=true] - Whether to show loading modal and status messages
 * @returns {Promise<void>} Promise that resolves when the joint command is complete
 */
async function setCustomJoints(showLoadingModal = true) {
    const joint1 = document.getElementById('joint1').value;
    const joint2 = document.getElementById('joint2').value;

    if (showLoadingModal) {
        showLoading();
    }

    try {
        const response = await fetch('/api/joints', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                joint1: parseFloat(joint1) * Math.PI / 180, // Convert to radians
                joint2: parseFloat(joint2) * Math.PI / 180
            })
        });

        const data = await response.json();

        if (data.success) {
            if (showLoadingModal) {
                showStatus('success', 'Joints Set', data.message);
            }
            updateLastCommand(`Custom: J1=${joint1}°, J2=${joint2}°`);
        } else {
            if (showLoadingModal) {
                showStatus('danger', 'Error', data.error);
            }
        }
    } catch (error) {
        if (showLoadingModal) {
            showStatus('danger', 'Error', 'Failed to send command: ' + error.message);
        }
    } finally {
        if (showLoadingModal) {
            hideLoading();
        }
        updateStats();
    }
}

/**
 * Reset joints to home position (0 degrees) and move robot to home pose.
 * Updates both slider values and display text, then triggers home pose movement.
 */
function resetJoints() {
    document.getElementById('joint1').value = 0;
    document.getElementById('joint2').value = 0;
    document.getElementById('joint1-value').textContent = '0°';
    document.getElementById('joint2-value').textContent = '0°';
    moveToPose('home');
}

/**
 * Perform a wave gesture with the robot.
 * @returns {Promise<void>} Promise that resolves when the wave gesture is complete
 */
async function waveGesture() {
    showLoading();

    try {
        const response = await fetch('/api/wave', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            }
        });

        const data = await response.json();

        if (data.success) {
            showStatus('success', 'Wave Gesture', data.message);
            updateLastCommand('Wave Gesture');
        } else {
            showStatus('danger', 'Error', 'Wave gesture failed');
        }
    } catch (error) {
        showStatus('danger', 'Error', 'Failed to send command: ' + error.message);
    } finally {
        hideLoading();
        updateStats();
    }
}

/**
 * Emergency stop function that immediately moves robot to home position.
 * Shows a warning status message and triggers home pose movement.
 */
function emergencyStop() {
    showStatus('warning', 'Emergency Stop', 'Moving to home position...');
    moveToPose('home');
}

/**
 * Get the current status of the robot.
 * @returns {Promise<void>} Promise that resolves when status is retrieved
 */
async function getStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();

        if (data.success) {
            showStatus('info', 'Robot Status', data.status);
        } else {
            showStatus('danger', 'Status Error', data.error);
        }
    } catch (error) {
        showStatus('danger', 'Error', 'Failed to get status: ' + error.message);
    }
}

/**
 * Get list of available ROS topics from the robot.
 * @returns {Promise<void>} Promise that resolves when topics are retrieved
 */
async function getTopics() {
    try {
        const response = await fetch('/api/topics');
        const data = await response.json();

        if (data.success) {
            const topicsList = data.topics.join('<br>');
            showStatus('info', 'Available Topics', topicsList);
        } else {
            showStatus('danger', 'Topics Error', data.error);
        }
    } catch (error) {
        showStatus('danger', 'Error', 'Failed to get topics: ' + error.message);
    }
}

/**
 * Check the health status of the connection to the robot.
 * Updates the connection status indicator based on API health check.
 * @returns {Promise<void>} Promise that resolves when health check is complete
 */
async function checkConnection() {
    try {
        const response = await fetch('/api/health');
        const data = await response.json();

        if (data.status === 'healthy') {
            updateConnectionStatus(true);
        } else {
            updateConnectionStatus(false);
        }
    } catch (error) {
        updateConnectionStatus(false);
    }
}

/**
 * Update the visual connection status indicator.
 * @param {boolean} connected - Whether the connection is active
 */
function updateConnectionStatus(connected) {
    const indicator = document.getElementById('connection-indicator');
    if (connected) {
        indicator.className = 'badge bg-success me-2';
        indicator.innerHTML = '<i class="fas fa-circle"></i> Connected';
    } else {
        indicator.className = 'badge bg-danger me-2';
        indicator.innerHTML = '<i class="fas fa-circle"></i> Disconnected';
    }
}

/**
 * Display a status message in the status panel.
 * @param {string} type - Alert type ('success', 'danger', 'warning', 'info')
 * @param {string} title - Title of the status message
 * @param {string} message - Content of the status message
 */
function showStatus(type, title, message) {
    const statusPanel = document.getElementById('status-panel');
    const alertDiv = document.createElement('div');
    alertDiv.className = `alert alert-${type} status-update`;
    alertDiv.innerHTML = `
        <strong>${title}:</strong> ${message}
        <button type="button" class="btn-close float-end" onclick="this.parentElement.remove()"></button>
    `;

    statusPanel.insertBefore(alertDiv, statusPanel.firstChild);

    // Keep only last 5 messages
    while (statusPanel.children.length > 5) {
        statusPanel.removeChild(statusPanel.lastChild);
    }

    // Auto-remove after 5 seconds for success messages
    if (type === 'success') {
        setTimeout(() => {
            if (alertDiv.parentElement) {
                alertDiv.remove();
            }
        }, 5000);
    }
}

/**
 * Update the display showing the last command sent to the robot.
 * @param {string} command - The command text to display
 */
function updateLastCommand(command) {
    document.getElementById('last-command').textContent = command;
}

/**
 * Update command statistics (commands sent and success rate).
 * Increments the commands sent counter and calculates success rate.
 */
function updateStats() {
    commandsSent++;
    const successRate = commandsSent > 0 ? Math.round((successfulCommands / commandsSent) * 100) : 100;

    document.getElementById('commands-sent').textContent = commandsSent;
    document.getElementById('success-rate').textContent = successRate + '%';
}

/**
 * Show the loading modal with backdrop.
 * Prevents body scrolling and displays a loading overlay.
 */
function showLoading() {
    console.log('Showing loading modal...');
    const modalElement = document.getElementById('loadingModal');
    modalElement.style.display = 'block';
    modalElement.classList.add('show');

    // Add backdrop
    const backdrop = document.createElement('div');
    backdrop.className = 'modal-backdrop';
    backdrop.id = 'loadingBackdrop';
    document.body.appendChild(backdrop);

    // Prevent body scroll
    document.body.style.overflow = 'hidden';
}

/**
 * Hide the loading modal and restore normal page behavior.
 * Removes backdrop and restores body scrolling.
 */
function hideLoading() {
    console.log('Hiding loading modal...');
    const modalElement = document.getElementById('loadingModal');
    modalElement.style.display = 'none';
    modalElement.classList.remove('show');

    // Remove backdrop
    const backdrop = document.getElementById('loadingBackdrop');
    if (backdrop) {
        backdrop.remove();
    }

    // Restore body scroll
    document.body.style.overflow = '';
}

// Periodic connection check
setInterval(checkConnection, 30000); // Check every 30 seconds

/**
 * Initialize WebSocket connection for real-time communication.
 * Sets up Socket.IO connection and event handlers for joint state updates
 * and connection status monitoring.
 */
function initializeWebSocket() {
    console.log('Initializing WebSocket connection...');

    // Connect to the Socket.IO server
    socket = io();

    // Connection event handlers
    socket.on('connect', function () {
        console.log('WebSocket connected');
        websocketConnected = true;
        updateWebSocketStatus(true);
        showStatus('success', 'WebSocket', 'Real-time connection established');
    });

    socket.on('disconnect', function () {
        console.log('WebSocket disconnected');
        websocketConnected = false;
        updateWebSocketStatus(false);
        showStatus('warning', 'WebSocket', 'Real-time connection lost');
    });

    socket.on('connect_error', function (error) {
        console.error('WebSocket connection error:', error);
        websocketConnected = false;
        updateWebSocketStatus(false);
        showStatus('danger', 'WebSocket', 'Connection failed: ' + error.message);
    });

    // Joint state update handler
    socket.on('joint_state_update', function (data) {
        console.log('Received joint state update:', data);
        updateRealTimePosition(data);
    });

    // Connection status handler
    socket.on('connection_status', function (data) {
        console.log('Connection status:', data);
        if (data.status === 'connected') {
            showStatus('info', 'WebSocket', data.message);
        }
    });
}

/**
 * Update the WebSocket connection status indicator.
 * @param {boolean} connected - Whether the WebSocket is connected
 */
function updateWebSocketStatus(connected) {
    const indicator = document.getElementById('websocket-indicator');
    if (connected) {
        indicator.className = 'badge bg-success me-2';
        indicator.innerHTML = '<i class="fas fa-plug"></i> WebSocket Connected';
    } else {
        indicator.className = 'badge bg-danger me-2';
        indicator.innerHTML = '<i class="fas fa-plug"></i> WebSocket Disconnected';
    }
}

/**
 * Update real-time position displays from WebSocket data.
 * Updates joint position displays and timestamp, then syncs sliders.
 * @param {Object} data - Joint state data from WebSocket
 * @param {Object} data.joint1 - Joint 1 position data
 * @param {Object} data.joint2 - Joint 2 position data
 * @param {number} data.timestamp - Timestamp of the position update
 */
function updateRealTimePosition(data) {
    if (data.error) {
        console.error('Joint state error:', data.error);
        return;
    }

    // Update joint position displays
    const joint1Element = document.getElementById('realtime-joint1');
    const joint2Element = document.getElementById('realtime-joint2');
    const timestampElement = document.getElementById('position-timestamp');

    if (joint1Element && data.joint1) {
        joint1Element.textContent = data.joint1.position_degrees.toFixed(1) + '°';
    }

    if (joint2Element && data.joint2) {
        joint2Element.textContent = data.joint2.position_degrees.toFixed(1) + '°';
    }

    if (timestampElement && data.timestamp) {
        const date = new Date(data.timestamp * 1000);
        timestampElement.textContent = 'Last update: ' + date.toLocaleTimeString();
    }

    // Update sliders to match real-time position
    updateSlidersFromRealTime(data);
}

/**
 * Update slider positions and values from real-time joint data.
 * Synchronizes slider UI with actual robot joint positions.
 * @param {Object} data - Joint state data from WebSocket
 * @param {Object} data.joint1 - Joint 1 position data with position_degrees property
 * @param {Object} data.joint2 - Joint 2 position data with position_degrees property
 */
function updateSlidersFromRealTime(data) {
    if (!data.joint1 || !data.joint2) return;

    const joint1Slider = document.getElementById('joint1');
    const joint2Slider = document.getElementById('joint2');
    const joint1Value = document.getElementById('joint1-value');
    const joint2Value = document.getElementById('joint2-value');

    if (joint1Slider && joint1Value) {
        joint1Slider.value = data.joint1.position_degrees;
        joint1Value.textContent = data.joint1.position_degrees.toFixed(1) + '°';
    }

    if (joint2Slider && joint2Value) {
        joint2Slider.value = data.joint2.position_degrees;
        joint2Value.textContent = data.joint2.position_degrees.toFixed(1) + '°';
    }
}

/**
 * Request a joint state update from the robot via WebSocket.
 * Only works when WebSocket connection is active.
 */
function requestJointUpdate() {
    if (socket && websocketConnected) {
        socket.emit('request_joint_update');
    } else {
        console.warn('WebSocket not connected, cannot request joint update');
    }
}
