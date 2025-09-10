// DrRoboto Web Control Interface JavaScript

let commandsSent = 0;
let successfulCommands = 0;

// Initialize the interface
document.addEventListener('DOMContentLoaded', function () {
    initializeSliders();
    updateStats();
    checkConnection();
});

// Initialize joint sliders
function initializeSliders() {
    const joint1Slider = document.getElementById('joint1');
    const joint2Slider = document.getElementById('joint2');
    const joint1Value = document.getElementById('joint1-value');
    const joint2Value = document.getElementById('joint2-value');

    joint1Slider.addEventListener('input', function () {
        joint1Value.textContent = this.value + '°';
    });

    joint2Slider.addEventListener('input', function () {
        joint2Value.textContent = this.value + '°';
    });
}

// Move robot to a predefined pose
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

// Set custom joint positions
async function setCustomJoints() {
    const joint1 = document.getElementById('joint1').value;
    const joint2 = document.getElementById('joint2').value;

    showLoading();

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
            showStatus('success', 'Joints Set', data.message);
            updateLastCommand(`Custom: J1=${joint1}°, J2=${joint2}°`);
        } else {
            showStatus('danger', 'Error', data.error);
        }
    } catch (error) {
        showStatus('danger', 'Error', 'Failed to send command: ' + error.message);
    } finally {
        hideLoading();
        updateStats();
    }
}

// Reset joints to home position
function resetJoints() {
    document.getElementById('joint1').value = 0;
    document.getElementById('joint2').value = 0;
    document.getElementById('joint1-value').textContent = '0°';
    document.getElementById('joint2-value').textContent = '0°';
    moveToPose('home');
}

// Perform wave gesture
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

// Emergency stop (move to home)
function emergencyStop() {
    showStatus('warning', 'Emergency Stop', 'Moving to home position...');
    moveToPose('home');
}

// Get robot status
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

// Get available topics
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

// Check connection health
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

// Update connection status indicator
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

// Show status message
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

// Update last command display
function updateLastCommand(command) {
    document.getElementById('last-command').textContent = command;
}

// Update statistics
function updateStats() {
    commandsSent++;
    const successRate = commandsSent > 0 ? Math.round((successfulCommands / commandsSent) * 100) : 100;

    document.getElementById('commands-sent').textContent = commandsSent;
    document.getElementById('success-rate').textContent = successRate + '%';
}

// Show loading modal
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

// Hide loading modal
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
