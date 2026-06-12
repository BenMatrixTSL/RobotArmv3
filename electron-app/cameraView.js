/**
 * USB camera view for the robot arm app.
 *
 * On the Pi, a small MJPEG server runs on port 8082 (camera-stream.py).
 * This file shows that stream in the Camera tab.
 */

var cameraStreamActive = false;

/**
 * Build the camera stream URL.
 * Kiosk on the Pi uses 127.0.0.1; PC app uses the Pi address field.
 */
function getCameraStreamUrl() {
    var port = 8082;
    var host = '127.0.0.1';

    if (window.location.search.indexOf('kiosk=1') < 0) {
        var addrInput = document.getElementById('piAddress');
        if (addrInput && addrInput.value) {
            host = addrInput.value.trim();
        }
    }

    return 'http://' + host + ':' + port + '/stream';
}

/**
 * Start showing the camera in the img element.
 */
function startCameraView() {
    var img = document.getElementById('cameraStreamImage');
    var status = document.getElementById('cameraStreamStatus');

    if (!img) {
        return;
    }

    var url = getCameraStreamUrl();
    cameraStreamActive = true;

    if (status) {
        status.textContent = 'Connecting to ' + url + ' ...';
        status.style.color = '#5c6370';
    }

    img.onload = function () {
        if (status) {
            status.textContent = 'Camera live';
            status.style.color = '#27ae60';
        }
    };

    img.onerror = function () {
        if (!cameraStreamActive) {
            return;
        }
        if (status) {
            status.textContent = 'Camera not available. On the Pi run: sudo ./install-camera-service.sh';
            status.style.color = '#e74c3c';
        }
    };

    // Cache-bust so reopening the tab gets a fresh stream (? is OK — server ignores query)
    img.src = url + '?t=' + String(Date.now());
}

/**
 * Stop the camera stream when leaving the tab.
 */
function stopCameraView() {
    var img = document.getElementById('cameraStreamImage');
    var status = document.getElementById('cameraStreamStatus');

    cameraStreamActive = false;

    if (img) {
        img.onload = null;
        img.onerror = null;
        img.removeAttribute('src');
    }

    if (status) {
        status.textContent = 'Stopped';
        status.style.color = '#5c6370';
    }
}

/**
 * Refresh button handler.
 */
function refreshCameraView() {
    stopCameraView();
    startCameraView();
}

/**
 * Wire up buttons when the page loads.
 */
function initializeCameraTab() {
    var startBtn = document.getElementById('cameraStartButton');
    var stopBtn = document.getElementById('cameraStopButton');
    var refreshBtn = document.getElementById('cameraRefreshButton');

    if (startBtn) {
        startBtn.addEventListener('click', startCameraView);
    }
    if (stopBtn) {
        stopBtn.addEventListener('click', stopCameraView);
    }
    if (refreshBtn) {
        refreshBtn.addEventListener('click', refreshCameraView);
    }
}
