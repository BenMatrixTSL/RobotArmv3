/**
 * USB camera view for the robot arm app.
 *
 * On the Pi, camera-stream.py runs on port 8082.
 * The web server (serve-app.py) proxies /camera/stream on port 80 (same origin).
 */

var cameraStreamActive = false;
var cameraRetryTimer = null;
var cameraRetryCount = 0;
var cameraUrlIndex = 0;
var cameraUrlList = [];

/**
 * Build list of stream URLs to try (same-origin proxy first, then port 8082).
 */
function buildCameraStreamUrlList() {
    var urls = [];
    var host = '127.0.0.1';

    if (window.location.search.indexOf('kiosk=1') < 0) {
        var addrInput = document.getElementById('piAddress');
        if (addrInput && addrInput.value) {
            host = addrInput.value.trim();
        }
    }

    // Same port as the web page (proxied by serve-app.py on the Pi)
    if (window.location.protocol === 'http:' || window.location.protocol === 'https:') {
        urls.push(window.location.origin + '/camera/stream');
    }

    // Direct to camera service
    urls.push('http://' + host + ':8082/stream');

    return urls;
}

function setCameraStatus(message, color) {
    var status = document.getElementById('cameraStreamStatus');
    if (status) {
        status.textContent = message;
        status.style.color = color || '#5c6370';
    }
}

/**
 * Start showing the camera stream.
 */
function startCameraView() {
    var img = document.getElementById('cameraStreamImage');
    var frame = document.getElementById('cameraStreamFrame');

    if (!img) {
        return;
    }

    cameraStreamActive = true;
    cameraRetryCount = 0;
    cameraUrlIndex = 0;
    cameraUrlList = buildCameraStreamUrlList();

    if (frame) {
        frame.classList.add('camera-stream-active');
    }

    tryNextCameraUrl();
}

function tryNextCameraUrl() {
    var img = document.getElementById('cameraStreamImage');

    if (!cameraStreamActive || !img) {
        return;
    }

    if (cameraUrlIndex >= cameraUrlList.length) {
        cameraRetryCount = cameraRetryCount + 1;
        if (cameraRetryCount <= 8) {
            cameraUrlIndex = 0;
            setCameraStatus('Waiting for camera... retry ' + cameraRetryCount + ' of 8', '#5c6370');
            cameraRetryTimer = setTimeout(tryNextCameraUrl, 2000);
            return;
        }
        setCameraStatus('Camera not available. Check: sudo systemctl status robot-arm-camera.service', '#e74c3c');
        return;
    }

    var url = cameraUrlList[cameraUrlIndex];
    var bust = url + (url.indexOf('?') >= 0 ? '&' : '?') + 't=' + String(Date.now());

    setCameraStatus('Connecting to ' + url + ' ...', '#5c6370');

    img.onload = function () {
        setCameraStatus('Camera live', '#27ae60');
    };

    img.onerror = function () {
        if (!cameraStreamActive) {
            return;
        }
        cameraUrlIndex = cameraUrlIndex + 1;
        cameraRetryTimer = setTimeout(tryNextCameraUrl, 500);
    };

    // MJPEG streams may not fire onload — show live after a short wait if no error
    if (cameraRetryTimer) {
        clearTimeout(cameraRetryTimer);
    }
    cameraRetryTimer = setTimeout(function () {
        if (cameraStreamActive && img.src) {
            setCameraStatus('Camera live', '#27ae60');
        }
    }, 3000);

    img.src = bust;
}

/**
 * Stop the camera stream.
 */
function stopCameraView() {
    var img = document.getElementById('cameraStreamImage');
    var frame = document.getElementById('cameraStreamFrame');

    cameraStreamActive = false;
    cameraUrlIndex = 0;
    cameraUrlList = [];

    if (cameraRetryTimer) {
        clearTimeout(cameraRetryTimer);
        cameraRetryTimer = null;
    }

    if (img) {
        img.onload = null;
        img.onerror = null;
        img.removeAttribute('src');
    }

    if (frame) {
        frame.classList.remove('camera-stream-active');
    }

    setCameraStatus('Stopped', '#5c6370');
}

function refreshCameraView() {
    stopCameraView();
    startCameraView();
}

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
