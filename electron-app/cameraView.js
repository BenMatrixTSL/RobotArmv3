/**
 * USB camera view for the robot arm app.
 *
 * Uses repeated JPEG snapshots (works reliably in Chromium on the Pi).
 * serve-app.py proxies /camera/snapshot on port 80.
 */

var cameraStreamActive = false;
var cameraSnapshotTimer = null;
var cameraFailCount = 0;
var cameraUrlIndex = 0;
var cameraUrlList = [];
var cameraVisionPollCount = 0;

function buildCameraSnapshotUrlList() {
    var urls = [];
    var host = '127.0.0.1';

    if (window.location.search.indexOf('kiosk=1') < 0) {
        var addrInput = document.getElementById('piAddress');
        if (addrInput && addrInput.value) {
            host = addrInput.value.trim();
        }
    }

    if (window.location.protocol === 'http:' || window.location.protocol === 'https:') {
        urls.push(window.location.origin + '/camera/snapshot');
    }

    urls.push('http://' + host + ':8082/snapshot');

    return urls;
}

function buildCameraVisionUrl() {
    var host = '127.0.0.1';

    if (window.location.search.indexOf('kiosk=1') < 0) {
        var addrInput = document.getElementById('piAddress');
        if (addrInput && addrInput.value) {
            host = addrInput.value.trim();
        }
    }

    if (window.location.protocol === 'http:' || window.location.protocol === 'https:') {
        return window.location.origin + '/camera/vision';
    }

    return 'http://' + host + ':8082/vision';
}

function updateCameraStatusFromVision() {
    if (!cameraStreamActive) {
        return;
    }

    var visionUrl = buildCameraVisionUrl();

    fetch(visionUrl)
        .then(function (response) {
            if (!response.ok) {
                return null;
            }
            return response.json();
        })
        .then(function (data) {
            if (!cameraStreamActive || !data) {
                return;
            }

            var parts = [];
            var markerCount = data.markers ? data.markers.length : 0;
            var blockCount = data.blocks ? data.blocks.length : 0;

            if (markerCount > 0) {
                parts.push(markerCount + ' ArUco marker' + (markerCount === 1 ? '' : 's'));
            }
            if (blockCount > 0) {
                parts.push(blockCount + ' coloured block' + (blockCount === 1 ? '' : 's'));
            }

            if (parts.length > 0) {
                setCameraStatus('Camera live — ' + parts.join(', '), '#27ae60');
            } else if (data.dictionary) {
                setCameraStatus('Camera live — vision on, no markers in view', '#27ae60');
            } else {
                setCameraStatus('Camera live — no markers or blocks detected', '#27ae60');
            }
        })
        .catch(function () {
            // Vision endpoint may be unavailable on older ffmpeg-only installs.
        });
}

function setCameraStatus(message, color) {
    var status = document.getElementById('cameraStreamStatus');
    if (status) {
        status.textContent = message;
        status.style.color = color || '#5c6370';
    }
}

function updateCameraButtons(running) {
    var startBtn = document.getElementById('cameraStartButton');
    var stopBtn = document.getElementById('cameraStopButton');

    if (startBtn) {
        startBtn.disabled = running;
        if (running) {
            startBtn.classList.remove('btn-primary');
            startBtn.classList.add('btn-secondary');
        } else {
            startBtn.classList.add('btn-primary');
            startBtn.classList.remove('btn-secondary');
        }
    }

    if (stopBtn) {
        stopBtn.disabled = !running;
    }
}

function showCameraFrame() {
    var img = document.getElementById('cameraStreamImage');
    var frame = document.getElementById('cameraStreamFrame');

    if (frame) {
        frame.classList.add('camera-stream-active');
    }

    if (img) {
        img.style.display = 'block';
    }
}

function hideCameraFrame() {
    var img = document.getElementById('cameraStreamImage');
    var frame = document.getElementById('cameraStreamFrame');

    if (frame) {
        frame.classList.remove('camera-stream-active');
    }

    if (img) {
        img.onload = null;
        img.onerror = null;
        img.removeAttribute('src');
        img.style.display = '';
    }
}

function getCurrentSnapshotBaseUrl() {
    if (cameraUrlList.length === 0) {
        cameraUrlList = buildCameraSnapshotUrlList();
    }
    if (cameraUrlIndex >= cameraUrlList.length) {
        return null;
    }
    return cameraUrlList[cameraUrlIndex];
}

function tryNextSnapshotUrl() {
    cameraUrlIndex = cameraUrlIndex + 1;
    cameraFailCount = 0;
    if (cameraUrlIndex >= cameraUrlList.length) {
        setCameraStatus('Camera not available. Check: sudo systemctl status robot-arm-camera.service', '#e74c3c');
        updateCameraButtons(false);
        hideCameraFrame();
        return;
    }
    pollCameraSnapshot();
}

function pollCameraSnapshot() {
    var img = document.getElementById('cameraStreamImage');
    var baseUrl = getCurrentSnapshotBaseUrl();

    if (!cameraStreamActive || !img || !baseUrl) {
        return;
    }

    var url = baseUrl + '?t=' + String(Date.now());

    img.onload = function () {
        if (!cameraStreamActive) {
            return;
        }

        cameraFailCount = 0;
        showCameraFrame();
        setCameraStatus('Camera live', '#27ae60');
        updateCameraButtons(true);

        cameraVisionPollCount = cameraVisionPollCount + 1;
        if (cameraVisionPollCount % 4 === 0) {
            updateCameraStatusFromVision();
        }

        cameraSnapshotTimer = setTimeout(pollCameraSnapshot, 150);
    };

    img.onerror = function () {
        if (!cameraStreamActive) {
            return;
        }

        cameraFailCount = cameraFailCount + 1;

        if (cameraFailCount >= 8) {
            setCameraStatus('Trying another camera URL...', '#5c6370');
            tryNextSnapshotUrl();
            return;
        }

        setCameraStatus('Waiting for camera... (' + cameraFailCount + ' of 8)', '#5c6370');
        cameraSnapshotTimer = setTimeout(pollCameraSnapshot, 500);
    };

    img.src = url;
}

function startCameraView() {
    var img = document.getElementById('cameraStreamImage');

    if (!img) {
        return;
    }

    if (cameraSnapshotTimer) {
        clearTimeout(cameraSnapshotTimer);
        cameraSnapshotTimer = null;
    }

    cameraStreamActive = true;
    cameraFailCount = 0;
    cameraUrlIndex = 0;
    cameraVisionPollCount = 0;
    cameraUrlList = buildCameraSnapshotUrlList();

    setCameraStatus('Connecting to camera...', '#5c6370');
    updateCameraButtons(true);
    pollCameraSnapshot();
}

function stopCameraView() {
    cameraStreamActive = false;
    cameraFailCount = 0;
    cameraUrlIndex = 0;
    cameraUrlList = [];

    if (cameraSnapshotTimer) {
        clearTimeout(cameraSnapshotTimer);
        cameraSnapshotTimer = null;
    }

    hideCameraFrame();
    updateCameraButtons(false);
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

    updateCameraButtons(false);
}
