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
var cameraVisionStatus = null;

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

            var msg;
            if (parts.length > 0) {
                msg = 'Camera live — ' + parts.join(', ');
            } else if (data.dictionary) {
                msg = 'Camera live — vision on, no markers in view';
            } else {
                msg = 'Camera live — no markers or blocks detected';
            }
            cameraVisionStatus = msg;
            setCameraStatus(msg, '#27ae60');
        })
        .catch(function () {
            // Vision endpoint may be unavailable on older ffmpeg-only installs.
        });
}

// ===== Detected block scanning =====
// Shared by the G-code (M780/M781), RAPID (GetBlockCount/SaveBlockToPos) and
// Blockly (Vision category) programming modes, so "how many blocks / what is
// at index N" behaves identically no matter which language calls it. Blocks
// are sorted by image Y ascending — the block nearest the top of what the
// camera sees gets index 0, so a block "below" another (further down in the
// camera's view) always gets a higher index. This works even when no ArUco
// markers are visible (pixel coordinates are always present); world_x_mm/
// world_y_mm are only added by camera-vision.py once 3-4 ArUco markers give
// it a homography, so hasWorldCoords can be false right after starting the
// vision service.

/**
 * Fetches current vision data and returns blocks sorted top-of-view first.
 * @returns {Promise<Array<{index:number, color:string, pixelX:number, pixelY:number, worldX:number|null, worldY:number|null, hasWorldCoords:boolean}>>}
 */
function getSortedDetectedBlocks() {
    return fetch(buildCameraVisionUrl())
        .then(function (response) {
            if (!response.ok) {
                throw new Error('Vision endpoint returned ' + response.status);
            }
            return response.json();
        })
        .then(function (data) {
            var blocks = (data && Array.isArray(data.blocks)) ? data.blocks.slice() : [];
            blocks.sort(function (a, b) {
                if (a.pixel_y !== b.pixel_y) return a.pixel_y - b.pixel_y;
                return a.pixel_x - b.pixel_x;
            });
            return blocks.map(function (b, i) {
                var hasWorld = typeof b.world_x_mm === 'number' && typeof b.world_y_mm === 'number';
                return {
                    index: i,
                    color: b.color,
                    pixelX: b.pixel_x,
                    pixelY: b.pixel_y,
                    worldX: hasWorld ? b.world_x_mm : null,
                    worldY: hasWorld ? b.world_y_mm : null,
                    hasWorldCoords: hasWorld
                };
            });
        });
}

/**
 * @returns {Promise<number>} How many coloured blocks the camera currently sees
 */
function getDetectedBlockCount() {
    return getSortedDetectedBlocks().then(function (blocks) { return blocks.length; });
}

/**
 * @param {number} index - Position in the sorted list (0 = topmost in view)
 * @returns {Promise<Object|null>} The block at that index, or null if out of range
 */
function getDetectedBlockAt(index) {
    return getSortedDetectedBlocks().then(function (blocks) {
        if (index < 0 || index >= blocks.length) return null;
        return blocks[index];
    });
}

/**
 * Convenience getters for Blockly's value blocks — throw (rather than
 * return null/NaN) so a bad index surfaces as a clear error in the Blockly
 * output log instead of silently sending the arm somewhere wrong.
 */
async function getDetectedBlockXAt(index) {
    var block = await getDetectedBlockAt(index);
    if (!block) throw new Error('No detected block at index ' + index);
    if (!block.hasWorldCoords) throw new Error('Block ' + index + ' has no world coordinates — ArUco markers must be visible first');
    return block.worldX;
}

async function getDetectedBlockYAt(index) {
    var block = await getDetectedBlockAt(index);
    if (!block) throw new Error('No detected block at index ' + index);
    if (!block.hasWorldCoords) throw new Error('Block ' + index + ' has no world coordinates — ArUco markers must be visible first');
    return block.worldY;
}

async function getDetectedBlockColorAt(index) {
    var block = await getDetectedBlockAt(index);
    if (!block) throw new Error('No detected block at index ' + index);
    return block.color;
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
        setCameraStatus('Camera not available. Check that the camera service is running.', '#e74c3c');
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
        setCameraStatus(cameraVisionStatus || 'Camera live', '#27ae60');
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
    cameraVisionStatus = null;

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
