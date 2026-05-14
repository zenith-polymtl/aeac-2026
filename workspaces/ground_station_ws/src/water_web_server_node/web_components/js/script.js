const API_MISSION_GO = "/api/mission/go";
const API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const API_AUTO_APPROACH = "/api/mission/auto_approach";
const API_AUTO_SHOOT = "/api/mission/auto_shoot";
const API_SHOOT = "/api/mission/shoot";
const TAKE_PICTURE = "/api/mission/take_picture";
const API_ABORT_ALL = "/api/mission/abort_all";
const API_GIMBAL_FOLLOW = "/api/gimbal_follow";
const API_GIMBAL_LOCK = "/api/gimbal_lock";
const API_CONFIRM_TARGET = "/api/confirm_target";
const API_SET_GIMBAL_OFFSET = "/api/mission/set_gimbal_offset";

function appendToLog(msg, type = 'info') {
    const logsContainer = document.querySelector('.logs-container');
    const newLog = document.createElement('span');
    const timestamp = new Date().toLocaleTimeString("it-IT");
    
    newLog.textContent = `[${timestamp}] ${msg}`;
    newLog.classList.add(type);
    logsContainer.appendChild(newLog);
    logsContainer.scrollTop = logsContainer.scrollHeight;
}

async function sendCommand(endpoint, payload = {}) {
    try {
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(payload)
        });
        if (response.ok) {
            const data = await response.json();
            const successMessage = `Success: ${JSON.stringify(data.message)}`;
            // appendToLog(successMessage)
        } else {
            const errorMessage = `Error: ${response.statusText}`;
        }
    } catch (error) {
        const errorMessage = `Error: ${error.message}`;
    }
}

function initaliseButtons() {
    document.getElementById('start-button').addEventListener('click', () => sendCommand(API_MISSION_GO));
    document.getElementById('go-to-site-button').addEventListener('click', () => sendCommand(API_MOVE_TO_SCENE));
    document.getElementById('auto-approach-button').addEventListener('click', () => sendCommand(API_AUTO_APPROACH));
    document.getElementById('auto-shoot-button').addEventListener('click', () => sendCommand(API_AUTO_SHOOT));
    document.getElementById('take-picture-button').addEventListener('click', () => sendCommand(TAKE_PICTURE));
    document.getElementById('shoot-button').addEventListener('click', () => sendCommand(API_SHOOT));
    document.getElementById('abort-button').addEventListener('click', () => sendCommand(API_ABORT_ALL));
    document.getElementById('btn-follow').addEventListener('click', () => {
        sendCommand(API_GIMBAL_FOLLOW);
        document.getElementById('btn-lock').classList.remove("active")
        document.getElementById('btn-follow').classList.add("active")
    });
    document.getElementById('btn-lock').addEventListener('click', () => {
        sendCommand(API_GIMBAL_LOCK);
        document.getElementById('btn-lock').classList.add("active")
        document.getElementById('btn-follow').classList.remove("active")

    });
    document.getElementById('accept-image-button').addEventListener('click', () => {
        sendCommand(API_CONFIRM_TARGET, {confirmed: true});
        clear_picture();
    });
    document.getElementById('deny-image-deny').addEventListener('click', () => {
        sendCommand(API_CONFIRM_TARGET, {confirmed: false});
        clear_picture();
    });
    document.getElementById('apply-offset-button').addEventListener('click', () => {
        sendCommand(API_SET_GIMBAL_OFFSET, { x: offsetX, y: offsetY });
    });
    document.getElementById('reset-offset-button').addEventListener('click', () => {
        updateOffset(0, 0, 'reset');
    });
}

function clear_picture() {
    document.getElementById("target-image").style.display = "none";
    const noPictureReceived = document.getElementById("no-picture-received");
    noPictureReceived.style.display = "block";
    noPictureReceived.textContent = "No more targets to confirm. Waiting for new targets...";
    document.getElementById('accept-image-button').classList.add('disabled');
    document.getElementById('deny-image-deny').classList.add('disabled');
}

function loadTheme() {
    const logoBtn = document.getElementById("logo-image")
    const body = document.body;

    if (localStorage.getItem('theme') === 'dark') {
        body.setAttribute('data-theme', 'dark');
    }

    logoBtn.addEventListener('click', () => {
        if (body.getAttribute('data-theme') === 'dark') {
            body.removeAttribute('data-theme');
            localStorage.setItem('theme', 'light');
        } else {
            body.setAttribute('data-theme', 'dark');
            localStorage.setItem('theme', 'dark');
        }
    });
}

function connection_logique(data) {
    console.log("Dronce connection change: ", data)
    const drone_connected_element =  document.getElementById("connection-span");
    drone_connected_element.innerHTML = "Drone Connection: " + (data.drone_is_connected ? "Connected" : "Disconneted")

    const zed_connected_element =  document.getElementById("zed-connection-span");
    zed_connected_element.innerHTML = "Zed Connection: " + (data.zed_is_connected ? "Connected" : "Disconneted")
}


function initaliseSocket() {
    const ws = new WebSocket("ws://" + window.location.host + "/ws/status");

    ws.onopen = function() {
        console.log("WebSocket connected!");
    };

    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);

            switch(data.type) {
                case "message":
                    console.log("Status update:", data);
                    appendToLog(`${data.message}`, data.is_success ? 'info':'error')
                    break;
                    
                case "gimbal_state":
                    const gimbalMode = document.getElementById('gimbal-mode-val');
                    const pitchElement = document.getElementById('pitch-val');
                    const yawElement = document.getElementById('yaw-val');

                    gimbalMode.textContent = data.mode;
                    pitchElement.textContent = data.pitch.toFixed(2);
                    yawElement.textContent = data.yaw.toFixed(2);
                    break;
                case "connection":
                    connection_logique(data);
                    break;
                case "new_picture":
                    console.log("New picture: ", data.url);
                    document.getElementById("no-picture-received").style.display = "none";
                    const imageElement = document.getElementById("target-image");
                    imageElement.style.display = "block";
                    imageElement.src = data.url;
                    document.getElementById('accept-image-button').classList.remove('disabled');
                    document.getElementById('deny-image-deny').classList.remove('disabled');
                    break;
                case "target_number":
                    const targetNumberElement = document.getElementById("target-shot-span");
                    targetNumberElement.textContent = `Target Shot: ${data.number}`;
                    break;
                case "state_change":
                    document.getElementById('mission-status-span').innerText = `Mission Status: ${data.state}`
                default:
                    console.warn("Unknown message type:", data.type);
            }
        } catch (e) {
            console.error("Invalid JSON:", event.data);
        }
    };

    ws.onclose = function() {
        console.log("WebSocket disconnected");
    };

    ws.onerror = function(error) {
        console.error("WebSocket error:", error);
    };
}

let offsetX = 0, offsetY = 0;

function updateOffset(x, y, source) {
    offsetX = Math.max(-100, Math.min(100, Math.round(x)));
    offsetY = Math.max(-100, Math.min(100, Math.round(y)));
    const px = ((offsetX + 100) / 200) * 100;
    const py = ((offsetY + 100) / 200) * 100;
    document.getElementById('pad-dot').style.left = px + '%';
    document.getElementById('pad-dot').style.top = py + '%';
    if (source !== 'x-slider') document.getElementById('x-slider').value = offsetX;
    if (source !== 'y-slider') document.getElementById('y-slider').value = offsetY;
    if (source !== 'x-num') document.getElementById('x-num').value = offsetX;
    if (source !== 'y-num') document.getElementById('y-num').value = offsetY;
}

function initialiseOffset() {
    const pad = document.getElementById('offset-pad');
    let dragging = false;

    function padEvent(e) {
        e.preventDefault();
        const rect = pad.getBoundingClientRect();
        const clientX = e.touches ? e.touches[0].clientX : e.clientX;
        const clientY = e.touches ? e.touches[0].clientY : e.clientY;
        const rx = Math.max(0, Math.min(1, (clientX - rect.left) / rect.width));
        const ry = Math.max(0, Math.min(1, (clientY - rect.top) / rect.height));
        updateOffset(rx * 200 - 100, ry * 200 - 100, 'pad');
    }

    pad.addEventListener('mousedown', e => { dragging = true; padEvent(e); });
    document.addEventListener('mousemove', e => { if (dragging) padEvent(e); });
    document.addEventListener('mouseup', () => { dragging = false; });
    pad.addEventListener('touchstart', padEvent, { passive: false });
    pad.addEventListener('touchmove', padEvent, { passive: false });

    document.getElementById('x-slider').addEventListener('input', () => updateOffset(+document.getElementById('x-slider').value, offsetY, 'x-slider'));
    document.getElementById('y-slider').addEventListener('input', () => updateOffset(offsetX, +document.getElementById('y-slider').value, 'y-slider'));
    document.getElementById('x-num').addEventListener('change', () => updateOffset(+document.getElementById('x-num').value, offsetY, 'x-num'));
    document.getElementById('y-num').addEventListener('change', () => updateOffset(offsetX, +document.getElementById('y-num').value, 'y-num'));
}

document.addEventListener('DOMContentLoaded', () => {
    initaliseButtons();
    loadTheme();
    initaliseSocket();
    initialiseOffset();
})