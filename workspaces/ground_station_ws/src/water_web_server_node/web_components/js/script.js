const API_MISSION_GO = "/api/mission/go";
const API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const API_AUTO_APPROACH = "/api/mission/auto_approach";
const API_AUTO_SHOOT = "/api/mission/auto_shoot";
const API_SHOOT = "/api/mission/shoot";
const TAKE_PICTURE = "/api/mission/take_picture";
const API_ABORT_ALL = "/api/mission/abort_all";
const API_GIMBAL_FOLLOW = "/api/gimbal_follow";
const API_GIMBAL_LOCK = "/api/gimbal_lock";


function appendToLog(msg, type = 'info') {
    const logsContainer = document.querySelector('.logs-container');
    const newLog = document.createElement('span');
    const timestamp = new Date().toLocaleTimeString("it-IT");
    
    newLog.textContent = `[${timestamp}] ${msg}`;
    newLog.classList.add(type);
    logsContainer.appendChild(newLog);
    logsContainer.scrollTop = logsContainer.scrollHeight;
}

async function sendCommand(endpoint) {
    try {
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
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
    document.getElementById('btn-follow').addEventListener('click', () => sendCommand(API_GIMBAL_FOLLOW));
    document.getElementById('btn-lock').addEventListener('click', () => sendCommand(API_GIMBAL_LOCK));
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
                case "new_picture":
                    console.log("New picture: ", data.url);
                    document.getElementById("target-image").src = data.url;
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

document.addEventListener('DOMContentLoaded', () => {
    initaliseButtons();
    loadTheme();
    initaliseSocket();
})