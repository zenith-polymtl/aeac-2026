const API_MISSION_GO = "/api/mission/go";
const API_START_LAP = "/api/mission/lap/start";
const API_FINISH_LAP = "/api/mission/lap/finish";
const API_STOP_LAP = "/api/mission/lap/stop";
const API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const API_SERVO = "/api/mission/servo";
const TAKE_PICTURE = "/api/mission/take_picture";
const API_ABORT_ALL = "/api/mission/abort_all";

function appendToLog(msg, type = 'info') {
    const logsContainer = document.querySelector('.logs-container');
    const newLog = document.createElement('span');
    const timestamp = new Date().toLocaleTimeString("it-IT");
    
    newLog.textContent = `[${timestamp}] ${msg}`;
    newLog.classList.add(type);
    logsContainer.appendChild(newLog);
    logsContainer.scrollTop = logsContainer.scrollHeight;
}

async function sendCommand(endpoint, body = null) {
    try {
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(body),
        });
        if (response.ok) {
            // const data = await response.json();
            // const successMessage = `Success: ${JSON.stringify(data.message)}`;
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
    document.getElementById('start-lap-button').addEventListener('click', () => sendCommand(API_START_LAP));
    document.getElementById('finish-lap-button').addEventListener('click', () => sendCommand(API_FINISH_LAP));
    document.getElementById('stop-lap-button').addEventListener('click', () => sendCommand(API_STOP_LAP));
    document.getElementById('go-to-site-button').addEventListener('click', () => sendCommand(API_MOVE_TO_SCENE));
    document.getElementById('servo-1-toggle').addEventListener('change', (e) => {
        const pwm = e.target.checked ? 2000 : 1000;
        sendCommand(API_SERVO, {servo_num: 1, pwm});
    });
    document.getElementById('servo-2-toggle').addEventListener('change', (e) => {
        const pwm = e.target.checked ? 2000 : 1000;
        sendCommand(API_SERVO, {servo_num: 2, pwm});
    });
    document.getElementById('take-picture-button').addEventListener('click', () => sendCommand(TAKE_PICTURE));
    document.getElementById('abort-button').addEventListener('click', () => sendCommand(API_ABORT_ALL));
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
                    appendToLog(`${data.message}`, data.is_success ? 'info':'error');
                    break;
                case "connection":
                    const connected_element =  document.getElementById("connection-span");
                    connected_element.innerHTML = "Connection: " + (data.is_connected ? "Connected" : "Disconneted")
                    break;
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