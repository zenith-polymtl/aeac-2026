const API_MISSION_GO = "/api/mission/go";
const API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const API_AUTO_APPROACH = "/api/mission/move_to_scene";
const API_AUTO_SHOOT = "/api/mission/move_to_scene";
const API_SHOOT = "/api/mission/move_to_scene";
const API_ABORT_ALL = "/api/mission/abort_all";
const API_GIMBAL_TOGGLE = "/api/toogle_gimbal"


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
            const successMessage = `Success: ${JSON.stringify(data)}`;
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
    document.getElementById('shoot-button').addEventListener('click', () => sendCommand(API_SHOOT));
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

function initaliseGimbalLogic() {
    const gimbalToggle = document.getElementById('gimbal-toggle');

    gimbalToggle.addEventListener('change', (event) => {
        sendCommand(API_GIMBAL_TOGGLE);
        if (event.target.checked) {
            console.log("Gimbal Control: ACTIVATED");
        } else {
            console.log("Gimbal Control: DEACTIVATED");
        }
    });
}

document.addEventListener('DOMContentLoaded', () => {
    initaliseButtons();
    initaliseGimbalLogic();
    loadTheme();

    const ws = new WebSocket("ws://" + window.location.host + "/ws/status");

    ws.onopen = function() {
        console.log("WebSocket connected!");
    };

    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            console.log("Status update:", data);
            appendToLog(`Received Message: ${data.message}`, data.is_success ? 'info':'error')
            
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
})