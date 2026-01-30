const API_MISSION_GO = "/api/mission/go";
const API_START_LAP = "/api/mission/lap/start";
const API_FINISH_LAP = "/api/mission/lap/finish";
const API_MOVE_TO_SCENE = "/api/mission/move_to_scene";
const API_ABORT_ALL = "/api/mission/abort_all";

const statusElement = document.getElementById('status');
const logElement = document.getElementById('log');

function appendToLog(message, type = 'info') {
    const p = document.createElement('p');
    p.textContent = `${new Date().toLocaleString()} - ${message}`;
    p.classList.add(type);
    logElement.appendChild(p);
    logElement.scrollTop = logElement.scrollHeight;
}

async function sendCommand(endpoint) {
    statusElement.textContent = 'Status: Sending command...';
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
            statusElement.textContent = `Status: ${successMessage}`;
            appendToLog(`Command to ${endpoint}: ${successMessage}`, 'success');
        } else {
            const errorMessage = `Error: ${response.statusText}`;
            statusElement.textContent = `Status: ${errorMessage}`;
            appendToLog(`Command to ${endpoint}: ${errorMessage}`, 'error');       
        }
    } catch (error) {
        const errorMessage = `Error: ${error.message}`;
        statusElement.textContent = `Status: ${errorMessage}`;
        appendToLog(`Command to ${endpoint}: ${errorMessage}`, 'error');    
    }
}

function initalise_buttons() {
    document.getElementById('mission-go').addEventListener('click', () => sendCommand(API_MISSION_GO));
    document.getElementById('start-lap').addEventListener('click', () => sendCommand(API_START_LAP));
    document.getElementById('finish-lap').addEventListener('click', () => sendCommand(API_FINISH_LAP));
    document.getElementById('move-to-scene').addEventListener('click', () => sendCommand(API_MOVE_TO_SCENE));
    document.getElementById('abort-all').addEventListener('click', () => sendCommand(API_ABORT_ALL));

}

document.addEventListener('DOMContentLoaded', () => {
    initalise_buttons();

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