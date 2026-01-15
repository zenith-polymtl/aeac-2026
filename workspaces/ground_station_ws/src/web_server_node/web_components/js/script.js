function initalise_buttons() {
    document.getElementById('/mission/go').onclick = async () => {
        const response = await fetch('/api/mission/go');
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        const data = await response.json();
        console.log(data.message);
    }

    document.getElementById('/mission/lap').onclick = async () => {
        const response = await fetch('/api/mission/lap');
        if (!response.ok) {
            throw new Error('Network response was not ok');
        }
        const data = await response.json();
        console.log(data.message);
    }
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
            
            // Update UI
            document.getElementById("status").textContent = data.status;
            document.getElementById("battery").textContent = data.battery + "%";
            document.getElementById("clients").textContent = data.clients;
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