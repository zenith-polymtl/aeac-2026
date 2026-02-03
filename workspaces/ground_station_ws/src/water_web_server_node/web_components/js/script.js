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

function initaliseButtons() {
    // document.getElementById('mission-go').addEventListener('click', () => sendCommand(API_MISSION_GO));
}

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