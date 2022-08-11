import { writable, get } from 'svelte/store';
import ReconnectingWebSocket from 'reconnecting-websocket'

let hidden; //Key for the Page Visibility API.  The key depends on what browser is being used, so we need to check that and assign it
            //This is used to access whether or not the user has the page visible on the screen, to determine if we should send updates to the controller
if (typeof document.hidden !== "undefined") {
    hidden = "hidden";
} else if (typeof document.msHidden !== "undefined") {
    hidden = "msHidden";
} else if (typeof document.webkitHidden !== "undefined") {
    hidden = "webkitHidden";
}

const socket = new ReconnectingWebSocket("ws://10.0.2.30:8082");
socket.addEventListener('message', onMessage);

function tick() {
    if (socket.readyState !== WebSocket.OPEN) {
        diagnostics.set(["Not connected to vehicle"])
    }

    if (!document[hidden]) {
        try {
            let stateObj = {
                type: "periodic",
                forward: get(forward),
                disable_sensors: get(overrideSensors)
            };
            socket.send(JSON.stringify(stateObj));
        } catch {} // This just means we are not connected.
    }

}

/**
 * Called when we receive a new message from the vehicle
 * @param {*} event 
 */
function onMessage(event) {
    let message = JSON.parse(event.data);

    // Update all state from message
    diagnostics.set(message["diagnostics"]);
    speed.set(message["speed_setpoint"]);
    currentSpeed.set(message["current_speed"]);
    voltage.set(message["voltage"]);
    draw.set(message["motor_current_draw"]);
    distance.set(message["distance"]);
    batteryState.set(message["battery_state"]);
    drawState.set(message["motor_current_state"]);
    distanceState.set(message["distance_state"]);

}

export function sendStopCommand() {
    try {
        let commandObj = {
            type: "command",
            command: "stop"
        };
        socket.send(JSON.stringify(commandObj));
    } catch {}
}

export function sendSpeedUpCommand() {
    try {
        let commandObj = {
            type: "command",
            command: "speed_up"
        };
        socket.send(JSON.stringify(commandObj));
    } catch {}
}

export function sendSpeedDownCommand() {
    try {
        let commandObj = {
            type: "command",
            command: "speed_down"
        };
        socket.send(JSON.stringify(commandObj));
    } catch {}
}


export const screenWidth = writable(1390);
export const screenHeight = writable(800);
export const diagnostics = writable([]);
export const speed = writable(0);
export const overrideSensors = writable(false);
export const forward=writable(true);

//-----

export const currentSpeed = writable(0);
export const voltage = writable(0);
export const draw = writable(0);
export const distance = writable(0);

export const batteryState = writable(2);
export const drawState = writable(0);
export const distanceState = writable(0);

setInterval(tick, 250);