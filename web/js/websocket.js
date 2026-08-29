/**
 * WebSocket Telemetry Manager for Quadruped Robot Dashboard
 */

class TelemetryWS {
    constructor() {
        this.socket = null;
        this.callbacks = [];
        this.onConnectCallbacks = [];
        this.onDisconnectCallbacks = [];
        this.reconnectTimer = null;
        
        // Auto-detect address based on the current window location
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host || 'localhost:8000';
        this.url = `${protocol}//${host}/ws`;
    }

    connect() {
        console.log(`[WS] Connecting to telemetry server: ${this.url}`);
        
        try {
            this.socket = new WebSocket(this.url);
            
            this.socket.onopen = () => {
                console.log("[WS] Connected successfully.");
                if (this.reconnectTimer) {
                    clearTimeout(this.reconnectTimer);
                    this.reconnectTimer = null;
                }
                this.messageCount = 0;
                this.onConnectCallbacks.forEach(cb => cb());
            };
            
            this.socket.onclose = (event) => {
                console.log(`[WS] Connection closed. Code: ${event.code}, Reason: ${event.reason || 'None'}`);
                this.onDisconnectCallbacks.forEach(cb => cb());
                this.triggerReconnect();
            };
            
            this.socket.onerror = (err) => {
                console.error("[WS] Socket error:", err);
            };
            
            this.socket.onmessage = (event) => {
                try {
                    const msg = jsonParse(event.data);
                    if (msg) {
                        this.messageCount = (this.messageCount || 0) + 1;
                        if (this.messageCount === 1) {
                            console.log("[WS] First telemetry packet received:", msg);
                        } else if (this.messageCount % 100 === 0) {
                            console.log(`[WS] Telemetry streaming active (${this.messageCount} frames received)`);
                        }
                        this.callbacks.forEach(cb => cb(msg));
                    }
                } catch (e) {
                    console.error("[WS] Error parsing message:", e);
                }
            };
        } catch (e) {
            console.error("[WS] Connection failed:", e);
            this.triggerReconnect();
        }
    }

    triggerReconnect() {
        if (!this.reconnectTimer) {
            this.reconnectTimer = setTimeout(() => {
                this.reconnectTimer = null;
                this.connect();
            }, 2000);
        }
    }

    send(type, payload) {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            this.socket.send(JSON.stringify({ type, payload }));
        }
    }

    /**
     * Send primary gait commands (enable/disable, steering)
     */
    sendCommand(gaitEnabled, steeringVal) {
        this.send("command", {
            gait_enabled: gaitEnabled,
            steering: steeringVal
        });
    }

    /**
     * Send CPG parameter tweaks (e.g. section='cpg', key='a_M', value=1.2)
     */
    sendConfigUpdate(section, key, value) {
        this.send("config", {
            [section]: {
                [key]: parseFloat(value)
            }
        });
    }

    // Callbacks registrations
    registerOnTelemetry(cb) {
        this.callbacks.push(cb);
    }

    registerOnConnect(cb) {
        this.onConnectCallbacks.push(cb);
    }

    registerOnDisconnect(cb) {
        this.onDisconnectCallbacks.push(cb);
    }
}

// Utility to parse JSON safely without throwing
function jsonParse(str) {
    try {
        return JSON.parse(str);
    } catch (e) {
        return null;
    }
}
