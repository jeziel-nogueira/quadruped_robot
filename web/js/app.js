/**
 * Main Application Orchestrator for Quadruped Dashboard
 */

document.addEventListener("DOMContentLoaded", () => {
    // 1. Initialize Components
    const ws = new TelemetryWS();
    
    // Canvas elements
    const cpgCanvas = document.getElementById("chart-cpg");
    const imuCanvas = document.getElementById("chart-imu");
    const robotCanvas = document.getElementById("robot-canvas");
    
    // Instantiate plots
    const cpgPlot = new TimeSeriesPlot(cpgCanvas, 120, ["#00f5d4", "#00bbf9", "#ff9f1c", "#f15bb5"]);
    const imuPlot = new TimeSeriesPlot(imuCanvas, 120, ["#ff9f1c", "#00f5d4"]);

    // State parameters
    let gaitEnabled = false;
    let steeringValue = 0.0;
    let configSynced = false;

    // 2. WebSocket Connection Callbacks
    const connBadge = document.getElementById("conn-badge");
    const badgeText = connBadge.querySelector(".badge-text");
    
    ws.registerOnConnect(() => {
        connBadge.className = "status-badge connected";
        badgeText.textContent = "Online";
        ws.sendCommand(gaitEnabled, steeringValue);
    });

    ws.registerOnDisconnect(() => {
        connBadge.className = "status-badge disconnected";
        badgeText.textContent = "Offline";
    });

    let telemetryCount = 0;

    // 3. Receive Live Telemetry
    ws.registerOnTelemetry((msg) => {
        if (msg.type !== "telemetry") return;
        
        const t = msg.payload;
        telemetryCount++;

        if (telemetryCount === 1) {
            console.log("[App] Processing first telemetry payload:", t);
        }
        
        // Update battery indicators
        const batteryPercent = Math.min(100, Math.max(0, ((t.battery - 7.0) / 1.4) * 100));
        document.getElementById("battery-bar").style.width = `${batteryPercent}%`;
        document.getElementById("battery-voltage").textContent = `${t.battery.toFixed(2)}V`;
        
        // Update distance sensor card
        const distLbl = document.getElementById("lbl-distance");
        const distBar = document.getElementById("distance-bar");
        
        if (t.distance >= 990.0) {
            distLbl.textContent = "Clear";
            distBar.style.width = "100%";
            distBar.className = "distance-progress-bar";
            distLbl.className = "val";
        } else {
            distLbl.textContent = `${t.distance.toFixed(0)} cm`;
            const distPercent = Math.min(100, (t.distance / 80.0) * 100);
            distBar.style.width = `${distPercent}%`;
            
            if (t.distance < 15.0) {
                distBar.className = "distance-progress-bar warning";
                distLbl.className = "val warning";
            } else {
                distBar.className = "distance-progress-bar";
                distLbl.className = "val";
            }
        }

        // Update attitude labels
        document.getElementById("lbl-pitch").textContent = `${t.imu.pitch.toFixed(1)}°`;

        // Update charts
        // Plot CPG excitatory outputs (u_Se) for all 4 legs
        if (t.cpg_states && t.cpg_states.u_Se) {
            cpgPlot.addPoint(t.cpg_states.u_Se, t.timestamp.toString());
        } else if (telemetryCount === 1) {
            console.warn("[App] No cpg_states.u_Se found in payload:", t);
        }

        // Plot IMU Yaw and Pitch
        if (t.imu) {
            imuPlot.addPoint([t.imu.pitch, t.imu.yaw], t.timestamp.toString());
        }

        // Draw visual skeletal robot
        if (t.joint_angles) {
            drawRobotKinematics(t.joint_angles);
        }

        // Sync slider values on initial load
        if (t.config && !configSynced) {
            syncConfigSliders(t.config);
            configSynced = true;
            console.log("[App] Config sliders synchronized with backend config.");
        }
        
        // Sync gait switch UI state with robot reality
        if (t.commands) {
            updatePowerButtonState(t.commands.gait_enabled);
        }
    });

    // Connect to server
    ws.connect();

    // 4. Synchronize UI Config Sliders
    function syncConfigSliders(config) {
        // CPG parameters mapping
        const cpgParams = ["tau_m", "a_M", "b_M", "c_M"];
        cpgParams.forEach(param => {
            const val = config.cpg[param];
            const slider = document.getElementById(`param-${param}`);
            if (slider) {
                slider.value = val;
                document.getElementById(`val-${param}`).textContent = val.toFixed(2);
            }
        });
        
        // Feedback gains mapping
        const fbParams = ["K_y", "K_p", "K_u"];
        fbParams.forEach(param => {
            const val = config.feedback[param];
            const slider = document.getElementById(`param-${param}`);
            if (slider) {
                slider.value = val;
                document.getElementById(`val-${param}`).textContent = val.toFixed(2);
            }
        });
    }

    // 5. Parameter Slider Handlers
    const sliders = document.querySelectorAll("input[type='range']");
    sliders.forEach(slider => {
        slider.addEventListener("input", (e) => {
            const id = e.target.id;
            const val = parseFloat(e.target.value);
            const valLbl = document.getElementById(`val-${id.replace("param-", "")}`);
            if (valLbl) {
                valLbl.textContent = val.toFixed(2);
            }

            // Determine if parameter is CPG math or feedback gains
            const paramName = id.replace("param-", "");
            if (["tau_m", "a_M", "b_M", "c_M"].includes(paramName)) {
                ws.sendConfigUpdate("cpg", paramName, val);
            } else if (["K_y", "K_p", "K_u"].includes(paramName)) {
                ws.sendConfigUpdate("feedback", paramName, val);
            }
        });
    });

    // 6. Power Switch Toggle
    const btnPower = document.getElementById("btn-power");
    
    function updatePowerButtonState(enabled) {
        gaitEnabled = enabled;
        if (enabled) {
            btnPower.className = "btn power-btn on";
            btnPower.querySelector("span").textContent = "Active";
        } else {
            btnPower.className = "btn power-btn off";
            btnPower.querySelector("span").textContent = "Standby";
        }
    }

    btnPower.addEventListener("click", () => {
        gaitEnabled = !gaitEnabled;
        updatePowerButtonState(gaitEnabled);
        ws.sendCommand(gaitEnabled, steeringValue);
    });

    // 7. Virtual Joystick Steering (Horizontal Spring Slider)
    const joyPad = document.getElementById("joystick-pad");
    const joyKnob = document.getElementById("joystick-knob");
    const joyXLbl = document.getElementById("joy-x");
    let isDragging = false;
    let padRect = null;

    function handleJoystickMove(clientX) {
        if (!padRect) return;
        
        const knobWidth = joyKnob.clientWidth;
        const halfKnob = knobWidth / 2;
        let localX = clientX - padRect.left;
        
        // Clamp to pad bounds
        localX = Math.max(halfKnob, Math.min(padRect.width - halfKnob, localX));
        
        // Set knob position
        joyKnob.style.left = `${localX - halfKnob}px`;
        
        // Calculate normalized steering value (-1.0 to 1.0)
        const center = padRect.width / 2;
        const maxDelta = center - halfKnob;
        const currentDelta = localX - center;
        steeringValue = currentDelta / maxDelta;
        
        joyXLbl.textContent = steeringValue.toFixed(2);
        ws.sendCommand(gaitEnabled, steeringValue);
    }

    function resetJoystick() {
        if (!padRect) return;
        joyKnob.style.left = `calc(50% - ${joyKnob.clientWidth / 2}px)`;
        steeringValue = 0.0;
        joyXLbl.textContent = "0.00";
        ws.sendCommand(gaitEnabled, steeringValue);
    }

    joyKnob.addEventListener("mousedown", (e) => {
        isDragging = true;
        padRect = joyPad.getBoundingClientRect();
        e.preventDefault();
    });

    document.addEventListener("mousemove", (e) => {
        if (!isDragging) return;
        handleJoystickMove(e.clientX);
    });

    document.addEventListener("mouseup", () => {
        if (isDragging) {
            isDragging = false;
            resetJoystick();
        }
    });

    // Touch support for mobile browsers
    joyKnob.addEventListener("touchstart", (e) => {
        isDragging = true;
        padRect = joyPad.getBoundingClientRect();
        if (e.cancelable) e.preventDefault();
    }, { passive: false });

    document.addEventListener("touchmove", (e) => {
        if (!isDragging) return;
        if (e.touches && e.touches.length > 0) {
            handleJoystickMove(e.touches[0].clientX);
            if (e.cancelable) e.preventDefault();
        }
    }, { passive: false });

    document.addEventListener("touchend", () => {
        if (isDragging) {
            isDragging = false;
            resetJoystick();
        }
    });

    // 8. HTML5 Canvas Quadruped Kinematics drawing
    const ctx = robotCanvas.getContext("2d");
    
    function drawRobotKinematics(jointAngles) {
        if (!jointAngles || jointAngles.length < 12) return;

        // Auto resize canvas
        const clientW = robotCanvas.clientWidth || 500;
        const clientH = robotCanvas.clientHeight || 300;
        const width = robotCanvas.width = clientW * window.devicePixelRatio;
        const height = robotCanvas.height = clientH * window.devicePixelRatio;
        ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        const w = clientW;
        const h = clientH;

        // Clear
        ctx.clearRect(0, 0, w, h);

        // Grid lines (subtle background)
        ctx.strokeStyle = "rgba(255, 255, 255, 0.02)";
        ctx.lineWidth = 1;
        const gridSize = 30;
        for (let x = 0; x < w; x += gridSize) {
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
        }
        for (let y = 0; y < h; y += gridSize) {
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
        }

        // Draw Ground Line
        ctx.strokeStyle = "rgba(255, 255, 255, 0.1)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(20, h - 35);
        ctx.lineTo(w - 20, h - 35);
        ctx.stroke();

        // Robot configuration dimensions (drawing scale)
        const centerX = w / 2;
        const centerY = h / 2 - 15;
        const bodyLength = 120; // in pixels
        const thighLen = 45;
        const calfLen = 45;

        // Shoulder attachment points (2D side view projection)
        const backShoulder = { x: centerX - bodyLength / 2, y: centerY };
        const frontShoulder = { x: centerX + bodyLength / 2, y: centerY };

        // Leg index maps: 0=FL, 1=FR, 2=HL, 3=HR
        // Let's draw background legs first (FR, HL) to create 3D depth, then body, then foreground legs (FL, HR)
        const legDrawOrder = [1, 2, 0, 3];

        legDrawOrder.forEach(legIdx => {
            const isFront = (legIdx === 0 || legIdx === 1);
            const isLeft = (legIdx === 0 || legIdx === 2);
            
            // Shoulder position
            const shoulder = isFront ? frontShoulder : backShoulder;
            
            // Get angles (radians)
            const baseAngleIdx = legIdx * 3;
            const roll = jointAngles[baseAngleIdx];
            const pitch = jointAngles[baseAngleIdx + 1];
            const knee = jointAngles[baseAngleIdx + 2];
            
            // In our simple 2D side view, pitch is rotation relative to downward vertical
            // Hip Joint is shoulder
            // Thigh end (Knee joint)
            const kneeX = shoulder.x + thighLen * Math.sin(pitch);
            const kneeY = shoulder.y + thighLen * Math.cos(pitch);
            
            // Foot end
            const footX = kneeX + calfLen * Math.sin(pitch + knee);
            const footY = kneeY + calfLen * Math.cos(pitch + knee);

            // Styling variables
            const isForeground = (legIdx === 0 || legIdx === 3);
            const color = isFront ? "#00f5d4" : "#ff9f1c";
            
            ctx.save();
            ctx.lineCap = "round";
            ctx.lineJoin = "round";
            
            if (isForeground) {
                ctx.strokeStyle = color;
                ctx.lineWidth = 6;
                ctx.shadowColor = color;
                ctx.shadowBlur = 4;
            } else {
                ctx.strokeStyle = color;
                ctx.lineWidth = 4;
                ctx.globalAlpha = 0.35; // fade out background legs
            }

            // Draw Thigh Link
            ctx.beginPath();
            ctx.moveTo(shoulder.x, shoulder.y);
            ctx.lineTo(kneeX, kneeY);
            ctx.stroke();

            // Draw Calf Link
            ctx.beginPath();
            ctx.moveTo(kneeX, kneeY);
            ctx.lineTo(footX, footY);
            ctx.stroke();

            // Draw Joints as circles
            ctx.fillStyle = isForeground ? "#fff" : "rgba(255,255,255,0.5)";
            
            // Knee Joint dot
            ctx.beginPath();
            ctx.arc(kneeX, kneeY, isForeground ? 4 : 3, 0, 2 * Math.PI);
            ctx.fill();
            
            // Foot dot (tip of leg)
            ctx.fillStyle = isForeground ? color : "rgba(255,255,255,0.3)";
            ctx.beginPath();
            ctx.arc(footX, footY, isForeground ? 5 : 4, 0, 2 * Math.PI);
            ctx.fill();

            ctx.restore();
        });

        // Draw Main Body Capsule
        ctx.save();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 14;
        ctx.lineCap = "round";
        ctx.shadowColor = "#00bbf9";
        ctx.shadowBlur = 10;
        
        ctx.beginPath();
        ctx.moveTo(backShoulder.x, backShoulder.y);
        ctx.lineTo(frontShoulder.x, frontShoulder.y);
        ctx.stroke();
        
        // Draw inner metal line
        ctx.strokeStyle = "#08090f";
        ctx.lineWidth = 4;
        ctx.shadowBlur = 0;
        ctx.beginPath();
        ctx.moveTo(backShoulder.x + 5, backShoulder.y);
        ctx.lineTo(frontShoulder.x - 5, frontShoulder.y);
        ctx.stroke();

        // Shoulder nodes
        ctx.fillStyle = "#00bbf9";
        ctx.beginPath();
        ctx.arc(backShoulder.x, backShoulder.y, 8, 0, 2 * Math.PI);
        ctx.arc(frontShoulder.x, frontShoulder.y, 8, 0, 2 * Math.PI);
        ctx.fill();
        
        ctx.restore();
    }
});
