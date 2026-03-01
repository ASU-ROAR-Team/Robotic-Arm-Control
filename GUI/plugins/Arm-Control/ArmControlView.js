// plugins/mission-control/ArmControlView.js
(function () {
    'use strict';

    class ArmControlView {
        constructor(container, openmct, wsUrl = "ws://localhost:8080") {
            this.container = container;
            this.openmct   = openmct;
            this.wsUrl     = wsUrl;
            this.orientationLocked = false;
            this.lockPublisher     = null;
            this.mode = "FK";

            this.jointNames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

            this.jointValues = {
                joint1: 0, joint2: 0, joint3: 0,
                joint4: 0, joint5: 0, joint6: 0
            };

            this.ikValues = { x: 50, y: 0, z: 20, yaw: 0, pitch: 0, roll: 0 };
            this.teleopDefaultCm = 1.0;

            this.fkPresets = {
                home: { joint1: 0,  joint2: 0,   joint3: 0,  joint4: 0, joint5: 0,  joint6: 0 },
                rest: { joint1: 0,  joint2: -45,  joint3: 90, joint4: 0, joint5: 45, joint6: 0 }
            };

            this.ikPresets = {
                home: { x: 50, y: 0,  z: 20, yaw: 0, pitch: 0,   roll: 0 },
                rest: { x: 30, y: 20, z: 10, yaw: 0, pitch: -45, roll: 0 }
            };

            // WebSocket state
            this.ws               = null;
            this.reconnectInterval = null;

            // ROS (optional direct ROSLIB path)
            this.ros            = null;
            this.jointPublisher = null;
            this.posePublisher  = null;

            // DOM refs populated in bindElements()
            this.sliders      = {};
            this.numberInputs = {};
            this.ikInputs     = {};
        }

        parseTeleopCommand(raw) {
            const text = String(raw || '').trim().toLowerCase();
            if (!text) return null;

            const parts = text.split(/\s+/);
            const cmd = parts[0];

            const map = {
                w: { x: +1, y: 0, z: 0 },
                s: { x: -1, y: 0, z: 0 },
                a: { x: 0, y: +1, z: 0 },
                d: { x: 0, y: -1, z: 0 },
                q: { x: 0, y: 0, z: +1 },
                e: { x: 0, y: 0, z: -1 }
            };

            if (!map[cmd]) return null;

            const cm = parts.length > 1 ? Number(parts[1]) : this.teleopDefaultCm;
            if (!Number.isFinite(cm)) return null;

            const meters = cm / 100.0;
            return {
                cmd,
                cm,
                dx: map[cmd].x * meters,
                dy: map[cmd].y * meters,
                dz: map[cmd].z * meters
            };
        }

        applyTeleopDelta(dx, dy, dz) {
            this.ikValues.x += dx * 100.0;
            this.ikValues.y += dy * 100.0;
            this.ikValues.z += dz * 100.0;

            this.publishPose();
            this.sendWSUpdate();
            this.updateIkPoseReadout();
        }

        runTeleopCommand(raw) {
            const parsed = this.parseTeleopCommand(raw);
            const statusEl = this.container.querySelector('#ikTeleopStatus');
            if (!parsed) {
                if (statusEl) {
                    statusEl.textContent = "Invalid command. Use: w/s/a/d/q/e [cm]";
                }
                return;
            }

            if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
                if (statusEl) {
                    statusEl.textContent = "WebSocket not connected.";
                }
                return;
            }

            const command = `${parsed.cmd} ${parsed.cm}`;
            this.ws.send(JSON.stringify({
                type: 'teleop_cmd',
                command
            }));

            if (statusEl) {
                statusEl.textContent =
                    `Sent '${command}' to /teleop_command`;
            }
        }

        updateIkPoseReadout() {
            const xEl = this.container.querySelector('#ikPoseX');
            const yEl = this.container.querySelector('#ikPoseY');
            const zEl = this.container.querySelector('#ikPoseZ');
            if (xEl) xEl.textContent = `${this.ikValues.x.toFixed(1)} cm`;
            if (yEl) yEl.textContent = `${this.ikValues.y.toFixed(1)} cm`;
            if (zEl) zEl.textContent = `${this.ikValues.z.toFixed(1)} cm`;
        }

        // ─── WebSocket ──────────────────────────────────────────────────────

        initWS() {
            if (this.ws && this.ws.readyState !== WebSocket.CLOSED) return;

            this.ws = new WebSocket(this.wsUrl);

            this.ws.onopen = () => {
                console.log("[ArmControlView] Connected to WS bridge");
                this.updateConnectionStatus(true);
                if (this.reconnectInterval) {
                    clearInterval(this.reconnectInterval);
                    this.reconnectInterval = null;
                }
            };

            this.ws.onmessage = (event) => {
                try {
                    const msg = JSON.parse(event.data);
                    const data = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;

                    if (msg.type === 'ik_pose' && data) {
                        if (Number.isFinite(Number(data.x))) this.ikValues.x = Number(data.x);
                        if (Number.isFinite(Number(data.y))) this.ikValues.y = Number(data.y);
                        if (Number.isFinite(Number(data.z))) this.ikValues.z = Number(data.z);
                        this.updateIkPoseReadout();
                        return;
                    }

                    console.log("[ArmControlView] RX:", msg);
                } catch (e) {
                    console.error("[ArmControlView] Failed to parse message", e);
                }
            };

            this.ws.onclose = () => {
                console.warn("[ArmControlView] Disconnected. Reconnecting in 3s...");
                this.updateConnectionStatus(false);
                this.scheduleReconnect();
            };

            this.ws.onerror = (err) => {
                console.error("[ArmControlView] WebSocket error", err);
                this.ws.close();
            };
        }

        scheduleReconnect() {
            if (this.reconnectInterval) return;
            this.reconnectInterval = setInterval(() => {
                console.log("[ArmControlView] Attempting reconnect...");
                this.initWS();
            }, 3000);
        }

        sendWSUpdate() {
            if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;

            const data = this.mode === 'FK'
                ? Object.values(this.jointValues)
                : Object.values(this.ikValues);

            if (data.some(v => isNaN(v))) return;

            this.ws.send(JSON.stringify({
                type: 'joint_cmd',
                mode: this.mode,
                data
            }));
        }

        // ─── Render ─────────────────────────────────────────────────────────

        render() {
            this.container.innerHTML = this.getHTML();
            this.statusElement = this.container.querySelector("#armStatus");
            this.statusDot     = this.container.querySelector(".status-dot");

            this.bindElements();
            this.initWS();
            this.tryConnectROS();
            this.updateModeUI();
        }

        // ─── HTML ────────────────────────────────────────────────────────────

        getHTML() {
            return `
            <div class="arm-control-container">
                <h2 class="section-header">Robotic Arm Control</h2>

                <div class="status-bar">
                    <div class="status-dot"></div>
                    <div id="armStatus">Connecting...</div>
                    <button id="modeSwitchButton">Switch to IK Mode</button>
                </div>

                <div class="controls-grid">
                    <div id="fk_container" class="joint-control">
                        <p>Adjust individual joint angles manually.</p>
                        ${this.jointNames.map(joint => `
                            <div class="joint-slider-container">
                                <label style="width:80px">${joint.toUpperCase()}</label>
                                <input type="range" id="${joint}_slider" min="-180" max="180" value="0">
                                <span id="${joint}_val_display">0°</span>
                                <input type="number" id="${joint}_number" value="0" style="display:none">
                            </div>
                        `).join('')}
                    </div>

                    <div id="ik_container" class="pose-control" style="display:none">
                        <p>IK Teleop: use teleop-style relative commands (same style as <strong>w 50</strong>).</p>
                        <div style="display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:10px;">
                            <input type="text" id="ikTeleopCommand" placeholder="w 50" style="width:140px;">
                            <button id="ikTeleopRunBtn" type="button">Run</button>
                            <span style="font-size:12px;opacity:0.8;">Format: w/s/a/d/q/e [cm]</span>
                        </div>

                        <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:10px;">
                            <label for="ikStepCm" style="font-size:12px;">Step (cm)</label>
                            <input type="number" id="ikStepCm" min="0.1" step="0.1" value="${this.teleopDefaultCm}" style="width:80px;">
                            <button id="ikWBtn" type="button">W +X</button>
                            <button id="ikSBtn" type="button">S -X</button>
                            <button id="ikABtn" type="button">A +Y</button>
                            <button id="ikDBtn" type="button">D -Y</button>
                            <button id="ikQBtn" type="button">Q +Z</button>
                            <button id="ikEBtn" type="button">E -Z</button>
                        </div>

                        <div style="font-size:12px;margin-bottom:6px;">
                            Current IK target:&nbsp;
                            X=<span id="ikPoseX">${this.ikValues.x.toFixed(1)} cm</span>,
                            Y=<span id="ikPoseY">${this.ikValues.y.toFixed(1)} cm</span>,
                            Z=<span id="ikPoseZ">${this.ikValues.z.toFixed(1)} cm</span>
                        </div>

                        <div id="ikTeleopStatus" style="font-size:12px;opacity:0.9;min-height:18px;"></div>

                        <div style="font-size:12px;opacity:0.7;">
                            Orientation fields (yaw/pitch/roll) remain unchanged while using IK teleop moves.
                        </div>
                    </div>
                </div>

                <div class="preset-buttons">
                    <button class="preset-button" id="homeBtn">Home Position</button>
                    <button class="preset-button" id="restBtn">Rest Position</button>
                    <button class="preset-button" id="lockOrientationBtn">Lock Orientation: OFF</button>
                </div>
            </div>
            `;
        }

        // ─── UI ──────────────────────────────────────────────────────────────

        updateModeUI() {
            const fk  = this.container.querySelector('#fk_container');
            const ik  = this.container.querySelector('#ik_container');
            const btn = this.container.querySelector('#modeSwitchButton');

            if (this.mode === 'FK') {
                fk.style.display  = 'flex';
                ik.style.display  = 'none';
                btn.innerText     = 'Switch to IK Mode';
            } else {
                fk.style.display  = 'none';
                ik.style.display  = 'flex';
                btn.innerText     = 'Switch to FK Mode';
            }
        }

        updateConnectionStatus(connected) {
            if (this.statusDot) this.statusDot.classList.toggle('connected', connected);
            if (this.statusElement) {
                this.statusElement.innerText = connected ? 'WS: Connected' : 'WS: Disconnected';
            }
        }

        bindElements() {
            // FK sliders
            this.jointNames.forEach(joint => {
                const s = this.container.querySelector(`#${joint}_slider`);
                const n = this.container.querySelector(`#${joint}_number`);

                this.sliders[joint]      = s;
                this.numberInputs[joint] = n;

                const handler = val => {
                    val = Number(val);
                    this.jointValues[joint] = val;
                    s.value = val;
                    n.value = val;
                    this.container.querySelector(`#${joint}_val_display`).innerText = `${val}°`;

                    if (this.mode === 'FK') this.publishJointStates();
                    this.sendWSUpdate();
                };

                s.oninput = () => handler(s.value);
                n.oninput = () => handler(n.value);
            });

            const ikCommandInput = this.container.querySelector('#ikTeleopCommand');
            const ikRunBtn       = this.container.querySelector('#ikTeleopRunBtn');
            const ikStepInput    = this.container.querySelector('#ikStepCm');

            if (ikStepInput) {
                ikStepInput.oninput = () => {
                    const value = Number(ikStepInput.value);
                    if (Number.isFinite(value) && value > 0) {
                        this.teleopDefaultCm = value;
                    }
                };
            }

            if (ikRunBtn) {
                ikRunBtn.onclick = () => this.runTeleopCommand(ikCommandInput ? ikCommandInput.value : '');
            }

            if (ikCommandInput) {
                ikCommandInput.onkeydown = (event) => {
                    if (event.key === 'Enter') {
                        this.runTeleopCommand(ikCommandInput.value);
                    }
                };
            }

            const stepButtons = [
                ['#ikWBtn', 'w'],
                ['#ikSBtn', 's'],
                ['#ikABtn', 'a'],
                ['#ikDBtn', 'd'],
                ['#ikQBtn', 'q'],
                ['#ikEBtn', 'e']
            ];

            stepButtons.forEach(([selector, cmd]) => {
                const button = this.container.querySelector(selector);
                if (!button) return;
                button.onclick = () => {
                    this.runTeleopCommand(`${cmd} ${this.teleopDefaultCm}`);
                };
            });

            // Mode switch
            this.container.querySelector('#modeSwitchButton').onclick = () => {
                this.mode = this.mode === 'FK' ? 'IK' : 'FK';
                this.updateModeUI();
            };

            // Preset buttons
            this.container.querySelector('#homeBtn').onclick = () => this.applyPreset('home');
            this.container.querySelector('#restBtn').onclick = () => this.applyPreset('rest');

            this.updateIkPoseReadout();

            // Lock Orientation Toggle
            this.container.querySelector('#lockOrientationBtn').onclick = () => {
                this.orientationLocked = !this.orientationLocked;

                const btn = this.container.querySelector('#lockOrientationBtn');
                const state = this.orientationLocked ? "ON" : "OFF";

                btn.innerText = `Lock Orientation: ${state}`;

                this.publishLockState(state);
                this.sendWSLockState(state);
                };
        }

        // ─── Presets ────────────────────────────────────────────────────────

        applyPreset(type) {
            if (this.mode === 'FK') {
                const preset = this.fkPresets[type];
                this.jointNames.forEach(j => {
                    const v = preset[j];
                    this.jointValues[j] = v;
                    this.sliders[j].value      = v;
                    this.numberInputs[j].value = v;
                    this.container.querySelector(`#${j}_val_display`).innerText = `${v}°`;
                });
                this.publishJointStates();
            } else {
                const preset = this.ikPresets[type];
                Object.keys(preset).forEach(k => {
                    this.ikValues[k]     = preset[k];
                    this.ikInputs[k].value = preset[k];
                });
                this.publishPose();
            }

            this.sendWSUpdate();
        }

        // ─── ROS (optional direct ROSLIB path) ──────────────────────────────

        tryConnectROS() {
            if (typeof ROSLIB === 'undefined') {
                console.info("[ArmControlView] ROSLIB not found — using WS bridge only");
                return;
            }

            this.ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

            this.ros.on('connection', () => {
                console.log("[ArmControlView] ROSLIB connected");
                this.setupROS();
            });

            this.ros.on('error', (e) => console.error("[ArmControlView] ROSLIB error", e));
            this.ros.on('close', ()  => console.warn("[ArmControlView] ROSLIB closed"));
        }

        setupROS() {
            this.jointPublisher = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fk_joint_states',
                messageType: 'sensor_msgs/JointState'
            });

            this.posePublisher = new ROSLIB.Topic({
                ros: this.ros,
                name: '/ik_target_pose',
                messageType: 'std_msgs/Float64MultiArray'
            });
            this.lockPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: '/lock_orientation',
            messageType: 'std_msgs/String'
                });
        }

        publishJointStates() {
            if (!this.jointPublisher) return;
            const pos = this.jointNames.map(j => this.jointValues[j] * Math.PI / 180);
            this.jointPublisher.publish({ name: this.jointNames, position: pos });
        }

        publishPose() {
            if (!this.posePublisher) return;
            this.posePublisher.publish({ data: Object.values(this.ikValues) });
        }
        publishLockState(state) {
        if (!this.lockPublisher) return;

        this.lockPublisher.publish({
            data: state
        });
        

        console.log("[ArmControlView] Lock Orientation:", state);
    }
    sendWSLockState(state) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        console.warn("[ArmControlView] WS not connected");
        return;
    }

    const message = {
        type: "lock_orientation",
        data: state
    };

    console.log("[ArmControlView] Sending WS:", message);
    this.ws.send(JSON.stringify(message));
}

        // ─── Destroy ────────────────────────────────────────────────────────

        destroy() {
            if (this.reconnectInterval) clearInterval(this.reconnectInterval);
            if (this.ws)  this.ws.close();
            if (this.ros) this.ros.close();
            console.log('[ArmControlView] destroyed');
        }
    }

    window.ArmControlView = ArmControlView;

})();