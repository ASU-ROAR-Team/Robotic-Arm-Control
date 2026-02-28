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
                    // Handle any inbound messages from the bridge if needed
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
                        <p>Define the target end-effector pose.</p>
                        <div style="display:grid;grid-template-columns:1fr 1fr;gap:15px">
                            ${['x','y','z','yaw','pitch','roll'].map(k => `
                                <div>
                                    <strong>${k.toUpperCase()}</strong>
                                    <input type="number" id="${k}_input" value="${this.ikValues[k]}" style="width:70px">
                                    <span>${['x','y','z'].includes(k) ? 'cm' : '°'}</span>
                                </div>
                            `).join('')}
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

            // IK inputs
            ['x','y','z','yaw','pitch','roll'].forEach(k => {
                const el = this.container.querySelector(`#${k}_input`);
                this.ikInputs[k] = el;

                el.oninput = () => {
                    this.ikValues[k] = Number(el.value);
                    if (this.mode === 'IK') this.publishPose();
                    this.sendWSUpdate();
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